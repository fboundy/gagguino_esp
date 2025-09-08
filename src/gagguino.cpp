/**
 * @file gagguino.cpp
 * @brief Gagguino firmware: ESP32 control for Gaggia Classic.
 *
 * High‑level responsibilities:
 * - Temperature control using a MAX31865 RTD amplifier (PT100) and PID.
 * - Heater PWM drive and optional TRIAC phase‑angle dimming for pump control.
 * - Flow, pressure and shot timing measurement with debounced ISR counters.
 * - Wi‑Fi + MQTT with Home Assistant discovery for control/telemetry.
 * - Robust OTA (ArduinoOTA) that throttles other work while an update runs.
 *
 * Hardware pins (ESP32 default board mapping):
 * - FLOW_PIN (26)   : Flow sensor input (interrupt on CHANGE)
 * - ZC_PIN (25)     : AC zero‑cross detect (interrupt on RISING)
 * - HEAT_PIN (27)   : Boiler relay/SSR output (PWM windowing)
 * - AC_SENS (14)    : Steam switch sense (digital input)
 * - TRIAC_PIN (17)  : TRIAC gate output for pump phase control
 * - MAX_CS (16)     : MAX31865 SPI chip‑select
 * - PRESS_PIN (35)  : Analog pressure sensor input
 */
#include "gagguino.h"

#include <Adafruit_MAX31865.h>
#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include <cstdarg>
#include <ctype.h>

#include "secrets.h"  // WIFI_*, MQTT_*

#define VERSION "7.0"
#define STARTUP_WAIT 1000
#define SERIAL_BAUD 115200

/**
 * @brief Lightweight printf‑style logger to the serial console.
 */
static inline void LOG(const char* fmt, ...) {
    static char buf[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.println(buf);
}

namespace {
constexpr int FLOW_PIN = 26, ZC_PIN = 25, HEAT_PIN = 27, AC_SENS = 14, TRIAC_PIN = 17;
constexpr int MAX_CS = 16;
constexpr int PRESS_PIN = 35;

constexpr unsigned long PRESS_CYCLE = 100, PID_CYCLE = 500, PWM_CYCLE = 500, LOG_CYCLE = 2000;
constexpr unsigned long MQTT_IDLE_CYCLE = 10000;  // ms between publishes when idle
constexpr unsigned long MQTT_SHOT_CYCLE = 1000;   // ms between publishes during a shot

// Brew & Steam setpoint limits
constexpr float BREW_MIN = 90.0f, BREW_MAX = 99.0f;
constexpr float STEAM_MIN_C = 145.0f, STEAM_MAX_C = 155.0f;

// Default steam setpoint (within limits)
constexpr float STEAM_DEFAULT = 152.0f;

constexpr float RREF = 430.0f, RNOMINAL = 100.0f;
// Default PID params (overridable via MQTT number entities)
// Default PID parameters tuned for stability
// Kp: 15–16 [out/°C]
// Ki: 0.3–0.5 [out/(°C·s)] → start at 0.35
// Kd: 50–70 [out·s/°C] → start at 60
// guard: ±8–±12% integral clamp on 0–100% heater
constexpr float P_GAIN_TEMP = 15.0f, I_GAIN_TEMP = 0.35f, D_GAIN_TEMP = 60.0f,
                WINDUP_GUARD_TEMP = 10.0f;

constexpr float PRESS_TOL = 0.4f, PRESS_GRAD = 0.00903f, PRESS_INT_0 = -4.0f;
constexpr int PRESS_BUFF_SIZE = 14;
constexpr float PRESS_THRESHOLD = 9.0f;

// FLOW_CAL in mL per pulse (1 cc == 1 mL)
constexpr float FLOW_CAL = 0.246f;
constexpr unsigned long PULSE_MIN = 10;

constexpr unsigned ZC_MIN = 4;
constexpr unsigned long ZC_WAIT = 2000, ZC_OFF = 1000, SHOT_RESET = 30000;
constexpr unsigned long AC_WAIT = 100;
constexpr int STEAM_MIN = 20;

const bool streamData = true, debugPrint = true;
// TRIAC dimmer control (RobotDyn module)
constexpr bool TRIAC_ENABLED = true;  // set to true if using dimmer module
constexpr int MAINS_HZ = 50;          // 50 or 60
constexpr unsigned long HALF_CYCLE_US = (1000000UL / (2 * MAINS_HZ));
constexpr unsigned int TRIAC_PULSE_US = 120;  // microseconds
}  // namespace

// ---------- Devices / globals ----------
namespace {
Adafruit_MAX31865 max31865(MAX_CS);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Temps / PID
float currentTemp = 0.0f, lastTemp = 0.0f, pvFiltTemp = 0.0f;
float brewSetpoint = 95.0f;           // HA-controllable (90–99)
float steamSetpoint = STEAM_DEFAULT;  // HA-controllable (145–155)
float setTemp = brewSetpoint;         // active target (brew or steam)
float iStateTemp = 0.0f, heatPower = 0.0f;
// Live-tunable PID parameters (default to constexprs above)
float pGainTemp = P_GAIN_TEMP, iGainTemp = I_GAIN_TEMP, dGainTemp = D_GAIN_TEMP,
      windupGuardTemp = WINDUP_GUARD_TEMP;
int heatCycles = 0;
bool heaterState = false;
bool heaterEnabled = true;  // HA switch default ON at boot
// TRIAC: manual pump power percentage 0..100, controlled via HA number
float pumpPowerPct = 0.0f;
// TRIAC scheduling state
volatile unsigned long lastZcMicros = 0;
volatile bool zcFlagTriac = false;
bool triacPulsePending = false;
bool triacPulseOn = false;
unsigned long triacFireTimeUs = 0;
unsigned long triacPulseEndUs = 0;

// Pressure
int rawPress = 0;
float lastPress = 0.0f, pressNow = 0.0f, pressSum = 0.0f, pressGrad = PRESS_GRAD,
      pressInt = PRESS_INT_0;
float pressBuff[PRESS_BUFF_SIZE] = {0};
uint8_t pressBuffIdx = 0;

// Time/shot
unsigned long nLoop = 0, currentTime = 0, lastPidTime = 0, lastPwmTime = 0, lastMqttTime = 0,
              lastLogTime = 0;
;
volatile unsigned long lastPulseTime = 0;
unsigned long shotStart = 0, startTime = 0;
float shotTime = 0;  //

// Flow / flags
volatile unsigned long pulseCount = 0;
volatile unsigned long zcCount = 0;
volatile unsigned long lastZcTime = 0;
int vol = 0, preFlowVol = 0, shotVol = 0;
unsigned int lastVol = 0;
bool prevSteamFlag = false, ac = false;
int acCount = 0;
bool shotFlag = false, preFlow = false, steamFlag = false, setupComplete = false, debugData = false;

// OTA
static bool otaInitialized = false;
static bool otaActive = false;  // minimize other work while OTA is in progress

// MQTT diagnostics
static IPAddress g_mqttIp;
static bool g_mqttIpResolved = false;

// ---------- HA Discovery identity / topics ----------
const char* DISCOVERY_PREFIX = "homeassistant";
// If your broker ACLs need a username prefix, change this:
const char* STATE_BASE = "gaggia_classic";

char dev_id[32] = {0};
char uid_suffix[16] = {0};

// Sensor state topics
char t_shotvol_state[96], t_settemp_state[96], t_curtemp_state[96], t_press_state[96],
    t_shottime_state[96], t_ota_state[96];
// Switch state/command topics
char t_heater_state[96], t_heater_cmd[96];
// TRIAC pump power number topics
char t_pumppower_state[96], t_pumppower_cmd[96];
// Diagnostics state topics
char t_accnt_state[96], t_zccnt_state[96], t_pulsecnt_state[96];
// Binary sensor state topics
char t_shot_state[96], t_preflow_state[96], t_steam_state[96];
// Number entities (brew/steam setpoint) command/state topics
char t_brewset_state[96], t_brewset_cmd[96];
char t_steamset_state[96], t_steamset_cmd[96];
// PID number entity topics
char t_pidp_state[96], t_pidp_cmd[96];
char t_pidi_state[96], t_pidi_cmd[96];
char t_pidd_state[96], t_pidd_cmd[96];
char t_pidg_state[96], t_pidg_cmd[96];
// Sensor “mirror” of setpoints for verification
char t_brewset_sensor_state[96], t_steamset_sensor_state[96];

// Config topics
char c_shotvol[128], c_settemp[128], c_curtemp[128], c_press[128], c_shottime[128], c_ota[128];
char c_accnt[128], c_zccnt[128], c_pulsecnt[128];
char c_pidp_number[128], c_pidi_number[128], c_pidd_number[128], c_pidg_number[128];
char c_shot[128], c_preflow[128], c_steam[128];
char c_brewset_number[128], c_steamset_number[128];
char c_brewset_sensor[128], c_steamset_sensor[128];
// Switch config
char c_heater[128];
// TRIAC pump power number config
char c_pumppower_number[128];

// ---------- helpers ----------
/**
 * @brief Convert Wi‑Fi status to a readable string.
 */
static inline const char* wifiStatusName(wl_status_t s) {
    switch (s) {
        case WL_IDLE_STATUS:
            return "IDLE";
        case WL_NO_SSID_AVAIL:
            return "NO_SSID";
        case WL_SCAN_COMPLETED:
            return "SCAN_DONE";
        case WL_CONNECTED:
            return "CONNECTED";
        case WL_CONNECT_FAILED:
            return "CONNECT_FAILED";
        case WL_CONNECTION_LOST:
            return "CONNECTION_LOST";
        case WL_DISCONNECTED:
            return "DISCONNECTED";
        default:
            return "UNKNOWN";
    }
}
/**
 * @brief Convert PubSubClient connection state to a readable string.
 */
static inline const char* mqttStateName(int8_t s) {
    switch (s) {
        case MQTT_CONNECTION_TIMEOUT:
            return "CONNECTION_TIMEOUT";
        case MQTT_CONNECTION_LOST:
            return "CONNECTION_LOST";
        case MQTT_CONNECT_FAILED:
            return "CONNECT_FAILED";
        case MQTT_DISCONNECTED:
            return "DISCONNECTED";
        case MQTT_CONNECTED:
            return "CONNECTED";
        case MQTT_CONNECT_BAD_PROTOCOL:
            return "BAD_PROTOCOL";
        case MQTT_CONNECT_BAD_CLIENT_ID:
            return "BAD_CLIENT_ID";
        case MQTT_CONNECT_UNAVAILABLE:
            return "UNAVAILABLE";
        case MQTT_CONNECT_BAD_CREDENTIALS:
            return "BAD_CREDENTIALS";
        case MQTT_CONNECT_UNAUTHORIZED:
            return "UNAUTHORIZED";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Tune MQTT timeouts/buffer to play nicer with OTA pauses.
 */
static void initMqttTuning() {
    // Longer keepalive so brief OTA stalls don't drop MQTT
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(5);
    mqttClient.setBufferSize(1024);
}
/**
 * @brief Resolve `MQTT_HOST` to an IP once and cache it.
 */
static void resolveBrokerIfNeeded() {
    if (g_mqttIpResolved) return;
    if (g_mqttIp.fromString(MQTT_HOST)) {
        g_mqttIpResolved = true;
        LOG("MQTT: using literal host %s", g_mqttIp.toString().c_str());
        mqttClient.setServer(g_mqttIp, MQTT_PORT);
        return;
    }
    if (WiFi.hostByName(MQTT_HOST, g_mqttIp) == 1) {
        g_mqttIpResolved = true;
        LOG("MQTT: %s resolved to %s", MQTT_HOST, g_mqttIp.toString().c_str());
        mqttClient.setServer(g_mqttIp, MQTT_PORT);
    } else
        LOG("MQTT: DNS resolve failed for %s", MQTT_HOST);
}

// ---------- OTA ----------
// Forward declaration for MQTT publish helper used in OTA callbacks
static void publishStr(const char* topic, const String& v, bool retain = false);
/**
 * @brief Initialize ArduinoOTA once Wi‑Fi is connected.
 *
 * Notes:
 * - Hostname is derived from MAC address for stability.
 * - If `OTA_PASSWORD` or `OTA_PASSWORD_HASH` is defined (via build flags),
 *   OTA authentication is enforced.
 * - During OTA, heater and TRIAC outputs are disabled and main work throttled.
 */
static void ensureOta() {
    if (otaInitialized || WiFi.status() != WL_CONNECTED) return;

    // Derive a stable hostname from MAC
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char host[32];
    snprintf(host, sizeof(host), "gaggia-%02X%02X%02X", mac[3], mac[4], mac[5]);

    ArduinoOTA.setHostname(host);
#if defined(OTA_PASSWORD_HASH)
    ArduinoOTA.setPasswordHash(OTA_PASSWORD_HASH);
#elif defined(OTA_PASSWORD)
    ArduinoOTA.setPassword(OTA_PASSWORD);
#endif

    ArduinoOTA.onStart([]() {
        LOG("OTA: Start (%s)", ArduinoOTA.getCommand() == U_FLASH ? "flash" : "fs");
        otaActive = true;  // signal loop to reduce load and suppress MQTT publishing
        // Turn off heater to reduce power/noise during OTA
        digitalWrite(HEAT_PIN, LOW);
        digitalWrite(TRIAC_PIN, LOW);
        heaterState = false;
    });
    ArduinoOTA.onEnd([]() {
        LOG("OTA: End");
        otaActive = false;
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned int lastPct = 101;
        unsigned int pct = (total ? (progress * 100u / total) : 0u);
        if (pct != lastPct && (pct % 10u == 0u)) {
            LOG("OTA: %u%%", pct);
            lastPct = pct;
        }
    });
    ArduinoOTA.onError([](ota_error_t error) {
        const char* msg = "UNKNOWN";
        switch (error) {
            case OTA_AUTH_ERROR: msg = "AUTH"; break;
            case OTA_BEGIN_ERROR: msg = "BEGIN"; break;
            case OTA_CONNECT_ERROR: msg = "CONNECT"; break;
            case OTA_RECEIVE_ERROR: msg = "RECEIVE"; break;
            case OTA_END_ERROR: msg = "END"; break;
        }
        LOG("OTA: Error %d (%s)", (int)error, msg);
        otaActive = false;
    });

    ArduinoOTA.begin();
    otaInitialized = true;
    LOG("OTA: Ready as %s.local", host);
}

/**
 * @brief PID controller with integral windup guard and filtered derivative.
 * @param Kp Proportional gain
 * @param Ki Integral gain
 * @param Kd Derivative gain
 * @param sp Setpoint (target)
 * @param pv Process value (measured)
 * @param dt Timestep in seconds
 * @param pvFilt Storage for filtered process variable (updated)
 * @param iSum Integral accumulator (updated)
 * @param guard Absolute limit for iTerm contribution (anti-windup)
 * @param dTau Derivative low-pass filter time constant (seconds)
 * @return Control output (unclamped)
 */
// Continuous-time gains: Kp [out/°C], Ki [out/(°C·s)], Kd [out·s/°C]
static float calcPID(float Kp, float Ki, float Kd,
                     float sp, float pv,
                     float dt,                    // seconds (0.5 at 2 Hz)
                     float& pvFilt,               // store filtered pv
                     float& iSum,                 // integral accumulator (∫err dt)
                     float guard,                 // limit on iTerm (e.g., 10 = ±10% duty)
                     float dTau = 1.0f)           // derivative LPF time const (s), ~1×dt
{
    // 1) Error
    float err = sp - pv;

    // 2) Integral (scaled by dt)
    iSum += err * dt;

    // 3) Derivative on measurement with 1st-order filter (dirty derivative)
    //    LPF on pv: pvFilt' = (pv - pvFilt)/dTau
    float alpha = dt / (dTau + dt);  // 0<alpha<1
    float prevPvFilt = pvFilt;
    pvFilt += alpha * (pv - pvFilt);  // low-pass the measurement
    float dMeas = (pvFilt - prevPvFilt) / dt;

    // 4) Terms
    float pTerm = Kp * err;
    float iTerm = Ki * iSum;
    // clamp the CONTRIBUTION of I (anti-windup)
    if (iTerm >  guard) iTerm =  guard;
    if (iTerm < -guard) iTerm = -guard;

    float dTerm = -Kd * dMeas;          // derivative on measurement

    // 5) Output (unclamped here; clamp at actuator)
    return pTerm + iTerm + dTerm;
}

// --- MAX31865 deep debug read: proves the analog path ---
/**
 * @brief Verbose one‑shot read of MAX31865 for debugging analog chain.
 */
static void debugReadMAX31865() {
    // Configure for your wiring (2/3/4 wire)
    max31865.begin(MAX31865_2WIRE);
    delay(5);

    // Clear any latched faults, enable bias, take a one-shot
    max31865.clearFault();
    max31865.enableBias(true);
    delay(10);  // bias settle
    max31865.autoConvert(false);
    delay(1);

    uint16_t raw = max31865.readRTD();  // Library returns 15-bit value (D15..D1), LSB is fault
    // Some forks return the unshifted word. Normalize defensively:
    uint16_t rtd = (raw > 32768) ? (raw >> 1) : raw;

    // Ratio is 15-bit / 32768.0
    float ratio = (float)rtd / 32768.0f;
    float R = ratio * RREF;

    uint8_t f = max31865.readFault();
    max31865.enableBias(false);

    LOG("MAX31865: raw=0x%04X rtd=%u ratio=%.6f R=%.2f Ω", raw, rtd, ratio, R);

    if (f) {
        LOG("MAX31865: FAULT=0x%02X%s%s%s%s%s%s", f,
            (f & MAX31865_FAULT_HIGHTHRESH) ? " HIGHTHRESH" : "",
            (f & MAX31865_FAULT_LOWTHRESH) ? " LOWTHRESH" : "",
            (f & MAX31865_FAULT_REFINLOW) ? " REFINLOW" : "",
            (f & MAX31865_FAULT_REFINHIGH) ? " REFINHIGH" : "",
            (f & MAX31865_FAULT_RTDINLOW) ? " RTDINLOW" : "",
            (f & MAX31865_FAULT_OVUV) ? " OVC/UV" : "");
        // Keep faults latched in logs; clear if you want to retry:
        // max31865.clearFault();
    }

    // For extra confidence, also compute temperature:
    float t = max31865.temperature(RNOMINAL, RREF);
    LOG("MAX31865: Temp=%.2f °C (RNOM=%.1f, RREF=%.1f)", t, RNOMINAL, RREF);
}

// --- MAX31865 connectivity check & logging ---
/**
 * @brief Check MAX31865 fault/status and log useful diagnostics.
 */
static void logMax31865Status() {
    uint8_t f = max31865.readFault();

    // If the chip isn't on the bus, many MCUs will read 0xFF on MISO
    if (f == 0xFF) {
        LOG("MAX31865: no SPI response (CS=%d) — check wiring/power", MAX_CS);
        return;
    }

    if (f) {
        // Decode common fault bits for clarity
        LOG("MAX31865: detected but reporting FAULT=0x%02X%s%s%s%s%s%s", f,
            (f & MAX31865_FAULT_HIGHTHRESH) ? " HIGHTHRESH" : "",
            (f & MAX31865_FAULT_LOWTHRESH) ? " LOWTHRESH" : "",
            (f & MAX31865_FAULT_REFINLOW) ? " REFINLOW" : "",
            (f & MAX31865_FAULT_REFINHIGH) ? " REFINHIGH" : "",
            (f & MAX31865_FAULT_RTDINLOW) ? " RTDINLOW" : "",
            (f & MAX31865_FAULT_OVUV) ? " OVC/UV" : "");
        max31865.clearFault();
    } else {
        // Read raw ratio (0..1), calculate resistance, and temperature
        uint16_t raw = max31865.readRTD();
        uint16_t rtd = (raw > 32768) ? (raw >> 1) : raw;
        float ratio = (float)rtd / 32768.0f;
        float resistance = ratio * RREF;
        float tempC = max31865.temperature(RNOMINAL, RREF);

        LOG("MAX31865: connected; raw=0x%04X rtd=%u ratio=%.6f  R=%.2f Ω  Temp=%.2f °C", raw, rtd,
            ratio, resistance, tempC);
    }
}

// --------------- espresso logic ---------------
/**
 * @brief Detect shot start/stop based on zero‑cross events and steam transitions.
 */
static void checkShotStartStop() {
    if ((zcCount >= ZC_MIN) && !shotFlag && setupComplete &&
        ((currentTime - startTime) > ZC_WAIT)) {
        shotStart = currentTime;
        shotTime = 0;
        shotFlag = true;
        pulseCount = 0;
        preFlow = true;
        preFlowVol = 0;
    }
    if ((steamFlag && !prevSteamFlag) ||
        (currentTime - lastZcTime >= SHOT_RESET && shotFlag && currentTime > lastZcTime)) {
        pulseCount = 0;
        lastVol = 0;
        shotVol = 0;
        shotTime = 0;
        lastPulseTime = currentTime;
        shotFlag = false;
        preFlow = false;
    }
}

/**
 * @brief Read temperature and update heater PID and window length.
 */
static void updateTempPID() {
    currentTemp = max31865.temperature(RNOMINAL, RREF);
    if (currentTemp < 0) currentTemp = lastTemp;
    float dt = (currentTime - lastPidTime) / 1000.0f;
    lastPidTime = currentTime;
    if (!heaterEnabled) {
        // Pause PID calculations when heater is disabled
        heatPower = 0.0f;
        heatCycles = PWM_CYCLE;
        return;
    }

    // Active target picks between brew and steam setpoints
    setTemp = steamFlag ? steamSetpoint : brewSetpoint;

    heatPower = calcPID(pGainTemp, iGainTemp, dGainTemp,
                        setTemp, currentTemp,
                        dt, pvFiltTemp, iStateTemp, windupGuardTemp);
    if (heatPower > 100.0f) heatPower = 100.0f;
    if (heatPower < 0.0f) heatPower = 0.0f;
    heatCycles = (int)((100.0f - heatPower) / 100.0f * PWM_CYCLE);
    lastTemp = currentTemp;
}

/**
 * @brief Apply time‑proportioning control to the heater output.
 */
static void updateTempPWM() {
    if (!heaterEnabled) {
        digitalWrite(HEAT_PIN, LOW);
        heaterState = false;
        return;
    }
    if (currentTime - lastPwmTime >= (unsigned long)heatCycles) {
        digitalWrite(HEAT_PIN, HIGH);
        heaterState = true;
    }
    if (currentTime - lastPwmTime >= PWM_CYCLE) {
        digitalWrite(HEAT_PIN, LOW);
        heaterState = false;
        lastPwmTime = currentTime;
        nLoop = 0;
    }
    nLoop++;
}

/**
 * @brief Sample pressure ADC and maintain a moving average buffer.
 */
static void updatePressure() {
    rawPress = analogRead(PRESS_PIN);
    pressNow = rawPress * pressGrad + pressInt;
    uint8_t idx = pressBuffIdx;
    pressSum -= pressBuff[idx];
    pressBuff[idx] = pressNow;
    pressSum += pressNow;
    pressBuffIdx++;
    if (pressBuffIdx >= PRESS_BUFF_SIZE) pressBuffIdx = 0;
    lastPress = pressSum / PRESS_BUFF_SIZE;
}

/**
 * @brief Infer steam mode based on AC sense and recent zero‑cross activity.
 */
static void updateSteamFlag() {
    ac = !digitalRead(AC_SENS);
    prevSteamFlag = steamFlag;
    if ((millis() - lastZcTime) > ZC_OFF && ac) {
        acCount++;
        if (acCount > STEAM_MIN) {
            steamFlag = true;
        }
    } else {
        steamFlag = false;
        acCount = 0;
    }
}

/**
 * @brief Track pre‑infusion phase and capture volume up to threshold pressure.
 */
static void updatePreFlow() {
    if (preFlow && lastPress > PRESS_THRESHOLD) {
        preFlow = false;
        preFlowVol = vol;
    }
}

/**
 * @brief Convert pulse counts to volumes and maintain shot volume.
 */
static void updateVols() {
    vol = (int)(pulseCount * FLOW_CAL);
    lastVol = vol;
    shotVol = (preFlow || !shotFlag) ? 0 : (vol - preFlowVol);
}

// TRIAC dimmer control (Pump): schedule and fire gate pulses after zero-cross
/**
 * @brief Phase‑angle control for pump TRIAC synchronized to zero‑cross.
 *
 * Schedules a short gate pulse at a delay mapped from desired power.
 * Disabled during OTA to prioritize Wi‑Fi responsiveness.
 */
static void updateTriacControl() {
    if (!TRIAC_ENABLED) return;
    // Disable TRIAC during OTA to keep WiFi responsive
    if (otaActive) {
        digitalWrite(TRIAC_PIN, LOW);
        triacPulsePending = false;
        triacPulseOn = false;
        return;
    }
    // New zero-cross detected: compute delay based on desired power
    if (zcFlagTriac) {
        zcFlagTriac = false;
        float pct = pumpPowerPct;
        if (pct <= 0.0f) {
            triacPulsePending = false;
            triacPulseOn = false;
            digitalWrite(TRIAC_PIN, LOW);
        } else if (pct >= 100.0f) {
            // Fire almost immediately after ZC
            triacFireTimeUs = lastZcMicros + 50;  // small fixed delay
            triacPulsePending = true;
        } else {
            unsigned long delayUs = (unsigned long)((100.0f - pct) * 0.01f * (float)HALF_CYCLE_US);
            triacFireTimeUs = lastZcMicros + delayUs;
            triacPulsePending = true;
        }
    }
    unsigned long nowUs = micros();
    if (triacPulsePending && ((long)(nowUs - triacFireTimeUs) >= 0)) {
        digitalWrite(TRIAC_PIN, HIGH);
        triacPulseOn = true;
        triacPulseEndUs = nowUs + TRIAC_PULSE_US;
        triacPulsePending = false;
    }
    if (triacPulseOn && ((long)(micros() - triacPulseEndUs) >= 0)) {
        digitalWrite(TRIAC_PIN, LOW);
        triacPulseOn = false;
    }
}

// ISRs
/**
 * @brief Flow sensor ISR with simple debounce using `PULSE_MIN`.
 */
static void IRAM_ATTR flowInt() {
    unsigned long now = millis();
    if (now - lastPulseTime >= PULSE_MIN) {
        pulseCount++;
        lastPulseTime = now;
    }
}
/**
 * @brief Zero‑cross detect ISR: records timing and triggers TRIAC scheduling.
 */
static void IRAM_ATTR zcInt() {
    lastZcTime = millis();
    zcCount++;
    // TRIAC phase control timing (use micros for precision)
    lastZcMicros = micros();
    zcFlagTriac = true;
}

// ---------- HA Discovery helpers ----------
/**
 * @brief Build stable device identifiers from the MAC address.
 */
static void makeIdsFromMac() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    snprintf(uid_suffix, sizeof(uid_suffix), "%02X%02X%02X", mac[3], mac[4], mac[5]);
    snprintf(dev_id, sizeof(dev_id), "gaggia_classic-%s", uid_suffix);
}

/**
 * @brief Construct MQTT topics for state, commands and discovery.
 */
static void buildTopics() {
    // sensor states
    snprintf(t_shotvol_state, sizeof(t_shotvol_state), "%s/%s/shot_volume/state", STATE_BASE,
             uid_suffix);
    snprintf(t_settemp_state, sizeof(t_settemp_state), "%s/%s/set_temp/state", STATE_BASE,
             uid_suffix);
    snprintf(t_curtemp_state, sizeof(t_curtemp_state), "%s/%s/current_temp/state", STATE_BASE,
             uid_suffix);
    snprintf(t_press_state, sizeof(t_press_state), "%s/%s/pressure/state", STATE_BASE, uid_suffix);
    snprintf(t_shottime_state, sizeof(t_shottime_state), "%s/%s/shot_time/state", STATE_BASE,
             uid_suffix);
    snprintf(t_ota_state, sizeof(t_ota_state), "%s/%s/ota/status", STATE_BASE, uid_suffix);
    // TRIAC power number topics (pump power)
    snprintf(t_pumppower_state, sizeof(t_pumppower_state), "%s/%s/pump_power/state", STATE_BASE,
             uid_suffix);
    snprintf(t_pumppower_cmd, sizeof(t_pumppower_cmd), "%s/%s/pump_power/set", STATE_BASE,
             uid_suffix);
    // heater switch topics
    snprintf(t_heater_state, sizeof(t_heater_state), "%s/%s/heater/state", STATE_BASE, uid_suffix);
    snprintf(t_heater_cmd, sizeof(t_heater_cmd), "%s/%s/heater/set", STATE_BASE, uid_suffix);
    // diagnostic counters states
    snprintf(t_accnt_state, sizeof(t_accnt_state), "%s/%s/ac_count/state", STATE_BASE,
             uid_suffix);
    snprintf(t_zccnt_state, sizeof(t_zccnt_state), "%s/%s/zc_count/state", STATE_BASE,
             uid_suffix);
    snprintf(t_pulsecnt_state, sizeof(t_pulsecnt_state), "%s/%s/pulse_count/state", STATE_BASE,
             uid_suffix);
    // binary sensor states
    snprintf(t_shot_state, sizeof(t_shot_state), "%s/%s/shot/state", STATE_BASE, uid_suffix);
    snprintf(t_preflow_state, sizeof(t_preflow_state), "%s/%s/preflow/state", STATE_BASE,
             uid_suffix);
    snprintf(t_steam_state, sizeof(t_steam_state), "%s/%s/steam/state", STATE_BASE, uid_suffix);
    // number command/state
    snprintf(t_brewset_cmd, sizeof(t_brewset_cmd), "%s/%s/brew_setpoint/set", STATE_BASE,
             uid_suffix);
    snprintf(t_brewset_state, sizeof(t_brewset_state), "%s/%s/brew_setpoint/state", STATE_BASE,
             uid_suffix);
    snprintf(t_steamset_cmd, sizeof(t_steamset_cmd), "%s/%s/steam_setpoint/set", STATE_BASE,
             uid_suffix);
    snprintf(t_steamset_state, sizeof(t_steamset_state), "%s/%s/steam_setpoint/state", STATE_BASE,
             uid_suffix);
    // PID numbers
    snprintf(t_pidp_cmd, sizeof(t_pidp_cmd), "%s/%s/pid_p/set", STATE_BASE, uid_suffix);
    snprintf(t_pidp_state, sizeof(t_pidp_state), "%s/%s/pid_p/state", STATE_BASE, uid_suffix);
    snprintf(t_pidi_cmd, sizeof(t_pidi_cmd), "%s/%s/pid_i/set", STATE_BASE, uid_suffix);
    snprintf(t_pidi_state, sizeof(t_pidi_state), "%s/%s/pid_i/state", STATE_BASE, uid_suffix);
    snprintf(t_pidd_cmd, sizeof(t_pidd_cmd), "%s/%s/pid_d/set", STATE_BASE, uid_suffix);
    snprintf(t_pidd_state, sizeof(t_pidd_state), "%s/%s/pid_d/state", STATE_BASE, uid_suffix);
    snprintf(t_pidg_cmd, sizeof(t_pidg_cmd), "%s/%s/pid_guard/set", STATE_BASE, uid_suffix);
    snprintf(t_pidg_state, sizeof(t_pidg_state), "%s/%s/pid_guard/state", STATE_BASE, uid_suffix);
    // sensor mirrors of setpoints
    snprintf(t_brewset_sensor_state, sizeof(t_brewset_sensor_state), "%s/%s/brew_setpoint/state",
             STATE_BASE, uid_suffix);
    snprintf(t_steamset_sensor_state, sizeof(t_steamset_sensor_state), "%s/%s/steam_setpoint/state",
             STATE_BASE, uid_suffix);

    // configs
    snprintf(c_shotvol, sizeof(c_shotvol), "%s/sensor/%s_shot_volume/config", DISCOVERY_PREFIX,
             dev_id);
    snprintf(c_settemp, sizeof(c_settemp), "%s/sensor/%s_set_temp/config", DISCOVERY_PREFIX,
             dev_id);
    snprintf(c_curtemp, sizeof(c_curtemp), "%s/sensor/%s_current_temp/config", DISCOVERY_PREFIX,
             dev_id);
    snprintf(c_press, sizeof(c_press), "%s/sensor/%s_pressure/config", DISCOVERY_PREFIX, dev_id);
    snprintf(c_shottime, sizeof(c_shottime), "%s/sensor/%s_shot_time/config", DISCOVERY_PREFIX,
             dev_id);
    snprintf(c_ota, sizeof(c_ota), "%s/sensor/%s_ota_status/config", DISCOVERY_PREFIX, dev_id);

    snprintf(c_shot, sizeof(c_shot), "%s/binary_sensor/%s_shot/config", DISCOVERY_PREFIX, dev_id);
    snprintf(c_preflow, sizeof(c_preflow), "%s/binary_sensor/%s_preflow/config", DISCOVERY_PREFIX,
             dev_id);
    snprintf(c_steam, sizeof(c_steam), "%s/binary_sensor/%s_steam/config", DISCOVERY_PREFIX,
             dev_id);

    // numbers
    snprintf(c_brewset_number, sizeof(c_brewset_number), "%s/number/%s_brew_setpoint/config",
             DISCOVERY_PREFIX, dev_id);
    snprintf(c_steamset_number, sizeof(c_steamset_number), "%s/number/%s_steam_setpoint/config",
             DISCOVERY_PREFIX, dev_id);
    // PID numbers
    snprintf(c_pidp_number, sizeof(c_pidp_number), "%s/number/%s_pid_p/config", DISCOVERY_PREFIX, dev_id);
    snprintf(c_pidi_number, sizeof(c_pidi_number), "%s/number/%s_pid_i/config", DISCOVERY_PREFIX, dev_id);
    snprintf(c_pidd_number, sizeof(c_pidd_number), "%s/number/%s_pid_d/config", DISCOVERY_PREFIX, dev_id);
    snprintf(c_pidg_number, sizeof(c_pidg_number), "%s/number/%s_pid_guard/config", DISCOVERY_PREFIX, dev_id);

    // sensor mirrors for setpoints
    snprintf(c_brewset_sensor, sizeof(c_brewset_sensor), "%s/sensor/%s_brew_setpoint/config",
             DISCOVERY_PREFIX, dev_id);
    snprintf(c_steamset_sensor, sizeof(c_steamset_sensor), "%s/sensor/%s_steam_setpoint/config",
             DISCOVERY_PREFIX, dev_id);
    // diagnostic sensors
    snprintf(c_accnt, sizeof(c_accnt), "%s/sensor/%s_ac_count/config", DISCOVERY_PREFIX, dev_id);
    snprintf(c_zccnt, sizeof(c_zccnt), "%s/sensor/%s_zc_count/config", DISCOVERY_PREFIX, dev_id);
    snprintf(c_pulsecnt, sizeof(c_pulsecnt), "%s/sensor/%s_pulse_count/config", DISCOVERY_PREFIX, dev_id);
    // switch
    snprintf(c_heater, sizeof(c_heater), "%s/switch/%s_heater/config", DISCOVERY_PREFIX, dev_id);
    // TRIAC power number (pump power)
    snprintf(c_pumppower_number, sizeof(c_pumppower_number), "%s/number/%s_pump_power/config",
             DISCOVERY_PREFIX, dev_id);
}

/**
 * @brief JSON snippet describing the device for HA discovery payloads.
 */
static String deviceJson() {
    String j = "{";
    j += "\"identifiers\":[\"" + String(dev_id) + "\"],";
    j += "\"name\":\"Gaggia "
         "Classic\",\"manufacturer\":\"Custom\",\"model\":\"Gagguino\",\"sw_version\":\"" VERSION
         "\"}";
    return j;
}

/**
 * @brief Publish a retained discovery/config message.
 */
static void publishRetained(const char* topic, const String& payload) {
    if (!mqttClient.publish(topic, payload.c_str(), true))
        LOG("MQTT: discovery publish failed (%s)", topic);
}

/**
 * @brief Publish Home Assistant discovery entities (sensors, numbers, switch).
 */
static void publishDiscovery() {
    String dev = deviceJson();

    // --- sensors ---
    publishRetained(c_shotvol,
                    String("{\"name\":\"Shot Volume\",\"uniq_id\":\"") + dev_id +
                        "_shot_volume\",\"stat_t\":\"" + t_shotvol_state +
                        "\",\"dev_cla\":\"volume\",\"unit_of_meas\":\"mL\",\"stat_cla\":"
                        "\"measurement\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev +
                        "}");
    publishRetained(c_settemp,
                    String("{\"name\":\"Set Temperature\",\"uniq_id\":\"") + dev_id +
                        "_set_temp\",\"stat_t\":\"" + t_settemp_state +
                        "\",\"dev_cla\":\"temperature\",\"unit_of_meas\":\"°C\",\"stat_cla\":"
                        "\"measurement\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev +
                        "}");
    publishRetained(c_curtemp,
                    String("{\"name\":\"Current Temperature\",\"uniq_id\":\"") + dev_id +
                        "_current_temp\",\"stat_t\":\"" + t_curtemp_state +
                        "\",\"dev_cla\":\"temperature\",\"unit_of_meas\":\"°C\",\"stat_cla\":"
                        "\"measurement\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev +
                        "}");
    publishRetained(c_press,
                    String("{\"name\":\"Pressure\",\"uniq_id\":\"") + dev_id +
                        "_pressure\",\"stat_t\":\"" + t_press_state +
                        "\",\"dev_cla\":\"pressure\",\"unit_of_meas\":\"bar\",\"stat_cla\":"
                        "\"measurement\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev +
                        "}");
    publishRetained(c_shottime,
                    String("{\"name\":\"Shot Time\",\"uniq_id\":\"") + dev_id +
                        "_shot_time\",\"stat_t\":\"" + t_shottime_state +
                        "\",\"dev_cla\":\"duration\",\"unit_of_meas\":\"s\",\"stat_cla\":"
                        "\"measurement\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev +
                        "}");

    // OTA status (diagnostic sensor)
    publishRetained(c_ota,
                    String("{\"name\":\"OTA Status\",\"uniq_id\":\"") + dev_id +
                        "_ota_status\",\"stat_t\":\"" + t_ota_state +
                        "\",\"entity_category\":\"diagnostic\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev +
                        "}");

    // Diagnostic counters
    publishRetained(c_accnt,
                    String("{\"name\":\"AC Count\",\"uniq_id\":\"") + dev_id +
                        "_ac_count\",\"stat_t\":\"" + t_accnt_state +
                        "\",\"stat_cla\":\"measurement\",\"entity_category\":\"diagnostic\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");
    publishRetained(c_zccnt,
                    String("{\"name\":\"ZC Count\",\"uniq_id\":\"") + dev_id +
                        "_zc_count\",\"stat_t\":\"" + t_zccnt_state +
                        "\",\"stat_cla\":\"measurement\",\"entity_category\":\"diagnostic\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");
    publishRetained(c_pulsecnt,
                    String("{\"name\":\"Pulse Count\",\"uniq_id\":\"") + dev_id +
                        "_pulse_count\",\"stat_t\":\"" + t_pulsecnt_state +
                        "\",\"stat_cla\":\"measurement\",\"entity_category\":\"diagnostic\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");

    // --- binary sensors ---
    publishRetained(
        c_shot, String("{\"name\":\"Shot\",\"uniq_id\":\"") + dev_id + "_shot\",\"stat_t\":\"" +
                    t_shot_state +
                    "\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev_cla\":\"running\",\"avty_t\":\"" +
                    String(MQTT_STATUS) +
                    "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");
    publishRetained(
        c_preflow,
        String("{\"name\":\"Pre-Flow\",\"uniq_id\":\"") + dev_id + "_preflow\",\"stat_t\":\"" +
            t_preflow_state +
            "\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev_cla\":\"running\",\"avty_t\":\"" +
            String(MQTT_STATUS) +
            "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");
    publishRetained(
        c_steam,
        String("{\"name\":\"Steam\",\"uniq_id\":\"") + dev_id + "_steam\",\"stat_t\":\"" +
            t_steam_state +
            "\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev_cla\":\"running\",\"avty_t\":\"" +
            String(MQTT_STATUS) +
            "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");

    // --- switch: heater enable ---
    publishRetained(
        c_heater,
        String("{\"name\":\"Heater\",\"uniq_id\":\"") + dev_id +
            "_heater\",\"cmd_t\":\"" + t_heater_cmd + "\",\"stat_t\":\"" + t_heater_state +
            "\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"avty_t\":\"" + String(MQTT_STATUS) +
            "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");

    // --- number: pump power (TRIAC) ---
        publishRetained(
            c_pumppower_number,
            String("{\"name\":\"Pump Power\",\"uniq_id\":\"") + dev_id +
            "_pump_power\",\"cmd_t\":\"" + t_pumppower_cmd + "\",\"stat_t\":\"" +
            t_pumppower_state +
            "\",\"unit_of_meas\":\"%\",\"min\":0,\"max\":100,\"step\":1,\"mode\":\"auto\",\"avty_t\":\"" +
            String(MQTT_STATUS) +
            "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"entity_category\":\"config\",\"dev\":" +
            dev + "}");

    // --- numbers (controllable) ---
    publishRetained(
        c_brewset_number,
        String("{\"name\":\"Brew Setpoint\",\"uniq_id\":\"") + dev_id +
            "_brew_setpoint\",\"cmd_t\":\"" + t_brewset_cmd + "\",\"stat_t\":\"" + t_brewset_state +
            "\",\"unit_of_meas\":\"°C\",\"dev_cla\":\"temperature\",\"min\":90,\"max\":99,\"step\":"
            "1,\"mode\":\"auto\",\"avty_t\":\"" +
            String(MQTT_STATUS) +
            "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");

    publishRetained(c_steamset_number,
                    String("{\"name\":\"Steam Setpoint\",\"uniq_id\":\"") + dev_id +
                        "_steam_setpoint\",\"cmd_t\":\"" + t_steamset_cmd + "\",\"stat_t\":\"" +
                        t_steamset_state +
                        "\",\"unit_of_meas\":\"°C\",\"dev_cla\":\"temperature\",\"min\":145,"
                        "\"max\":155,\"step\":1,\"mode\":\"auto\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev +
                        "}");

    // PID number entities
    publishRetained(
        c_pidp_number,
        String("{\"name\":\"PID P\",\"uniq_id\":\"") + dev_id +
            "_pid_p\",\"cmd_t\":\"" + t_pidp_cmd + "\",\"stat_t\":\"" + t_pidp_state +
            "\",\"min\":0,\"max\":200,\"step\":0.1,\"mode\":\"auto\",\"avty_t\":\"" +
            String(MQTT_STATUS) +
            "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");
    publishRetained(
        c_pidi_number,
        String("{\"name\":\"PID I\",\"uniq_id\":\"") + dev_id +
            "_pid_i\",\"cmd_t\":\"" + t_pidi_cmd + "\",\"stat_t\":\"" + t_pidi_state +
            "\",\"min\":0,\"max\":10,\"step\":0.05,\"mode\":\"auto\",\"avty_t\":\"" +
            String(MQTT_STATUS) +
            "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");
    publishRetained(
        c_pidd_number,
        String("{\"name\":\"PID D\",\"uniq_id\":\"") + dev_id +
            "_pid_d\",\"cmd_t\":\"" + t_pidd_cmd + "\",\"stat_t\":\"" + t_pidd_state +
            "\",\"min\":0,\"max\":500,\"step\":0.5,\"mode\":\"auto\",\"avty_t\":\"" +
            String(MQTT_STATUS) +
            "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");
    publishRetained(
        c_pidg_number,
        String("{\"name\":\"PID Guard\",\"uniq_id\":\"") + dev_id +
            "_pid_guard\",\"cmd_t\":\"" + t_pidg_cmd + "\",\"stat_t\":\"" + t_pidg_state +
            "\",\"min\":0,\"max\":100,\"step\":0.5,\"mode\":\"auto\",\"avty_t\":\"" +
            String(MQTT_STATUS) +
            "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev + "}");

    // --- sensor mirrors (read-only) for verification ---
    publishRetained(c_brewset_sensor,
                    String("{\"name\":\"Brew Setpoint (Sensor)\",\"uniq_id\":\"") + dev_id +
                        "_brew_setpoint_sensor\",\"stat_t\":\"" + t_brewset_sensor_state +
                        "\",\"dev_cla\":\"temperature\",\"unit_of_meas\":\"°C\",\"stat_cla\":"
                        "\"measurement\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev +
                        "}");

    publishRetained(c_steamset_sensor,
                    String("{\"name\":\"Steam Setpoint (Sensor)\",\"uniq_id\":\"") + dev_id +
                        "_steam_setpoint_sensor\",\"stat_t\":\"" + t_steamset_sensor_state +
                        "\",\"dev_cla\":\"temperature\",\"unit_of_meas\":\"°C\",\"stat_cla\":"
                        "\"measurement\",\"avty_t\":\"" +
                        String(MQTT_STATUS) +
                        "\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\",\"dev\":" + dev +
                        "}");
}

// ---------- publishing states ----------
/**
 * @brief Publish a string value to MQTT.
 */
static void publishStr(const char* topic, const String& v, bool retain) {
    if (!mqttClient.publish(topic, v.c_str(), retain))
        LOG("MQTT: state publish failed (%s) val=%s", topic, v.c_str());
}
/**
 * @brief Publish a floating‑point value with fixed decimals.
 */
static void publishNum(const char* topic, float v, uint8_t decimals = 1, bool retain = false) {
    char tmp[24];
    dtostrf(v, 0, decimals, tmp);
    publishStr(topic, String(tmp), retain);
}
static void publishBool(const char* topic, bool on, bool retain = false) {
    publishStr(topic, on ? "ON" : "OFF", retain);
}

/**
 * @brief Publish periodic telemetry and mirrored setpoint values.
 */
static void publishStates() {
    // measurements
    publishNum(t_shotvol_state, shotVol, 0);  // mL
    publishNum(t_settemp_state, setTemp, 1);  // active target
    publishNum(t_curtemp_state, currentTemp, 1);
    publishNum(t_press_state, lastPress, 1);
    publishNum(t_shottime_state, shotTime, 1);  // seconds
    // heater switch state (retained so HA keeps last state)
    publishBool(t_heater_state, heaterEnabled, true);
    // Pump power state (TRIAC, retained)
    publishNum(t_pumppower_state, pumpPowerPct, 0, true);
    // diagnostics
    publishNum(t_accnt_state, acCount, 0);
    publishNum(t_zccnt_state, zcCount, 0);
    publishNum(t_pulsecnt_state, pulseCount, 0);

    // flags
    publishBool(t_shot_state, shotFlag);
    publishBool(t_preflow_state, preFlow);
    publishBool(t_steam_state, steamFlag);

    // number entity states (retained so HA persists)
    publishNum(t_brewset_state, brewSetpoint, 1, true);
    publishNum(t_steamset_state, steamSetpoint, 1, true);
    // PID number states
    publishNum(t_pidp_state, pGainTemp, 2, true);
    publishNum(t_pidi_state, iGainTemp, 2, true);
    publishNum(t_pidd_state, dGainTemp, 2, true);
    publishNum(t_pidg_state, windupGuardTemp, 2, true);

    // sensor mirrors (verification)
    publishNum(t_brewset_sensor_state, brewSetpoint, 1);
    publishNum(t_steamset_sensor_state, steamSetpoint, 1);
}

// ---------- MQTT callback (handle both setpoints) ----------
/**
 * @brief Handle inbound MQTT commands (setpoints, PID gains, pump power, switch).
 */
static void mqttCallback(char* topic, uint8_t* payload, unsigned int len) {
    auto parse_clamped = [&](const char* t, float lo, float hi, float& outVal) -> bool {
        if (strcmp(topic, t) != 0) return false;
        char buf[32] = {0};
        unsigned n = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
        memcpy(buf, payload, n);
        float v = atof(buf);
        if (v < lo) v = lo;
        if (v > hi) v = hi;
        outVal = v;
        return true;
    };
    auto parse_onoff = [&](const char* t, bool& outVal) -> bool {
        if (strcmp(topic, t) != 0) return false;
        char buf[8] = {0};
        unsigned n = (len < sizeof(buf) - 1) ? len : sizeof(buf) - 1;
        memcpy(buf, payload, n);
        for (unsigned i = 0; i < n; ++i) buf[i] = (char)toupper((unsigned char)buf[i]);
        if (strcmp(buf, "ON") == 0) {
            outVal = true;
            return true;
        }
        if (strcmp(buf, "OFF") == 0) {
            outVal = false;
            return true;
        }
        return false;
    };
    bool changed = false;
    if (parse_clamped(t_brewset_cmd, BREW_MIN, BREW_MAX, brewSetpoint)) {
        changed = true;
        LOG("HA: Brew setpoint -> %.1f °C", brewSetpoint);
    }
    if (parse_clamped(t_steamset_cmd, STEAM_MIN_C, STEAM_MAX_C, steamSetpoint)) {
        changed = true;
        LOG("HA: Steam setpoint -> %.1f °C", steamSetpoint);
    }
    // PID params
    float tmp;
    if (parse_clamped(t_pidp_cmd, 0.0f, 200.0f, tmp)) {
        pGainTemp = tmp;
        publishNum(t_pidp_state, pGainTemp, 2, true);
        LOG("HA: PID P -> %.2f", pGainTemp);
    }
    if (parse_clamped(t_pidi_cmd, 0.0f, 10.0f, tmp)) {
        iGainTemp = tmp;
        publishNum(t_pidi_state, iGainTemp, 2, true);
        LOG("HA: PID I -> %.2f", iGainTemp);
    }
    if (parse_clamped(t_pidd_cmd, 0.0f, 500.0f, tmp)) {
        dGainTemp = tmp;
        publishNum(t_pidd_state, dGainTemp, 2, true);
        LOG("HA: PID D -> %.2f", dGainTemp);
    }
    if (parse_clamped(t_pidg_cmd, 0.0f, 100.0f, tmp)) {
        windupGuardTemp = tmp;
        publishNum(t_pidg_state, windupGuardTemp, 2, true);
        LOG("HA: PID Guard -> %.2f", windupGuardTemp);
    }
    // TRIAC pump power (0..100)
    if (parse_clamped(t_pumppower_cmd, 0.0f, 100.0f, tmp)) {
        pumpPowerPct = tmp;
        publishNum(t_pumppower_state, pumpPowerPct, 0, true);
        LOG("HA: Pump Power -> %.0f%%", pumpPowerPct);
    }
    // heater switch
    bool hv;
    if (parse_onoff(t_heater_cmd, hv)) {
        heaterEnabled = hv;
        if (!heaterEnabled) {
            heatPower = 0.0f;
            heatCycles = PWM_CYCLE;
            digitalWrite(HEAT_PIN, LOW);
            heaterState = false;
        }
        publishBool(t_heater_state, heaterEnabled, true);
        LOG("HA: Heater -> %s", heaterEnabled ? "ON" : "OFF");
    }
    if (changed) {
        if (!steamFlag) setTemp = brewSetpoint;  // if brewing, apply immediately
        // reflect new values
        publishNum(t_brewset_state, brewSetpoint, 1, true);
        publishNum(t_steamset_state, steamSetpoint, 1, true);
        // sensors (verification)
        publishNum(t_brewset_sensor_state, brewSetpoint, 1);
        publishNum(t_steamset_sensor_state, steamSetpoint, 1);
    }
}

// ---------- WiFi / MQTT ----------
/**
 * @brief Maintain Wi‑Fi connection with periodic reconnect attempts.
 */
static void ensureWifi() {
    wl_status_t now = WiFi.status();
    static wl_status_t last = (wl_status_t)255;
    if (now != last) {
        if (now == WL_CONNECTED) {
            LOG("WiFi: %s  IP=%s  GW=%s  RSSI=%d dBm", wifiStatusName(now),
                WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str(),
                WiFi.RSSI());
            g_mqttIpResolved = false;
            resolveBrokerIfNeeded();
        } else {
            LOG("WiFi: %s (code=%d) — reconnecting…", wifiStatusName(now), (int)now);
        }
        last = now;
    }
    if (now == WL_CONNECTED) return;
    static unsigned long t0 = 0;
    if (millis() - t0 < 2000) return;
    t0 = millis();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

/**
 * @brief Maintain MQTT session, (re)publish discovery and subscribe to commands.
 */
static void ensureMqtt() {
    mqttClient.loop();
    if (otaActive) {
        // Avoid connect/discovery/publishing churn during OTA
        return;
    }
    static bool lastConn = false;
    if (WiFi.status() != WL_CONNECTED) {
        lastConn = false;
        return;
    }
    if (mqttClient.connected()) {
        if (!lastConn) {
            LOG("MQTT: connected (state=%d %s)", mqttClient.state(),
                mqttStateName(mqttClient.state()));
            mqttClient.publish(MQTT_STATUS, "online", true);
            makeIdsFromMac();
            buildTopics();
            publishDiscovery();
            mqttClient.subscribe(t_brewset_cmd);
            mqttClient.subscribe(t_steamset_cmd);
            // PID control subscriptions
            mqttClient.subscribe(t_pidp_cmd);
            mqttClient.subscribe(t_pidi_cmd);
            mqttClient.subscribe(t_pidd_cmd);
            mqttClient.subscribe(t_pidg_cmd);
            // heater switch
            mqttClient.subscribe(t_heater_cmd);
            // TRIAC pump power control
            mqttClient.subscribe(t_pumppower_cmd);
        }
        lastConn = true;
        return;
    }
    if (lastConn) {
        LOG("MQTT: disconnected (state=%d %s) — will retry", mqttClient.state(),
            mqttStateName(mqttClient.state()));
        lastConn = false;
    }
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt < 2000) return;
    lastAttempt = millis();
    LOG("MQTT: connecting to %s:%u as '%s' (user=%s)…", MQTT_HOST, MQTT_PORT, MQTT_CLIENTID,
        (MQTT_USER && MQTT_USER[0]) ? MQTT_USER : "(none)");
    bool ok;
    if (MQTT_USER && MQTT_USER[0])
        ok = mqttClient.connect(MQTT_CLIENTID, MQTT_USER, MQTT_PASS, MQTT_STATUS, 0, true,
                                "offline");
    else
        ok = mqttClient.connect(MQTT_CLIENTID, nullptr, nullptr, MQTT_STATUS, 0, true, "offline");
    if (!ok)
        LOG("MQTT: connect failed rc=%d (%s)  WiFi=%s RSSI=%d IP=%s GW=%s", mqttClient.state(),
            mqttStateName(mqttClient.state()), wifiStatusName(WiFi.status()), WiFi.RSSI(),
            WiFi.localIP().toString().c_str(), WiFi.gatewayIP().toString().c_str());
}
}  // namespace

namespace gag {

/**
 * @copydoc gag::setup()
 */
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(300);
    LOG("Booting… FW %s", VERSION);

#if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
#endif

    pinMode(MAX_CS, OUTPUT);
    pinMode(HEAT_PIN, OUTPUT);
    pinMode(TRIAC_PIN, OUTPUT);
    pinMode(PRESS_PIN, INPUT);
    pinMode(FLOW_PIN, INPUT_PULLUP);
    pinMode(ZC_PIN, INPUT);
    pinMode(AC_SENS, INPUT_PULLUP);
    digitalWrite(HEAT_PIN, LOW);
    digitalWrite(TRIAC_PIN, LOW);
    heaterState = false;

    debugReadMAX31865();
    // 🔎 Confirm MAX31865 is present and healthy
    logMax31865Status();

    // zero pressure
    float startP = analogRead(PRESS_PIN) * pressGrad + pressInt;
    if (startP > -PRESS_TOL && startP < PRESS_TOL) {
        pressInt -= startP;
        LOG("Pressure Intercept reset to %f", pressInt);
    }

    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowInt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ZC_PIN), zcInt, RISING);

    pulseCount = 0;
    startTime = millis();
    lastPidTime = startTime;
    lastPwmTime = startTime;
    lastPulseTime = startTime;
    setupComplete = true;

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#if defined(ARDUINO_ARCH_ESP32)
    // Improve OTA/MQTT reliability by disabling WiFi modem sleep
    WiFi.setSleep(false);
#endif
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    initMqttTuning();
    resolveBrokerIfNeeded();

    LOG("Pins: FLOW=%d ZC=%d HEAT=%d AC_SENS=%d PRESS=%d  SPI{CS=%d}",
        FLOW_PIN, ZC_PIN, HEAT_PIN, AC_SENS, PRESS_PIN, MAX_CS);
    LOG("WiFi: connecting to '%s'…  MQTT: %s:%u id=%s", WIFI_SSID, MQTT_HOST, MQTT_PORT,
        MQTT_CLIENTID);
}

/**
 * @copydoc gag::loop()
 */
void loop() {
    currentTime = millis();

    // During OTA, minimize work to keep WiFi/TCP responsive
    if (!otaActive) {
    checkShotStartStop();
    if (currentTime - lastPidTime >= PID_CYCLE) updateTempPID();
    updateTempPWM();
    updatePressure();
    updatePreFlow();
    updateVols();
    updateSteamFlag();
    }

    // Update shot time continuously while shot is active (seconds)
    if (shotFlag) {
        shotTime = (currentTime - shotStart) / 1000.0f;
    }

    ensureWifi();
    ensureMqtt();
    ensureOta();
    if (WiFi.status() == WL_CONNECTED) ArduinoOTA.handle();
    // TRIAC control needs frequent checks (after OTA.handle to keep WiFi responsive)
    if (!otaActive) updateTriacControl();

    unsigned long publishInterval = shotFlag ? MQTT_SHOT_CYCLE : MQTT_IDLE_CYCLE;
    if (!otaActive && streamData && (currentTime - lastMqttTime) >= publishInterval) {
        if (mqttClient.connected()) publishStates();
        lastMqttTime = currentTime;
    }

    if (!otaActive && debugPrint && (currentTime - lastLogTime) > LOG_CYCLE) { /* optional debug printing */
        LOG("Pressure: Raw=%d, Now=%0.2f Last=%0.2f", rawPress, pressNow, lastPress);
        LOG("Temp: Set=%0.1f, Current=%0.2f", setTemp, currentTemp);        
        LOG("Heat: Power=%0.1f, Cycles=%d",heatPower, heatCycles);     
        LOG("Vol: Pulses=%lu, Vol=%d", pulseCount, vol);
        LOG("Pump: ZC Count =%lu", zcCount);
        LOG("Flags: Steam=%d, Shot=%d", steamFlag, shotFlag);
        LOG("AC Count=%d", acCount);
        LOG("");
        lastLogTime = currentTime;
    }
}

}  // namespace gag
