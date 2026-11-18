// Gloria.ino
//
// Firmware for "Gloria" - a solar pump controller.
// Hardware = 1x Valentino base PCB + 1x Gloria shield PCB.
//
// This is Langley.ino (Projects/Langley, v19) with the Langley shield code (fence-pulse sense,
// energizer battery monitor, two TM1637 displays) taken out and the Gloria shield put in. The
// Valentino base code - power management, sleep, LoRa, WiFi, vital signs, device commands, relay
// mode - is Langley's, unchanged. Pump control logic is not written yet: the pump relay stays off.
//
//   Valentino Build 3 (base board): ESP32-WROOM-32E, PCF8563 RTC, RA-02 (SX1278)
//   LoRa module, MCP9808 board-temperature sensor (I2C 0x18), TPL5010 hardware
//   watchdog, dual INA219 current/power monitors (battery + charge input),
//   ADS1115 (0x48: thermistor, RTC coin cell, charger STAT1/STAT2) and a
//   BQ24650 solar charger.
//
//   Gloria shield (HardwareDesign/Gloria, bench-tested with TestGloria 2026-11-18): 1.54" SSD1309
//   OLED (0x3C), 9 WS2812B LEDs, pump relay (Q3) in series with the emergency switch SW1, START
//   button S2, AUTO/MANUAL switch SW2, ADS1115 U1 (0x4A, config DIP + emergency sense) and U2
//   (0x49, four sensor inputs), four level-shifted digital sensor inputs, TPS2051 switch for the
//   OLED/LED rail.
//
// The I2C bus runs at 100 kHz, not Langley's 400 kHz: with U1 fitted (ADDR tied to SDA) the bus
// hung within seconds at 400 kHz; at 100 kHz it ran clean (TestGloria, 2026-11-18).

#include "Arduino.h"
#include <DeviceSerial.h>  // the serial number is the efuse MAC (2026-10-30)
#include <esp_mac.h>
#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_INA219.h>
#include "ADS1X15.h"
#include <FastLED.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <PCF8563TimeManager.h>
#include <Esp32SecretManager.h>
#include <GeneralFunctions.h>
#include <ErrorManager.h>
#include <ErrorDefinitions.h>
#include <Timer.h>
#include <LangleyData.h>
#include <LittleFS.h>
#include <LangleyWifiManager.h>
#include <SolarInfo.h>
#include <SolarPowerData.h>
#include <BatteryPowerManager.h>
#include <CommaRecord.h>  // OPERATING_STATUS_* constants, shared across all solar Digital Stables devices
#include <VitalSignsTracker.h>
#include <DeviceCommandRecord.h>
#include <LinkAckRecord.h>
#include <Preferences.h>
#include <LoRaRelay.h>  // relay mode (2026-11-11) - see the relay section
#include <DigitalStablesData.h>  // heard from Daffodils (e.g. Pool), for the pump rule
#include <ChinampaData.h>
#include "GloriaPumpRule.h"

// ============================================================
// Pin definitions - Valentino base board
// ============================================================

// LoRa (RA-02 / SX1278) SPI bus - U1 IO12/13/14/15, RESET on IO4, DIO0 on IO35
#define MISO 12
#define MOSI 13
#define SCK 14
#define LoRa_SS 15
#define LORA_RESET 4
#define LORA_DIO0 35

// I2C bus - shared by the RTC, both onboard INA219s and everything on the shield
#define SDA_PIN 21
#define SCL_PIN 22

// PCF8563 RTC 1Hz tick output (input-only pin, used as a once-a-second interrupt)
#define RTC_CLK_OUT 34


// TPL5010 hardware watchdog
#define WATCHDOG_WAKE 25  // input: watchdog asks us to wake/pet it
#define WATCHDOG_DONE 26  // output: pulse high then low to pet the watchdog

// Input-only ADC1 pins used as analog voltage senses
// RTC backup coin cell (CR1220) through the R41(1M)/R46(2M) divider. Build 3 reads it on the
// Valentino ADS1115 AIN1; GPIO36 is no longer connected.
#define RTC_BATT_DIVIDER_R1 1000000.0f  // R41, ohms
#define RTC_BATT_DIVIDER_R2 2000000.0f  // R46, ohms
#define CHEMISTRY_PIN 39   // SENSOR_VN - JP3: R16 10k pull-up to 3V3, jumper to GND; also drives the
                           // BQ24650 TERM_EN pin, see detectBatteryChemistry()

// ============================================================
// Pin definitions - Gloria shield
// (wired through Valentino's J3/J4 expansion headers)
// ============================================================
#define LED_DATA_PIN 16        // WS2812B chain data through R9 330
#define PUMP_RELAY_PIN 17      // Q3 gate; HIGH = relay on = pump allowed to run (NC opens). Only closes if SW1 is closed
#define START_PIN 27           // S2 START, R11 47k pull-up, press = LOW
#define MODE_PIN 5             // SW2 AUTO/MANUAL, R3 47k pull-up
#define SHIELD_POWER_PIN 19    // TPS2051 U4 enable for V50_SW (OLED + LEDs). The schematic says ~EN, but on the
                               // board HIGH = on (TestGloria 2026-11-18; Daffodil drives its TPS2051 the same way)
#define SHIELD_POWER_ON HIGH
#define SHIELD_POWER_OFF LOW
#define SENSOR_J3P2_PIN 33     // digital path of terminal J3 pin 2 (JP5, Q6); analog is U2 AIN0 (JP8)
#define SENSOR_J3P1_PIN 18     // J3 pin 1 (JP6, Q7); analog U2 AIN1 (JP7)
#define SENSOR_J6P2_PIN 32     // J6 pin 2 (JP1, Q1); analog U2 AIN2 (JP3)
#define SENSOR_J6P1_PIN 23     // J6 pin 1 (JP2, Q2); analog U2 AIN3 (JP4)

#define NUM_STATUS_LEDS 9
#define LED_BRIGHTNESS_MAX 128  // nine WS2812B near 270 mA worst case; the TPS2051 is 500 mA continuous

// ============================================================
// I2C addresses
// ============================================================
#define INA219_SOLAR_ADDR 0x40      // Valentino U8 (A0=GND, A1=GND) - solar input current
#define INA219_BATTERY_ADDR 0x41    // Valentino U9 (A0=VS,  A1=GND) - battery output current
#define GLORIA_ADS_SENSORS_ADDR 0x49  // Gloria U2, ADDR to V50: AIN0-3 = the four sensor terminals (via JP8/JP7/JP3/JP4)
#define GLORIA_ADS_CONFIG_ADDR 0x4A   // Gloria U1, ADDR to SDA: AIN2 config DIP ladder, AIN3 emergency sense
#define GLORIA_ADS_CONFIG_ADDR_ALT 0x4B  // U1 if its ADDR is ever reworked to SCL - whichever answers is used
#define GLORIA_ADS_CH_CONFIG 2
#define GLORIA_ADS_CH_EMERGENCY 3
#define EMERGENCY_DIVIDER_RATIO 3.0f  // R20 200k over R21 100k: relay coil node = pin volts x 3
#define COIL_LIVE_MIN_VOLTS 3.0f      // coil node above this = SW1 closed (about 5 V)
#define OLED_ADDR 0x3C                // OLED1, SSD1309 128x64
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define I2C_CLOCK_HZ 100000           // see the header comment: 400 kHz hangs the bus with U1 fitted
#define VALENTINO_ADS1115_ADDR 0x48 // Valentino U10 - AIN0 thermistor, AIN1 RTC coin cell, AIN2/3 STAT1/2
#define VALENTINO_ADS_CH_THERMISTOR 0  // J5 NTC divider, also the BQ24650 TS pin
#define VALENTINO_ADS_CH_RTC_BATT 1
#define VALENTINO_ADS_CH_STAT1 2       // BQ24650 STAT1, LED D10 "IN PROGRESS"
#define VALENTINO_ADS_CH_STAT2 3       // BQ24650 STAT2, LED D8 "COMPLETE"
// Thermistor divider and STAT thresholds - same values as ValentinoTest, checked on a Build 3 board
#define RAIL_3V3 3.3f
#define NTC_PULLUP_OHMS 5230.0f        // R26 from 3V3 to the thermistor node
#define NTC_PARALLEL_OHMS 30100.0f     // R27 from the thermistor node to GND
#define NTC_NOMINAL_OHMS 10000.0f
#define NTC_BETA 3435.0f               // ASSUMED (103AT-2 type)
#define NTC_MISSING_VOLTS 2.75f        // no thermistor in J5 reads about 2.81 V
#define STAT_UNPOWERED_MAX_VOLTS 0.5f  // measured 0 V with no charger input
#define STAT_ASSERTED_MAX_VOLTS 2.8f   // measured 2.51 V asserted, 3.30 V released
#define VALENTINO_ADS_GAIN_4V096 1  // +-4.096 V; the default +-2.048 V would clip the 3.3 V signals
#define MCP9808_ADDR 0x18           // Valentino U11 - board temperature, A0-A2 to GND
#define MCP9808_REG_AMBIENT 0x05
#define MCP9808_REG_MANUFACTURER 0x06
#define MCP9808_MANUFACTURER_ID 0x0054

// The Adafruit_INA219 library's default begin()/calibration assumes a 0.1ohm shunt. Valentino
// R9/R10 are 2mΩ (solar/battery), so getCurrent_mA() would be wrong; current is computed from raw
// shunt voltage divided by the actual shunt resistance instead, see readSensorData().
#define VALENTINO_SHUNT_OHMS 0.002f  // R9 (solar, U8) and R10 (battery, U9) on Valentino

// ============================================================
// LoRa send tuning (same values as Daffodil.ino's CAD/retry logic)
// ============================================================
#define CAD_RSSI_THRESHOLD -50  // RSSI above this = channel busy; restore to -85 outdoors
#define MAX_RETRIES 5
#define MIN_BACKOFF 500
#define MAX_BACKOFF 1500

// ============================================================
// Solar / power management (same operating-mode taxonomy as Daffodil.ino -
// OPERATING_STATUS_* from CommaRecord.h). Unlike Daffodil (single fixed LiFePO4 cell), Langley
// runs off a 12V battery pack whose chemistry varies by deployment. The actual thresholds/SoC
// curve/sleep-timing logic live in the shared BatteryPowerManager class (DigitalStablesEsp32Lib)
// so Gloria/Tucci (also Valentino-based) can reuse it instead of copy-pasting this per device.
// Chemistry is read from hardware at boot - see detectBatteryChemistry() below - not a compile-
// time selection. Set BATTERY_CAPACITY_MAH to match the battery actually installed before flashing.
// ============================================================
#define BATTERY_CAPACITY_MAH 7000.0  // usable capacity of the installed battery, in mAh -
                                     // <-- set to match (e.g. a 7Ah battery = 7000.0)

// NOTE: BATTERY_CAPACITY_MAH only controls firmware's LOW-battery behavior (when to sleep/COMMA).
// The Valentino Build 3 charger (BQ24650) sets the CHARGE voltage in hardware with JP7/JP8
// (configuration guide section 4): both open 13.6 V, either one closed 14.2 V, both closed 14.9 V.
// Firmware cannot read JP7/JP8, so CHARGE_REGULATION_VOLTS must be set to match them by hand.
// It is only used to name the charge stage on the web Power panel (bulk / absorption / float).
#define CHARGE_REGULATION_VOLTS 13.6f   // <-- set to match JP7/JP8
#define CHARGE_CV_MARGIN_VOLTS 0.25f    // within this of the set voltage = constant-voltage stage
                                        // (allows for the INA219 offset and wiring drop)
#define FLOAT_CURRENT_FRACTION 0.02f    // below 2% of capacity (140 mA for 7 Ah) = battery full
#define FLOAT_SETTLE_MS 60000UL         // current must stay that low for a minute before "float"

uint8_t cloudyThreshold = 70;         // forecast cloudiness % at/above which we enter CLOUDY mode
uint8_t cloudyDutyCyclePercent = 50;  // % of status-LED cycles lit in CLOUDY mode (see updateStatusLed)

// Direct solar-input current below which we're not really harvesting despite it being daytime -
// same role as Daffodil's panelCurrentCloudyThreshold_mA, and forecast-independent: it doesn't
// need a LoRa-relayed forecast to have arrived (or stayed fresh) to catch real overcast/rain.
// Confirmed necessary on 2026-08-19: a heavy-rain morning kept a unit in FULL_MODE for 8+ hours
// (Solar Current pinned at 0-25mA the whole time, Battery Current net-negative/discharging) purely
// because updateOperatingMode() had no forecast-independent cloudy signal - see updateOperatingMode().
// The 2mOhm solar shunt quantizes to ~5mA steps (VALENTINO_SHUNT_OHMS), so 30mA gives a couple of
// steps of margin above the observed 0-25mA rain-morning noise floor. Not yet validated against a
// real clear-sky midday reading - watch for false CLOUDY triggers on a genuinely sunny day.
float solarCurrentCloudyThreshold_mA = 30.0;

// Real, measured voltage on the solar input (INA219) above which we consider power actually
// available right now - true regardless of what the astronomical elevation/forecast calculation
// thinks. That calculation has no way to know about a non-solar DC source (e.g. a bench supply
// or a 19V laptop brick plugged into the solar input to run the unit continuously for testing),
// so a real measured voltage here always wins over it. Well below any real panel/adapter voltage
// (~14-19V seen in practice) but well above what an unpowered/disconnected input floats at.
#define SOLAR_INPUT_PRESENT_VOLTS 15.0f

// ============================================================
// Global objects
// ============================================================
PCF8563TimeManager timeManager(Serial);
Esp32SecretManager secretManager(timeManager);

// Vital signs (reset / sleep / power / TX telemetry), sent after the data pulse - Langley's first
// diagnostics. See VitalSignsTracker.h and Projects/Annabelle/VitalSigns_Design.pdf.
VitalSignsTracker vitalSigns;
const uint32_t FIRMWARE_BUILD = VitalSignsTracker::buildStamp(__DATE__, __TIME__);
#define VITAL_SIGNS_GAP_MS 1000                         // after the data packet, so Annabelle has read it
#define VITAL_SIGNS_AWAKE_INTERVAL_MS (10UL * 60UL * 1000UL)  // only while staying awake (FULL_MODE)
unsigned long lastVitalSignsMs = 0;
GeneralFunctions generalFunctions;
ErrorManager errorManager;

Adafruit_INA219 solarMonitor(INA219_SOLAR_ADDR);
Adafruit_INA219 batteryMonitor(INA219_BATTERY_ADDR);

ADS1115 valentinoADC(VALENTINO_ADS1115_ADDR);

// Gloria shield. U1 is looked for at 0x4A, then 0x4B (see GLORIA_ADS_CONFIG_ADDR_ALT).
ADS1115 shieldSensorADC(GLORIA_ADS_SENSORS_ADDR);
ADS1115 shieldConfigADCatSda(GLORIA_ADS_CONFIG_ADDR);
ADS1115 shieldConfigADCatScl(GLORIA_ADS_CONFIG_ADDR_ALT);
ADS1115 *shieldConfigADC = &shieldConfigADCatSda;
// OLED like Annabelle's (Adafruit_SSD1306 on the SSD1309); clock kept at I2C_CLOCK_HZ during and after transfers
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, I2C_CLOCK_HZ, I2C_CLOCK_HZ);

CRGB statusLeds[NUM_STATUS_LEDS];

LangleyData langleyData;

// The web page (data/ folder) lives in its own read-only "www" partition, flashed with each
// firmware release - see partitions.csv. Mounted lazily by startWifi(): most wakes are short
// duty-cycle wakes that never start WiFi, so they shouldn't pay for the mount.
fs::LittleFSFS wwwFS;

// Langley's web page and handlers (LangleyWifiManager, shared with Langley - not changed), plus Gloria's
// own /GloriaServlet for the pump rule editor (2026-11-18). The handlers run on the web server task:
// they only read copies and hand changes to loop() (see pumpWebSetRule()), never touch I2C or the pump.
class GloriaWifiManager : public LangleyWifiManager {
public:
  using LangleyWifiManager::LangleyWifiManager;
  std::function<void(JsonDocument &)> pumpDataProvider;
  std::function<String(const String &, long)> pumpRuleSetter;  // (rule json, authenticator code) -> "" or error
  std::function<String(long)> pumpFaultClearer;                 // (authenticator code) -> "" or error

  void start() {
    LangleyWifiManager::start();
    if (gloriaRoutesAdded) return;
    gloriaRoutesAdded = true;
    asyncWebServer.on("/GloriaServlet", HTTP_GET, [this](AsyncWebServerRequest *request) {
      String formName = request->hasParam("formName") ? request->getParam("formName")->value() : String();
      if (formName == "GetPumpData") {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        JsonDocument json;
        if (pumpDataProvider) pumpDataProvider(json);
        serializeJson(json, *response);
        request->send(response);
      } else {
        request->send(400, "text/plain", "Unknown formName");
      }
    });
    asyncWebServer.on("/GloriaServlet", HTTP_POST, [this](AsyncWebServerRequest *request) {
      String formName = param(request, "formName");
      long code = param(request, "code").toInt();
      String err = "Unknown formName";
      if (formName == "SetPumpRule" && pumpRuleSetter) err = pumpRuleSetter(param(request, "rule"), code);
      else if (formName == "ClearPumpFault" && pumpFaultClearer) err = pumpFaultClearer(code);
      if (err.length()) request->send(400, "text/plain", err);
      else request->send(200, "text/plain", "ok");
    });
  }

private:
  bool gloriaRoutesAdded = false;
  static String param(AsyncWebServerRequest *request, const char *name) {
    const AsyncWebParameter *p = request->getParam(name, true);
    return p ? p->value() : String();
  }
};
GloriaWifiManager wifiManager(Serial, wwwFS, timeManager, secretManager, langleyData);
bool initiatedWifi = false;   // wifiManager.start() (routes + first connect) has run once
bool wwwMounted = false;
bool wifiRunning = false;
uint16_t wifiWantedSeconds = 0;       // how long wifiAllowed() has been continuously true
unsigned long lastInternetCheckMs = 0;
String ipAddress = "";
volatile bool petWatchdogDuringWifiStart = false;

uint8_t loraSendIntervalSeconds = 60;  // how often we transmit (FULL_MODE) - pulses arrive roughly once
                                       // a second, so several get aggregated into each send (see
                                       // pulseVoltageMin/Max/Sum/CountThisInterval below). Was 10 s until
                                       // 2026-11-11: ~0.9 s on air every 10 s took ~9% of the channel.

// How long to stay awake sampling each duty-cycle wake before calling goToSleep(), which sends the
// wake's one packet - see loop()'s send/sleep block (checked every second since 2026-11-11).
#define AWAKE_WINDOW_SECONDS 30

// Battery current during deep sleep, used by the overnight-mAh accumulator (goToSleep()). The
// INA219 is only read while awake, so the sleep part of each cycle can't be measured by the
// firmware itself. Estimate - replace with a multimeter reading in the battery + lead taken
// between wakes (Valentino Build 3 + Langley, 2026-10-22).
#define SLEEP_CURRENT_MA 3.0f

Timer displayTimer(2);                     // seconds between sensor read + display refresh
Timer sendTimer(loraSendIntervalSeconds);  // seconds between LoRa updates

bool debug = true;
bool foundSolarMonitor = false;
bool foundBatteryMonitor = false;
bool foundBoardTempSensor = false;
bool foundValentinoADC = false;

// Gloria shield state - read in readGloriaShield(), shown on the OLED and the web Power panel only
// (not sent over LoRa yet: Gloria still sends LangleyData).
bool foundOled = false;
bool foundShieldSensorADC = false;
bool foundShieldConfigADC = false;
uint8_t shieldConfigAddr = GLORIA_ADS_CONFIG_ADDR;
bool shieldPowered = false;
bool pumpRelayOn = false;     // GPIO17 level we drive; the relay only closes if SW1 is closed too
bool startPressed = false;    // debounced in pollGloriaInputs()
bool modeSwitchLow = false;   // SW2 pin LOW; which position is AUTO is decided with the case
float coilNodeVolts = NAN;    // relay coil supply after SW1: about 5 V = SW1 closed
bool emergencyClosed = false; // SW1 closed (normal); false = emergency position, relay cannot close
int16_t configLadderRaw = 0;  // config DIP ladder, U1 AIN2 - table of positions still to be measured
float shieldSensorVolts[4] = { NAN, NAN, NAN, NAN };  // U2 AIN0-3
uint8_t shieldDigital[4] = { 0, 0, 0, 0 };            // GPIO 33, 18, 32, 23

// Valentino charger and battery temperature - web Power panel only, not sent over LoRa (Annabelle
// identifies LangleyData by its size). Read in readSensorData(), copied by fillPowerData().
enum ChargerState { CHARGER_UNKNOWN, CHARGER_NO_INPUT, CHARGER_CHARGING, CHARGER_COMPLETE, CHARGER_SUSPENDED, CHARGER_INVALID };
ChargerState chargerState = CHARGER_UNKNOWN;
float stat1Volts = 0, stat2Volts = 0;

// Charge stage, worked out from the battery voltage and current while the charger is charging.
// The BQ24650 has one regulation voltage: in lead-acid mode (JP3 fitted, termination off) it holds
// it for ever, so "float" here means absorption has tapered to a trickle - the battery is full.
enum ChargeStage { STAGE_NONE, STAGE_BULK, STAGE_ABSORPTION, STAGE_FLOAT, STAGE_COMPLETE };
ChargeStage chargeStage = STAGE_NONE;
unsigned long lowCurrentSinceMs = 0;  // when the charge current first fell below the float level
bool batteryThermistorPresent = false;
float batteryThermistorC = NAN;

// Last VitalSignsRecord built, for the Power panel (same as Daffodil.ino)
VitalSignsRecord lastVitalSigns;
bool haveLastVitalSigns = false;
unsigned long lastVitalSignsBuiltMs = 0;
portMUX_TYPE lastVitalSignsMux = portMUX_INITIALIZER_UNLOCKED;
bool loraActive = false;

volatile bool clockTicked = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

float avgRssi = 0;

// ---- Solar / weather (filled in from secretManager in setup(), then held here so
//      updateOperatingMode() can use them every cycle) ----
double latitude = 0, longitude = 0, altitude = 0;
uint8_t minimumEfficiencyForLed = 0;
bool usingSolarPower = true;  // Gloria is always panel-powered (as Langley)
String timezone = "";
char sensor1name[16] = { 0 };
char sensor2name[16] = { 0 };
uint8_t minimumEfficiencyForWifi = 0;

SolarInfo *solarInfo = nullptr;
BatteryPowerManager *batteryPowerManager = nullptr;
RTCInfoRecord currentTimerRecord;
HourlySolarPowerData hourlySolarPowerData;

// Survives deep-sleep reboots (unlike a plain bool) so a COMMA condition isn't silently forgotten
// and re-evaluated from scratch every wake - mirrors Daffodil.ino's rtc_comma_mode. Self-clears
// once a quick voltage check on wake shows recovery - see setup()'s pre-boot check.
RTC_DATA_ATTR bool rtc_comma_mode = false;

// Weather forecast relayed over LoRa from the base station (Langley's WiFi is a local web page
// only, so unlike Daffodil it never calls WeatherForecastManager::downloadWeatherData() directly -
// see processLoraPacket()). SolarInfo keeps its own internal copy once setWeatherForecast() is called;
// this local copy exists only so updateOperatingMode() can inspect forecasts[0].cloudiness.
// RTC_DATA_ATTR so a received forecast survives the deep-sleep reboots goToSleep() now does -
// plain globals would be wiped (re-initialized by setup()) on every wake. Freshness is tracked
// in wall-clock seconds (langleyData.secondsTime), not millis() - millis() resets to 0 on every
// deep-sleep reboot so a millis()-based staleness check would never see real elapsed time.
RTC_DATA_ATTR WeatherForecast weatherForecasts[4];
RTC_DATA_ATTR bool haveWeatherForecast = false;
RTC_DATA_ATTR long lastWeatherForecastSeconds = 0;
// How strongly the last accepted forecast was heard - this unit's link to Annabelle, reported in the
// answer to Paula's ping (LinkAckRecord.gatewayRssi). 0 = none since the last real reset.
RTC_DATA_ATTR int16_t rtc_weatherRssi = 0;
RTC_DATA_ATTR int8_t rtc_weatherSnrQ = 0;
#define WEATHER_FRESH_SECONDS 1860L  // 31 min, same staleness window as Daffodil

// WiFi on demand, same scheme as Daffodil.ino (2026-10-14): a DeviceCommandRecord from Annabelle's
// web page or Paula turns WiFi on for a while regardless of the operating mode (and keeps the unit
// awake for it), or off. See acceptDeviceCommand() and listenForReplies().
#define WIFI_ON_DEMAND_DEFAULT_MIN 15
#define WIFI_ON_DEMAND_MAX_MIN 60
// On-demand WiFi ends when the battery (under load) drops below sleepingVoltage() plus this, so a
// low battery still gets to sleep - Daffodil's 3.20 V floor sits the same way above its 3.12 V.
#define WIFI_ON_DEMAND_FLOOR_MARGIN_V 0.2f
#define WIFI_OFF_SUPPRESS_AUTO_SEC 3600  // after an explicit "off", the automatic start waits this long
#define COMMAND_LISTEN_MS 700            // listen after each VitalSigns: sender's 100 ms + CAD + ~0.19 s on air + margin
#define WEATHER_REPLY_LISTEN_MS 2500     // listen this long instead when the forecast is stale
// 1 = listen after every VitalSigns, day and night. 0 = daylight only: no night cost, a night
// command waits for sunrise.
#define LISTEN_FOR_COMMANDS_AT_NIGHT 1
#define DEVICE_COMMAND_MAX_CLOCK_DIFF_SEC 900
RTC_DATA_ATTR uint32_t wifiOnDemandUntil = 0;        // code-time seconds (getTimeForCodeGeneration), 0 = off
RTC_DATA_ATTR uint32_t wifiAutoSuppressedUntil = 0;  // code-time seconds, see WIFI_OFF_SUPPRESS_AUTO_SEC
bool wifiOffRequested = false;                       // set by acceptDeviceCommand(), acted on in manageWifi()

// Overnight battery-sizing calibration run (see conversation 2026-07-14) - real coulomb-counted
// mAh consumed since sunset, persisted across the deep-sleep reboots goToSleep() does. Armed the
// first sleep after daytime solar efficiency drops <=0.3, frozen again once efficiency recovers
// (sunrise). Reported every send (langleyData.overnightMah) so it's visible live overnight via
// Hippocampus/webapp, to be checked against actual voltage sag the next morning.
RTC_DATA_ATTR float overnightMah = 0.0f;
RTC_DATA_ATTR bool overnightTrackingActive = false;
bool cloudyLedCycleOn = true;    // toggles every status-LED refresh while CLOUDY, see updateStatusLed()

// ============================================================
// Interrupt service routines
// ============================================================
void IRAM_ATTR clockTick() {
  portENTER_CRITICAL_ISR(&mux);
  clockTicked = true;
  portEXIT_CRITICAL_ISR(&mux);
}

// ============================================================
// Helpers
// ============================================================
void resetI2CDevices() {
  Wire.beginTransmission(0x00);  // General Call Address
  Wire.write(0x06);              // Reset command code
  Wire.endTransmission();
  delay(10);
}

// Refreshes every field of langleyData from the live sensors. Mirrors Daffodil.ino's
// readSensorData() - called both on the display-refresh cadence and immediately before
// every LoRa send, so a transmitted packet is never more than a few seconds stale.
//
// devicename/deviceshortname are set once in setup() from secretManager and not touched here.
// operatingStatus is set by updateOperatingMode(); sleepTime/wifiStatus/opMode right before each
// send (sendLangleyData()). rssi/snr are filled in by the receiver.
void readSensorData() {
  if (foundSolarMonitor) {
    // A single instantaneous shunt sample is a bad estimator of Solar Current: the Valentino
    // charger (LM25085, a non-synchronous P-FET+Schottky buck - see
    // Valentino_LM25085_OCP_bugreport.pdf) draws genuinely discontinuous/pulsed input current -
    // current flows from the panel only during the FET's brief on-time each switching cycle, zero
    // the rest of the time. One sample can land anywhere in that cycle. Confirmed on Langley_E
    // 2026-08-22 14:14:54: a single-sample Solar Current of 285mA alongside a simultaneous Battery
    // Current of 715mA implies >180% conversion efficiency - not a real reading, just an aliased
    // snapshot. Average several back-to-back samples (same approach as the RTC battery-voltage
    // read below) to span multiple switching cycles instead of trusting one.
    float solarBusSumV = 0;
    float solarShuntSumMv = 0;
    const uint8_t solarSamples = 20;
    for (uint8_t i = 0; i < solarSamples; i++) {
      solarBusSumV += solarMonitor.getBusVoltage_V();
      solarShuntSumMv += solarMonitor.getShuntVoltage_mV();
      delay(1);
    }
    langleyData.solarVoltage = solarBusSumV / solarSamples;
    langleyData.solarCurrentMa = (solarShuntSumMv / solarSamples) / VALENTINO_SHUNT_OHMS;
  }
  if (foundBatteryMonitor) {
    langleyData.batteryVoltage = batteryMonitor.getBusVoltage_V();
    langleyData.batteryCurrentMa = batteryMonitor.getShuntVoltage_mV() / VALENTINO_SHUNT_OHMS;
    // Gloria has no external battery: LangleyData.externalBatteryVoltage stays 0

    // Estimated runtime via the shared BatteryPowerManager (chemistry-appropriate SoC curve x
    // rated capacity, divided by the present discharge current) - this is still the instantaneous
    // discharge rate, not duty-cycle-averaged, since the awake window between sleeps is brief and
    // roughly constant-load, unlike Daffodil's WiFi-driven variable load.
    // Sign convention here is the opposite of Daffodil's: batteryCurrentMa is positive = charging,
    // so discharging shows up as negative.
    if (langleyData.batteryCurrentMa < -1.0 && batteryPowerManager != nullptr) {
      float dischargeMa = abs(langleyData.batteryCurrentMa);
      langleyData.estimatedRuntime = batteryPowerManager->estimatedRuntimeHours(langleyData.batteryVoltage, dischargeMa);
    } else {
      langleyData.estimatedRuntime = 0.0;  // charging or unknown
    }
  }
  // RTC backup coin-cell (CR1220) voltage, via the R41(1M)/R46(2M) divider on the Valentino
  // ADS1115 AIN1 (Build 3). -1 = ADS1115 not found.
  if (foundValentinoADC) {
    float voltage = valentinoADC.toVoltage(valentinoADC.readADC(VALENTINO_ADS_CH_RTC_BATT));
    langleyData.rtcBatVolt = voltage * (RTC_BATT_DIVIDER_R1 + RTC_BATT_DIVIDER_R2) / RTC_BATT_DIVIDER_R2;
  } else {
    langleyData.rtcBatVolt = -1;
  }

  // Battery thermistor and charger status, from the same ADS1115 (Build 3)
  if (foundValentinoADC) {
    float ntcVolts = valentinoADC.toVoltage(valentinoADC.readADC(VALENTINO_ADS_CH_THERMISTOR));
    batteryThermistorPresent = ntcVolts < NTC_MISSING_VOLTS;
    if (batteryThermistorPresent) {
      float parallelOhms = NTC_PULLUP_OHMS * ntcVolts / (RAIL_3V3 - ntcVolts);
      float ntcOhms = 1.0f / (1.0f / parallelOhms - 1.0f / NTC_PARALLEL_OHMS);
      batteryThermistorC = 1.0f / (1.0f / 298.15f + logf(ntcOhms / NTC_NOMINAL_OHMS) / NTC_BETA) - 273.15f;
    } else {
      batteryThermistorC = NAN;
    }
    // in the data packet (LangleyData.batteryTemperature, 2026-11-12) - until now only on Langley's own page
    langleyData.batteryTemperature = isnan(batteryThermistorC) ? -127.0f : batteryThermistorC;
    stat1Volts = valentinoADC.toVoltage(valentinoADC.readADC(VALENTINO_ADS_CH_STAT1));
    stat2Volts = valentinoADC.toVoltage(valentinoADC.readADC(VALENTINO_ADS_CH_STAT2));
    chargerState = decodeCharger(stat1Volts, stat2Volts);
    updateChargeStage();
  }

  if (foundBoardTempSensor) {
    float boardC = readBoardCelsius();
    if (!isnan(boardC)) langleyData.temperature = boardC;
  }

  langleyData.secondsSinceLastPulse = -1;  // no fence on Gloria
  readGloriaShield();

  langleyData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);

  // Reflect the persisted overnight-mAh total every call, not just right before sleep - goToSleep()
  // is the only other writer of this field, but it's never called while in FULL_MODE, so without
  // this the telemetry showed a stale struct-default 0 for the entire day instead of the previous
  // night's frozen total. See updateOperatingMode() for the matching disarm-on-daylight fix.
  langleyData.overnightMah = overnightMah;

  if (debug) {
    Serial.print("solarV=");
    Serial.print(langleyData.solarVoltage);
    Serial.print(" solarMa=");
    Serial.print(langleyData.solarCurrentMa);
    Serial.print(" battV=");
    Serial.print(langleyData.batteryVoltage);
    Serial.print(" battMa=");
    Serial.print(langleyData.batteryCurrentMa);
    Serial.print(" estRuntimeHrs=");
    Serial.print(langleyData.estimatedRuntime);
    Serial.print(" chem=");
    Serial.print(langleyData.batteryChemistry);
    Serial.print(" rtcBatVolt=");
    Serial.print(langleyData.rtcBatVolt);
    Serial.print(" temp=");
    Serial.print(langleyData.temperature);
    Serial.print(" relay=");
    Serial.print(pumpRelayOn ? "ON" : "off");
    Serial.print(" SW1=");
    Serial.print(foundShieldConfigADC ? (emergencyClosed ? "closed" : "OPEN") : "n/a");
    Serial.print(" overnightMah=");
    Serial.println(langleyData.overnightMah);
  }
}

// ============================================================
// Gloria shield
// ============================================================
// V50_SW (TPS2051) feeds the OLED and the LEDs only - U1, U2, SW1 and the relay coil are on V50,
// which is always on. Before switching off, the LEDs are sent black so the data line idles low and
// cannot back-power the first LED.
void setShieldPower(bool on) {
  if (!on) {
    FastLED.clear(true);
    if (foundOled) {
      display.clearDisplay();
      display.display();
    }
  }
  digitalWrite(SHIELD_POWER_PIN, on ? SHIELD_POWER_ON : SHIELD_POWER_OFF);
  shieldPowered = on;
}

void setPumpRelay(bool on) {
  pumpRelayOn = on;
  digitalWrite(PUMP_RELAY_PIN, on ? HIGH : LOW);
}

bool oledBegin() {
  if (!i2cDevicePresent(OLED_ADDR)) return false;
  return display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false);  // Wire already started
}

bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// U1 (config DIP + emergency sense) and U2 (sensor inputs), and the four digital inputs.
void readGloriaShield() {
  if (foundShieldConfigADC) {
    coilNodeVolts = shieldConfigADC->toVoltage(shieldConfigADC->readADC(GLORIA_ADS_CH_EMERGENCY)) * EMERGENCY_DIVIDER_RATIO;
    emergencyClosed = coilNodeVolts > COIL_LIVE_MIN_VOLTS;
    configLadderRaw = shieldConfigADC->readADC(GLORIA_ADS_CH_CONFIG);
  }
  if (foundShieldSensorADC) {
    for (uint8_t ch = 0; ch < 4; ch++) shieldSensorVolts[ch] = shieldSensorADC.toVoltage(shieldSensorADC.readADC(ch));
  }
  shieldDigital[0] = digitalRead(SENSOR_J3P2_PIN);
  shieldDigital[1] = digitalRead(SENSOR_J3P1_PIN);
  shieldDigital[2] = digitalRead(SENSOR_J6P2_PIN);
  shieldDigital[3] = digitalRead(SENSOR_J6P1_PIN);
}

// START and AUTO/MANUAL, debounced; called every loop() iteration. Only reported for now - the pump
// logic that uses them is still to be written.
#define SWITCH_DEBOUNCE_MS 30
void pollGloriaInputs() {
  static bool startRaw = false, modeRaw = false;
  static uint32_t startChangedMs = 0, modeChangedMs = 0;
  uint32_t now = millis();
  bool start = digitalRead(START_PIN) == LOW;
  if (start != startRaw) {
    startRaw = start;
    startChangedMs = now;
  } else if (startRaw != startPressed && now - startChangedMs >= SWITCH_DEBOUNCE_MS) {
    startPressed = startRaw;
    if (debug) Serial.printf("START %s\n", startPressed ? "pressed" : "released");
    if (startPressed) onStartPressed();
  }
  bool modeLow = digitalRead(MODE_PIN) == LOW;
  if (modeLow != modeRaw) {
    modeRaw = modeLow;
    modeChangedMs = now;
  } else if (modeRaw != modeSwitchLow && now - modeChangedMs >= SWITCH_DEBOUNCE_MS) {
    modeSwitchLow = modeRaw;
    if (debug) Serial.printf("Mode switch: pin %s\n", modeSwitchLow ? "LOW" : "HIGH");
  }
}

// ============================================================
// Pump control - the user's rule for AUTO, START for MANUAL, and the protections. See GloriaPumpRule.h.
// ============================================================
#define FLOW_PIN SENSOR_J3P2_PIN  // flow meter on terminal J3 pin 2 (JP5 closed, JP8 open)
volatile uint32_t flowPulses = 0;
void IRAM_ATTR flowPulseISR() {
  flowPulses++;
}
float flowRateLpm = 0;      // over the last second
float flowTotalLitres = 0;  // since boot

PumpRule pumpRule;
Preferences gloriaPrefs;

// Every device Gloria has heard: its last packet as raw bytes, under its name (as Daffodil's Viewer).
constexpr size_t heardMaxSize(size_t a, size_t b) {
  return a > b ? a : b;
}
#define HEARD_RAW_MAX heardMaxSize(sizeof(DigitalStablesData), heardMaxSize(sizeof(ChinampaData), sizeof(LangleyData)))
struct HeardDevice {
  bool used;
  uint8_t type;
  uint32_t atMs;  // millis() when heard - Gloria stays awake while it runs the rule
  int16_t rssi;
  char name[HEARD_NAME_LEN];
  uint8_t raw[HEARD_RAW_MAX];
};
HeardDevice heardDevices[HEARD_MAX_DEVICES];
portMUX_TYPE heardMux = portMUX_INITIALIZER_UNLOCKED;  // the web task copies the table (fillPumpData)

// Changes from the web page, applied by loop() (pumpApplyWebChanges) - the web task never changes the rule itself
PumpRule pumpRulePending;
volatile bool pumpRulePendingSet = false;
volatile bool pumpFaultClearPending = false;
portMUX_TYPE pumpRuleMux = portMUX_INITIALIZER_UNLOCKED;

extern int loraRxRssi;  // defined with the LoRa receive buffer below
PumpReason pumpReason = PUMP_IDLE;
bool pumpDataStale = false;    // all LEDs yellow
bool pumpDryRunFault = false;  // LEDs 4-8 red
uint32_t pumpOnSinceMs = 0;
uint32_t flowZeroSinceMs = 0;

const char *pumpReasonText(PumpReason r) {
  switch (r) {
    case PUMP_IDLE: return "idle";
    case PUMP_ON_AUTO: return "rule holds";
    case PUMP_ON_MANUAL: return "START";
    case PUMP_OFF_STOP_RULE: return "stop rule";
    case PUMP_OFF_NO_RULE: return "no rule set";
    case PUMP_OFF_STALE: return "data stale";
    case PUMP_OFF_DRY_RUN: return "NO FLOW";
    case PUMP_OFF_LOW_BATTERY: return "battery low";
    case PUMP_OFF_MANUAL_TIME: return "time limit";
    case PUMP_OFF_MANUAL_LOW_V: return "battery min";
    default: return "?";
  }
}

bool autoMode() {
  return modeSwitchLow == pumpRule.autoWhenLow;
}

bool batteryTooLowToPump() {
  return langleyData.operatingStatus == OPERATING_STATUS_SLEEP || langleyData.operatingStatus == OPERATING_STATUS_COMMA;
}

int pumpTypeIndex(const char *id) {
  for (uint8_t i = 0; i < PUMP_RULE_TYPE_COUNT; i++)
    if (strcmp(PUMP_RULE_TYPES[i].id, id) == 0) return i;
  return -1;
}

int pumpFieldIndex(int type, const char *id) {
  for (uint8_t i = 0; i < PUMP_RULE_FIELD_COUNT; i++)
    if (PUMP_RULE_FIELDS[i].type == type && strcmp(PUMP_RULE_FIELDS[i].id, id) == 0) return i;
  return -1;
}

int pumpOpIndex(const char *op) {
  for (uint8_t i = 0; i < 4; i++)
    if (strcmp(PUMP_OP_TEXT[i], op) == 0) return i;
  return -1;
}

// processLoraPacket(): a packet from a device the rule can use. Kept under the device's name - the
// same device again replaces its entry, a new one takes an empty or the oldest entry.
void heardRecord(uint8_t type, const char *name, size_t nameLen, const void *data, size_t len) {
  char n[HEARD_NAME_LEN];
  size_t k = 0;
  for (; k < nameLen && k < HEARD_NAME_LEN - 1 && name[k]; k++) n[k] = name[k];
  n[k] = 0;
  while (k > 0 && n[k - 1] == ' ') n[--k] = 0;
  if (!n[0]) return;  // no name: the rule could never refer to it
  int slot = -1, oldest = 0;
  for (int i = 0; i < HEARD_MAX_DEVICES; i++) {
    if (heardDevices[i].used && heardDevices[i].type == type && strcmp(heardDevices[i].name, n) == 0) {
      slot = i;
      break;
    }
    if (!heardDevices[i].used || (heardDevices[oldest].used && heardDevices[i].atMs < heardDevices[oldest].atMs)) oldest = i;
  }
  if (slot < 0) slot = oldest;
  portENTER_CRITICAL(&heardMux);
  HeardDevice &h = heardDevices[slot];
  h.used = true;
  h.type = type;
  h.atMs = millis();
  h.rssi = (int16_t)loraRxRssi;
  strcpy(h.name, n);
  memset(h.raw, 0, sizeof(h.raw));
  memcpy(h.raw, data, len < sizeof(h.raw) ? len : sizeof(h.raw));
  portEXIT_CRITICAL(&heardMux);
  if (debug) Serial.printf(" pump rule: heard %s %s (%d dBm)\n", PUMP_RULE_TYPES[type].name, n, (int)loraRxRssi);
}

int heardFind(uint8_t type, const char *name) {
  for (int i = 0; i < HEARD_MAX_DEVICES; i++)
    if (heardDevices[i].used && heardDevices[i].type == type && strcmp(heardDevices[i].name, name) == 0) return i;
  return -1;
}

// A level from a sonar distance: measured is the distance down to the surface (as Daffodil's Viewer).
bool levelFrom(float measured, float height, bool percent, float &v) {
  if (measured <= 0 || height <= 0) return false;  // -99 = no reading
  v = height - measured;
  if (percent) v = v / height * 100.0f;
  return true;
}

// A field's value from a device's last packet. False when the packet has no reading for it.
bool heardFieldValue(uint8_t fi, const uint8_t *raw, float &v) {
  switch (PUMP_RULE_FIELDS[fi].type) {
    case PUMP_RULE_TYPE_DAFFODIL:
      {
        DigitalStablesData d;
        memcpy(&d, raw, sizeof(d));
        switch (fi) {
          case PF_DA_LEVEL_CM: return levelFrom(d.measuredHeight, d.maximumScepticHeight, false, v);
          case PF_DA_LEVEL_PCT: return levelFrom(d.measuredHeight, d.maximumScepticHeight, true, v);
          case PF_DA_FLOW1: v = d.flowRate; return v >= 0;
          case PF_DA_FLOW2: v = d.flowRate2; return v >= 0;
          case PF_DA_OUTDOOR_TEMP: v = d.outdoortemperature; return v != -99;
        }
        return false;
      }
    case PUMP_RULE_TYPE_CHINAMPA:
      {
        ChinampaData c;
        memcpy(&c, raw, sizeof(c));
        switch (fi) {
          case PF_CH_SUMP_LEVEL_CM: return levelFrom(c.sumpTroughMeasuredHeight, c.sumpTroughHeight, false, v);
          case PF_CH_SUMP_LEVEL_PCT: return levelFrom(c.sumpTroughMeasuredHeight, c.sumpTroughHeight, true, v);
          case PF_CH_FISH_LEVEL_CM: return levelFrom(c.fishTankMeasuredHeight, c.fishTankHeight, false, v);
          case PF_CH_FISH_LEVEL_PCT: return levelFrom(c.fishTankMeasuredHeight, c.fishTankHeight, true, v);
          case PF_CH_PUMP_FLOW: v = c.pumpflowRate; return v >= 0;
          case PF_CH_FISH_OUTFLOW: v = c.fishtankoutflowflowRate; return v >= 0;
          case PF_CH_OUTDOOR_TEMP: v = c.outdoortemperature; return v != -99;
        }
        return false;
      }
    case PUMP_RULE_TYPE_LANGLEY:
      {
        LangleyData l;
        memcpy(&l, raw, sizeof(l));
        if (fi == PF_LA_TEMP) {
          v = l.temperature;
          return v != -99;
        }
        if (fi == PF_LA_BATTERY) {
          v = l.batteryVoltage;
          return v > 0;
        }
        return false;
      }
  }
  return false;
}

#define COND_FALSE 0
#define COND_TRUE 1
#define COND_NO_DATA 2
uint8_t pumpConditionEval(const PumpCondition &c, float &value) {
  int i = heardFind(PUMP_RULE_FIELDS[c.field].type, c.device);
  if (i < 0) return COND_NO_DATA;
  if (millis() - heardDevices[i].atMs > (uint32_t)pumpRule.staleSec * 1000UL) return COND_NO_DATA;
  if (!heardFieldValue(c.field, heardDevices[i].raw, value)) return COND_NO_DATA;
  switch (c.op) {
    case PUMP_OP_LT: return value < c.value ? COND_TRUE : COND_FALSE;
    case PUMP_OP_LE: return value <= c.value ? COND_TRUE : COND_FALSE;
    case PUMP_OP_GT: return value > c.value ? COND_TRUE : COND_FALSE;
    case PUMP_OP_GE: return value >= c.value ? COND_TRUE : COND_FALSE;
  }
  return COND_NO_DATA;
}

String pumpConditionsParse(JsonArrayConst list, PumpCondition *dst, uint8_t &count, const char *which) {
  count = 0;
  if (list.size() > PUMP_RULE_MAX_CONDITIONS) return String("more than ") + PUMP_RULE_MAX_CONDITIONS + " " + which + " conditions";
  for (JsonObjectConst c : list) {
    int type = pumpTypeIndex(c["t"] | "");
    if (type < 0) return String(which) + ": unknown device type \"" + (const char *)(c["t"] | "") + "\"";
    int field = pumpFieldIndex(type, c["f"] | "");
    if (field < 0) return String(which) + ": unknown variable \"" + (const char *)(c["f"] | "") + "\"";
    const char *dev = c["d"] | "";
    if (!dev[0] || strlen(dev) >= HEARD_NAME_LEN) return String(which) + ": device name missing or too long";
    int op = pumpOpIndex(c["op"] | "");
    if (op < 0) return String(which) + ": op must be <, <=, > or >=";
    if (!c["v"].is<float>()) return String(which) + ": value missing";
    PumpCondition &p = dst[count++];
    p.field = field;
    strncpy(p.device, dev, HEARD_NAME_LEN - 1);
    p.device[HEARD_NAME_LEN - 1] = 0;
    p.op = op;
    p.value = c["v"].as<float>();
  }
  return "";
}

// "" when the JSON is a valid rule (then in out), else what is wrong with it.
String pumpRuleParse(const String &json, PumpRule &out) {
  JsonDocument doc;
  if (deserializeJson(doc, json)) return "not valid JSON";
  PumpRule r;
  r.staleSec = doc["stale"] | 600;
  r.dryRunSec = doc["dryRunSec"] | 5;
  r.manualMin = doc["manualMin"] | 30;
  r.manualMinV = doc["manualMinV"] | 12.0f;
  r.qfactor = doc["qfactor"] | 4.5f;
  r.autoWhenLow = doc["autoWhenLow"] | false;
  if (r.staleSec < 30) return "stale must be at least 30 s";
  if (r.dryRunSec < 2) return "dryRunSec must be at least 2";
  if (r.qfactor <= 0) return "qfactor must be above 0";
  String e = pumpConditionsParse(doc["start"].as<JsonArrayConst>(), r.start, r.startCount, "start");
  if (e.length()) return e;
  e = pumpConditionsParse(doc["stop"].as<JsonArrayConst>(), r.stop, r.stopCount, "stop");
  if (e.length()) return e;
  out = r;
  return "";
}

String pumpRuleToJson(const PumpRule &r) {
  JsonDocument doc;
  doc["stale"] = r.staleSec;
  doc["dryRunSec"] = r.dryRunSec;
  doc["manualMin"] = r.manualMin;
  doc["manualMinV"] = r.manualMinV;
  doc["qfactor"] = r.qfactor;
  doc["autoWhenLow"] = r.autoWhenLow;
  for (uint8_t list = 0; list < 2; list++) {
    JsonArray a = doc[list == 0 ? "start" : "stop"].to<JsonArray>();
    const PumpCondition *cs = list == 0 ? r.start : r.stop;
    uint8_t n = list == 0 ? r.startCount : r.stopCount;
    for (uint8_t i = 0; i < n; i++) {
      JsonObject c = a.add<JsonObject>();
      c["t"] = PUMP_RULE_TYPES[PUMP_RULE_FIELDS[cs[i].field].type].id;
      c["d"] = cs[i].device;
      c["f"] = PUMP_RULE_FIELDS[cs[i].field].id;
      c["op"] = PUMP_OP_TEXT[cs[i].op];
      c["v"] = cs[i].value;
    }
  }
  String out;
  serializeJson(doc, out);
  return out;
}

void pumpRuleLoad() {
  gloriaPrefs.begin("gloria", true);
  String json = gloriaPrefs.getString("rule", PUMP_RULE_DEFAULT);
  gloriaPrefs.end();
  String e = pumpRuleParse(json, pumpRule);
  if (e.length()) {
    if (debug) Serial.println("Saved pump rule not usable (" + e + ") - using the default (no conditions)");
    pumpRuleParse(PUMP_RULE_DEFAULT, pumpRule);
  }
}

// "" and saved when valid, else what is wrong (nothing changes then)
String pumpRuleSet(const String &json) {
  PumpRule r;
  String e = pumpRuleParse(json, r);
  if (e.length()) return e;
  pumpRule = r;
  gloriaPrefs.begin("gloria", false);
  gloriaPrefs.putString("rule", pumpRuleToJson(pumpRule));
  gloriaPrefs.end();
  return "";
}

void pumpSet(bool on, PumpReason reason) {
  if (on && !pumpRelayOn) {
    pumpOnSinceMs = millis();
    flowZeroSinceMs = millis();
  }
  if (on != pumpRelayOn || reason != pumpReason) {
    if (debug) Serial.printf("Pump %s (%s)\n", on ? "ON" : "off", pumpReasonText(reason));
  }
  setPumpRelay(on);
  pumpReason = reason;
}

// Pulses counted by flowPulseISR over the time since the last call (about 1 s, from loop()'s tick)
void updateFlow() {
  static uint32_t lastPulses = 0, lastMs = 0;
  uint32_t now = millis();
  uint32_t pulses = flowPulses;
  uint32_t d = pulses - lastPulses;
  lastPulses = pulses;
  float sec = (now - lastMs) / 1000.0f;
  lastMs = now;
  if (sec <= 0 || pumpRule.qfactor <= 0) return;
  flowRateLpm = (d / sec) / pumpRule.qfactor;
  flowTotalLitres += d / (pumpRule.qfactor * 60.0f);
}

// START: clears a no-flow fault first; in MANUAL it then toggles the pump. Does nothing else in AUTO.
void onStartPressed() {
  if (pumpDryRunFault) {
    pumpDryRunFault = false;
    pumpSet(false, PUMP_IDLE);
    if (debug) Serial.println("No-flow fault cleared by START");
    return;
  }
  if (autoMode()) return;
  if (pumpRelayOn) {
    pumpSet(false, PUMP_IDLE);
  } else if (batteryTooLowToPump()) {
    pumpSet(false, PUMP_OFF_LOW_BATTERY);
  } else if (langleyData.batteryVoltage > 0 && langleyData.batteryVoltage < pumpRule.manualMinV) {
    pumpSet(false, PUMP_OFF_MANUAL_LOW_V);
  } else {
    pumpSet(true, PUMP_ON_MANUAL);
  }
  updateStatusLed();
  updateDisplays();
}

// Once a second from loop()
void pumpControl() {
  static bool lastAuto = autoMode();
  bool isAuto = autoMode();
  if (isAuto != lastAuto) {  // the AUTO/MANUAL switch moved: start from off either way
    lastAuto = isAuto;
    if (debug) Serial.printf("Pump mode %s\n", isAuto ? "AUTO" : "MANUAL");
    pumpSet(false, PUMP_IDLE);
  }
  pumpDataStale = false;

  if (pumpDryRunFault) {
    pumpSet(false, PUMP_OFF_DRY_RUN);
    return;
  }
  if (batteryTooLowToPump()) {
    pumpSet(false, PUMP_OFF_LOW_BATTERY);
    return;
  }

  if (isAuto) {
    if (pumpRule.startCount == 0) {
      pumpSet(false, PUMP_OFF_NO_RULE);
      return;
    }
    bool allStart = true, anyStop = false, noData = false;
    float v;
    for (uint8_t i = 0; i < pumpRule.startCount; i++) {
      uint8_t r = pumpConditionEval(pumpRule.start[i], v);
      if (r == COND_NO_DATA) noData = true;
      else if (r == COND_FALSE) allStart = false;
    }
    for (uint8_t i = 0; i < pumpRule.stopCount; i++) {
      uint8_t r = pumpConditionEval(pumpRule.stop[i], v);
      if (r == COND_NO_DATA) noData = true;
      else if (r == COND_TRUE) anyStop = true;
    }
    if (noData) {
      pumpDataStale = true;
      pumpSet(false, PUMP_OFF_STALE);
      return;
    }
    if (pumpRelayOn) {
      if (anyStop || (pumpRule.stopCount == 0 && !allStart)) pumpSet(false, PUMP_OFF_STOP_RULE);
    } else if (allStart && !anyStop) {
      pumpSet(true, PUMP_ON_AUTO);
    } else if (pumpReason != PUMP_OFF_STOP_RULE || !anyStop) {
      pumpSet(false, anyStop ? PUMP_OFF_STOP_RULE : PUMP_IDLE);
    }
  } else if (pumpRelayOn) {
    if (millis() - pumpOnSinceMs >= (uint32_t)pumpRule.manualMin * 60000UL) {
      pumpSet(false, PUMP_OFF_MANUAL_TIME);
    } else if (langleyData.batteryVoltage > 0 && langleyData.batteryVoltage < pumpRule.manualMinV) {
      pumpSet(false, PUMP_OFF_MANUAL_LOW_V);
    }
  }

  // No flow: only counted while the relay can actually close (SW1 closed, or U1 not fitted to say)
  if (pumpRelayOn) {
    bool relayClosed = !foundShieldConfigADC || emergencyClosed;
    if (!relayClosed || flowRateLpm > 0) {
      flowZeroSinceMs = millis();
    } else if (millis() - flowZeroSinceMs >= (uint32_t)pumpRule.dryRunSec * 1000UL) {
      pumpDryRunFault = true;
      pumpSet(false, PUMP_OFF_DRY_RUN);
      updateStatusLed();
    }
  }
}


// ---- Web page (GloriaWifiManager /GloriaServlet) - these run on the web server task ----

// The authenticator code, checked at this unit's clock and one period either side, with
// generateCodeAt() - it does not touch the code history loop() uses (checkCode() does).
bool webCodeOk(long code) {
  if (code <= 0) return false;
  long t = timeManager.getTimeForCodeGeneration(currentTimerRecord);
  for (int d = -30; d <= 30; d += 30)
    if (secretManager.generateCodeAt(t + d) == code) return true;
  return false;
}

String pumpWebSetRule(const String &json, long code) {
  if (!webCodeOk(code)) return "The authenticator code is not valid";
  PumpRule r;
  String e = pumpRuleParse(json, r);
  if (e.length()) return e;
  portENTER_CRITICAL(&pumpRuleMux);
  pumpRulePending = r;
  pumpRulePendingSet = true;
  portEXIT_CRITICAL(&pumpRuleMux);
  return "";
}

String pumpWebClearFault(long code) {
  if (!webCodeOk(code)) return "The authenticator code is not valid";
  pumpFaultClearPending = true;
  return "";
}

// Everything the rule editor needs: status, the rule, the device types and their variables, and the
// devices heard with their values now.
void fillPumpData(JsonDocument &json) {
  JsonObject st = json["status"].to<JsonObject>();
  st["auto"] = autoMode();
  st["pump"] = pumpRelayOn;
  st["reason"] = pumpReasonText(pumpReason);
  st["onSec"] = pumpRelayOn ? (millis() - pumpOnSinceMs) / 1000 : 0;
  st["flow"] = flowRateLpm;
  st["total"] = flowTotalLitres;
  st["stale"] = pumpDataStale;
  st["fault"] = pumpDryRunFault;
  st["sw1"] = foundShieldConfigADC ? (emergencyClosed ? "closed" : "open") : "n/a";
  st["applying"] = (bool)pumpRulePendingSet;

  portENTER_CRITICAL(&pumpRuleMux);
  PumpRule r = pumpRulePendingSet ? pumpRulePending : pumpRule;
  portEXIT_CRITICAL(&pumpRuleMux);
  JsonDocument ruleDoc;
  deserializeJson(ruleDoc, pumpRuleToJson(r));
  json["rule"] = ruleDoc;

  JsonArray types = json["types"].to<JsonArray>();
  for (uint8_t t = 0; t < PUMP_RULE_TYPE_COUNT; t++) {
    if (t == PUMP_RULE_TYPE_LANGLEY) continue;  // no TOTP code in LangleyData: never heard
    JsonObject to = types.add<JsonObject>();
    to["id"] = PUMP_RULE_TYPES[t].id;
    to["name"] = PUMP_RULE_TYPES[t].name;
    JsonArray fields = to["fields"].to<JsonArray>();
    for (uint8_t f = 0; f < PUMP_RULE_FIELD_COUNT; f++) {
      if (PUMP_RULE_FIELDS[f].type != t) continue;
      JsonObject fo = fields.add<JsonObject>();
      fo["id"] = PUMP_RULE_FIELDS[f].id;
      fo["label"] = PUMP_RULE_FIELDS[f].label;
      fo["units"] = PUMP_RULE_FIELDS[f].units;
    }
  }

  JsonArray devices = json["devices"].to<JsonArray>();
  for (int i = 0; i < HEARD_MAX_DEVICES; i++) {
    HeardDevice h;
    portENTER_CRITICAL(&heardMux);
    h = heardDevices[i];
    portEXIT_CRITICAL(&heardMux);
    if (!h.used) continue;
    JsonObject d = devices.add<JsonObject>();
    d["t"] = PUMP_RULE_TYPES[h.type].id;
    d["name"] = h.name;
    d["age"] = (millis() - h.atMs) / 1000;
    d["rssi"] = h.rssi;
    d["stale"] = millis() - h.atMs > (uint32_t)r.staleSec * 1000UL;
    JsonObject values = d["values"].to<JsonObject>();
    for (uint8_t f = 0; f < PUMP_RULE_FIELD_COUNT; f++) {
      if (PUMP_RULE_FIELDS[f].type != h.type) continue;
      float v;
      if (heardFieldValue(f, h.raw, v)) values[PUMP_RULE_FIELDS[f].id] = v;
      else values[PUMP_RULE_FIELDS[f].id] = nullptr;
    }
  }
}

// loop(): what the page asked for
void pumpApplyWebChanges() {
  if (pumpRulePendingSet) {
    PumpRule r;
    portENTER_CRITICAL(&pumpRuleMux);
    r = pumpRulePending;
    pumpRulePendingSet = false;
    portEXIT_CRITICAL(&pumpRuleMux);
    pumpRule = r;
    gloriaPrefs.begin("gloria", false);
    gloriaPrefs.putString("rule", pumpRuleToJson(pumpRule));
    gloriaPrefs.end();
    if (debug) Serial.println("Pump rule set from the web page: " + pumpRuleToJson(pumpRule));
  }
  if (pumpFaultClearPending) {
    pumpFaultClearPending = false;
    pumpDryRunFault = false;
    pumpSet(false, PUMP_IDLE);
    if (debug) Serial.println("No-flow fault cleared from the web page");
  }
}

void printHeardDevices() {
  bool any = false;
  for (int i = 0; i < HEARD_MAX_DEVICES; i++) {
    const HeardDevice &h = heardDevices[i];
    if (!h.used) continue;
    any = true;
    Serial.printf("%s \"%s\"  heard %lu s ago  %d dBm\n", PUMP_RULE_TYPES[h.type].id, h.name, (unsigned long)((millis() - h.atMs) / 1000),
                  (int)h.rssi);
    for (uint8_t f = 0; f < PUMP_RULE_FIELD_COUNT; f++) {
      if (PUMP_RULE_FIELDS[f].type != h.type) continue;
      float v;
      if (heardFieldValue(f, h.raw, v)) Serial.printf("    %-12s %-22s %.2f %s\n", PUMP_RULE_FIELDS[f].id, PUMP_RULE_FIELDS[f].label, v, PUMP_RULE_FIELDS[f].units);
      else Serial.printf("    %-12s %-22s (no reading)\n", PUMP_RULE_FIELDS[f].id, PUMP_RULE_FIELDS[f].label);
    }
  }
  if (!any) Serial.println("No devices heard yet");
}

void printPumpStatus() {
  Serial.printf("mode %s  pump %s (%s)  flow %.2f L/min  total %.1f L  SW1 %s%s%s\n", autoMode() ? "AUTO" : "MANUAL",
                pumpRelayOn ? "ON" : "off", pumpReasonText(pumpReason), flowRateLpm, flowTotalLitres,
                foundShieldConfigADC ? (emergencyClosed ? "closed" : "OPEN") : "n/a", pumpDataStale ? "  DATA STALE" : "",
                pumpDryRunFault ? "  NO-FLOW FAULT" : "");
  const char *names[2] = { "start (all)", "stop (any)" };
  for (uint8_t list = 0; list < 2; list++) {
    const PumpCondition *cs = list == 0 ? pumpRule.start : pumpRule.stop;
    uint8_t n = list == 0 ? pumpRule.startCount : pumpRule.stopCount;
    Serial.printf("  %s:%s\n", names[list], n ? "" : " none");
    for (uint8_t i = 0; i < n; i++) {
      float v = NAN;
      uint8_t r = pumpConditionEval(cs[i], v);
      Serial.printf("    %s > %s (%s) %s %.2f   now %s\n", cs[i].device, PUMP_RULE_FIELDS[cs[i].field].label, PUMP_RULE_FIELDS[cs[i].field].units,
                    PUMP_OP_TEXT[cs[i].op], cs[i].value,
                    r == COND_NO_DATA ? "no data" : (String(v, 2) + (r == COND_TRUE ? " (holds)" : " (does not hold)")).c_str());
    }
  }
}

// Classifies langleyData.operatingStatus from battery voltage and forecast/measured solar
// conditions, same taxonomy/thresholds as Daffodil.ino. Unlike Daffodil there's no lux sensor on
// this hardware, so CLOUDY is decided from whichever of two independent signals fires: the
// LoRa-relayed forecast, or the solar-input INA219's own current reading (solarCurrentCloudyThreshold_mA)
// - the latter doesn't need a forecast to exist or be fresh, same role as Daffodil's
// panelCurrentSaysCloudy. If neither fires we assume FULL_MODE rather than guess.
//
// SLEEP/COMMA/CLOUDY/NO_LED all trigger real ESP32 deep sleep between cycles (see loop()'s
// sendTimer block and goToSleep()) - only FULL_MODE stays continuously awake. The pump relay is
// off while asleep (see goToSleep()); the pump logic will have to decide how that fits.
void updateOperatingMode() {
  // currentTimerRecord is refreshed once per second at the top of loop()'s clockTicked
  // block (same as Daffodil.ino) - reused here rather than re-querying the RTC over I2C again.
  if (solarInfo != nullptr) hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);

  // Disarm the overnight-mAh tracker (see goToSleep()) the moment daytime efficiency actually
  // resumes, regardless of operating mode. Doing this only inside goToSleep() (as before) meant it
  // never fired during a real day: FULL_MODE never calls goToSleep(), so efficiency crossing back
  // above 0.3 at dawn was never observed by the one piece of code that cleared the flag - the
  // accumulator then kept climbing across every subsequent night instead of restarting each sunset.
  // This runs every loop iteration in every mode, so the disarm actually happens at dawn.
  if (hourlySolarPowerData.efficiency > 0.3) {
    overnightTrackingActive = false;
  }

  float batteryVoltage = langleyData.batteryVoltage;

  if (usingSolarPower && batteryVoltage > 0 && batteryPowerManager != nullptr && batteryVoltage < batteryPowerManager->commaVoltage()) {
    langleyData.operatingStatus = OPERATING_STATUS_COMMA;
    return;
  }
  if (usingSolarPower && batteryVoltage > 0 && batteryPowerManager != nullptr && batteryVoltage < batteryPowerManager->sleepingVoltage()) {
    langleyData.operatingStatus = OPERATING_STATUS_SLEEP;
    return;
  }
  if (!usingSolarPower) {
    langleyData.operatingStatus = OPERATING_STATUS_FULL_MODE;
    return;
  }

  // A real measured voltage on the solar input beats the astronomical/forecast-based efficiency
  // estimate below - that estimate only knows what time of day it thinks it is, not whether
  // there's actually a DC source connected right now (sun on the panel, or a bench/laptop-brick
  // supply). If there's real voltage present, there's no reason to duty-cycle sleep at all.
  if (foundSolarMonitor && langleyData.solarVoltage > SOLAR_INPUT_PRESENT_VOLTS) {
    langleyData.operatingStatus = OPERATING_STATUS_FULL_MODE;
    return;
  }

  if (hourlySolarPowerData.efficiency * 100 < minimumEfficiencyForLed) {
    langleyData.operatingStatus = OPERATING_STATUS_NO_LED;
    return;
  }

  bool forecastSaysCloudy = weatherIsFresh() && (weatherForecasts[0].cloudiness >= cloudyThreshold);
  // Direct harvest measurement, forecast-independent - see solarCurrentCloudyThreshold_mA above for
  // why this exists. Range check (not just "< threshold") because the 2mOhm shunt's near-zero
  // readings jitter a few mA either side of 0 from ADC noise.
  bool currentSaysCloudy = foundSolarMonitor
                          && langleyData.solarCurrentMa > -solarCurrentCloudyThreshold_mA
                          && langleyData.solarCurrentMa < solarCurrentCloudyThreshold_mA;
  langleyData.operatingStatus = (forecastSaysCloudy || currentSaysCloudy) ? OPERATING_STATUS_CLOUDY : OPERATING_STATUS_FULL_MODE;
}

// ============================================================
// WiFi / web page - same station/access-point behaviour as Daffodil (see LangleyWifiManager)
//
// Only runs while the unit is continuously awake in FULL_MODE, which already means the battery is
// above its sleep threshold (see updateOperatingMode()) - every other mode duty-cycle deep-sleeps,
// which would drop WiFi anyway. Also needs to have held for WIFI_START_DELAY_SECONDS so a flicker
// between modes doesn't repeatedly pay for a WiFi start, which blocks loop() (network scan +
// connect) for tens of seconds: no LoRa sends and no fence pulses are sampled meanwhile.
//
// WiFi on demand (DeviceCommandRecord over LoRa, see acceptDeviceCommand()) overrides both: it
// starts straight away in any mode and keeps the unit awake until it ends or the battery drops to
// the floor. An explicit "off" also holds the automatic start back for WIFI_OFF_SUPPRESS_AUTO_SEC.
// ============================================================
#define WIFI_START_DELAY_SECONDS 30
#define INTERNET_RECHECK_SECONDS 600
#define WIFI_MANAGE_PERIOD_SECONDS 2  // = displayTimer's period, manageWifi() runs from that block

long codeTimeNow() {
  return timeManager.getTimeForCodeGeneration(currentTimerRecord);
}

bool wifiOnDemandActive() {
  return wifiOnDemandUntil != 0 && codeTimeNow() < (long)wifiOnDemandUntil;
}

// Active and the battery (under load) still above the floor - see WIFI_ON_DEMAND_FLOOR_MARGIN_V.
bool wifiOnDemandAllowed() {
  if (!wifiOnDemandActive()) return false;
  if (!foundBatteryMonitor || batteryPowerManager == nullptr || langleyData.batteryVoltage <= 0) return true;  // not read yet
  return langleyData.batteryVoltage >= batteryPowerManager->sleepingVoltage() + WIFI_ON_DEMAND_FLOOR_MARGIN_V;
}

// The forecast relayed by Annabelle is under WEATHER_FRESH_SECONDS old (RTC time, so it survives
// deep sleep - see lastWeatherForecastSeconds).
bool weatherIsFresh() {
  if (!haveWeatherForecast) return false;
  long age = timeManager.getCurrentTimeInSeconds(currentTimerRecord) - lastWeatherForecastSeconds;
  return age >= 0 && age < WEATHER_FRESH_SECONDS;
}

bool wifiAllowed() {
  if (wifiOnDemandAllowed()) return true;
  if (codeTimeNow() < (long)wifiAutoSuppressedUntil) return false;  // not right after an explicit "off"
  if (langleyData.operatingStatus != OPERATING_STATUS_FULL_MODE) return false;
  // Real measured input voltage (bench supply / adapter) means power is available regardless of
  // what the astronomical estimate thinks - same reasoning as updateOperatingMode().
  if (foundSolarMonitor && langleyData.solarVoltage > SOLAR_INPUT_PRESENT_VOLTS) return true;
  return hourlySolarPowerData.efficiency * 100 >= minimumEfficiencyForWifi;
}

// WiFi's scan + connect can outlast the TPL5010's window, and nothing pets it while loop() is
// blocked - so a helper task does it until startWifi() is done.
void wifiStartWatchdogTask(void *) {
  while (petWatchdogDuringWifiStart) {
    digitalWrite(WATCHDOG_DONE, HIGH);
    delayMicroseconds(100);
    digitalWrite(WATCHDOG_DONE, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  vTaskDelete(NULL);
}

// Persists GPS saved from the web page. The sketch owns the full device-config record (name,
// timezone, altitude, thresholds), so it does the save. SolarInfo was built from the old
// coordinates at boot and only picks up the new ones on the next restart.
void onGpsSaved(float lat, float lon) {
  latitude = lat;
  longitude = lon;
  secretManager.saveDeviceSensorConfig(String(langleyData.devicename), String(langleyData.deviceshortname),
                                       String(sensor1name), String(sensor2name), timezone,
                                       latitude, longitude, altitude,
                                       minimumEfficiencyForLed, minimumEfficiencyForWifi);
}

void startWifi() {
  if (debug) Serial.println("Starting WiFi");
  petWatchdogDuringWifiStart = true;
  xTaskCreatePinnedToCore(wifiStartWatchdogTask, "wifiWdt", 2048, NULL, 1, NULL, 0);

  if (!wwwMounted) {
    wwwMounted = wwwFS.begin(false, "/www", 10, "www");
    if (!wwwMounted) Serial.println("www partition mount FAILED - web page unavailable until the www image is flashed");
  }

  String sn = "";
  sn = DeviceSerial::toString(langleyData.serialnumberarray);  // the factory's form (2026-10-30)
  wifiManager.setSerialNumber(sn);
  wifiManager.setLora(loraActive);

  if (!initiatedWifi) {
    wifiManager.setGpsSavedCallback(onGpsSaved);
    wifiManager.setPowerDataProvider(fillPowerData);
    wifiManager.setRelayHandlers(relayStatusJson, relayWebSet);
    wifiManager.pumpDataProvider = fillPumpData;
    wifiManager.pumpRuleSetter = pumpWebSetRule;
    wifiManager.pumpFaultClearer = pumpWebClearFault;
    wifiManager.start();
    initiatedWifi = true;
  }
  // start() only connects; restartWifi() is what falls back to access-point mode when a station
  // connection fails - and it's the only way back up after stopWifi().
  if (!wifiManager.getWifiStatus()) wifiManager.restartWifi();

  ipAddress = wifiManager.getAPStatus() ? wifiManager.getApAddress() : wifiManager.getIpAddress();
  if (wifiManager.getWifiStatus() && !wifiManager.getAPStatus()) {
    wifiManager.checkInternetConnectionAvailable();
    lastInternetCheckMs = millis();
  }

  petWatchdogDuringWifiStart = false;
  wifiRunning = true;
  if (debug) {
    Serial.print("WiFi up, address=");
    Serial.println(ipAddress);
  }
}

void stopWifi() {
  if (debug) Serial.println("Stopping WiFi");
  wifiManager.stop();
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(true);
  wifiRunning = false;
  wifiWantedSeconds = 0;
  ipAddress = "";
}

// What the data packet reports as wifiStatus - same values as Daffodil's, which Annabelle's page
// shows next to the WiFi on/off button.
uint8_t wifiStatusCode() {
  if (!wifiRunning) return 0;
  if (wifiManager.getAPStatus()) return 1;
  if (!wifiManager.getWifiStatus()) return 0;
  return wifiManager.getInternetAvailable() ? 3 : 2;
}

void manageWifi() {
  if (wifiOnDemandActive() && !wifiOnDemandAllowed()) wifiOnDemandUntil = 0;  // battery under the floor: over
  if (wifiOffRequested) {
    wifiOffRequested = false;
    if (wifiRunning) stopWifi();
  }
  if (wifiAllowed()) {
    if (!wifiRunning) {
      wifiWantedSeconds += WIFI_MANAGE_PERIOD_SECONDS;
      // Asked for on purpose: no point making the user wait out the flicker delay
      if (wifiWantedSeconds >= WIFI_START_DELAY_SECONDS || wifiOnDemandAllowed()) startWifi();
    } else if (!wifiManager.getAPStatus() && wifiManager.getWifiStatus()
               && (millis() - lastInternetCheckMs) > (unsigned long)INTERNET_RECHECK_SECONDS * 1000UL) {
      wifiManager.checkInternetConnectionAvailable();
      lastInternetCheckMs = millis();
    }
  } else {
    wifiWantedSeconds = 0;
    if (wifiRunning) stopWifi();
  }
}

// DIO0 interrupt: only set a flag. arduino-LoRa's own LoRa.onReceive() handler (used here before
// 2026-10-23) reads the radio over SPI inside the interrupt, and SPI on ESP32 core 3.x takes a
// mutex, which must never be waited on in an ISR (Annabelle's xQueueSemaphoreTake assert PANIC,
// 2026-10-05). The packet is read at the top of loop() with LoRa.parsePacket() - same as Daffodil.
volatile bool loraDio0Fired = false;

// ---- Relay (2026-11-11) ----
// Langley_W's final spot is far from Annabelle, and from where it is now it may hear Chinampa better
// than the house does. With relay on, it resends the packets it hears that need help (LoRaRelay.h:
// heard weaker than the strong-signal limit, checked out, not already relayed), stamping its
// deviceshortname into sentbyarray - Annabelle shows "via <shortname>". Only while awake: in FULL_MODE
// all the time, in the duty-cycling modes during each awake window. On by default; the setting is
// kept in Preferences ("relay"), so it survives the deep-sleep reboots. Serial: Relay#1 / Relay#0 /
// Relay#1#<dBm> (strong-signal limit, 0 = relay everything) / Relay (status).
#define RELAY_PREFS "relay"
LoRaRelay relay;
// Staying awake for the relay (2026-11-11): in the modes that would sleep (SLEEP / CLOUDY / NO_LED -
// at night, or on a low battery), relay on keeps Langley awake as long as the battery reads at least
// relayMinVolts - otherwise it sleeps as before, so the relay never runs it flat. COMMA always sleeps.
// While it is awake only for the relay, the status LEDs and both 7-segment displays stay dark
// (relayDark). relayMinVolts is set on the web page's R panel (or Relay#...#<volts>), kept in Preferences.
#define RELAY_MIN_VOLTS_DEFAULT 12.4f
#define RELAY_MIN_VOLTS_HYSTERESIS 0.1f  // back above min + this before it stays awake again
float relayMinVolts = RELAY_MIN_VOLTS_DEFAULT;
bool relayDark = false;      // awake only for the relay: LEDs and displays off
bool relayAwakeOk = false;   // battery above the limit (with hysteresis)

// Every LED update goes through here: dark while awake only for the relay.
void ledsShow() {
  if (relayDark) FastLED.clear(true);
  else FastLED.show();
}

// Battery high enough to stay awake for the relay - false when it can't be read (sleep, to be safe).
bool relayCanKeepAwake() {
  if (!relay.on || !foundBatteryMonitor || langleyData.batteryVoltage <= 0) { relayAwakeOk = false; return false; }
  float limit = relayAwakeOk ? relayMinVolts : relayMinVolts + RELAY_MIN_VOLTS_HYSTERESIS;
  relayAwakeOk = langleyData.batteryVoltage >= limit;
  return relayAwakeOk;
}
void LoRa_txMode();
void LoRa_rxMode();

bool relaySend(const uint8_t *bytes, size_t len) {
  LoRa_txMode();
  LoRa.beginPacket();
  LoRa.write(bytes, len);
  bool ok = LoRa.endPacket(false);
  loraDio0Fired = false;  // that was TX done, not a packet
  LoRa_rxMode();
  return ok;
}

bool relayBusy() {
  return LoRa.rssi() > CAD_RSSI_THRESHOLD;  // same test as performCAD()
}

void relayLoadSettings() {
  Preferences prefs;
  prefs.begin(RELAY_PREFS, true);
  relay.setOn(prefs.getBool("on", true));  // on unless turned off (2026-11-11: on by default)
  relay.strongDbm = prefs.getInt("dbm", LORA_RELAY_STRONG_DEFAULT);
  relayMinVolts = prefs.getFloat("minv", RELAY_MIN_VOLTS_DEFAULT);
  prefs.end();
}

// The web page's Relay switch (LangleyWifiManager SetRelay) runs on the web server task: it only
// leaves a request, applied and saved from loop() (relayApplyWebRequest).
volatile bool relayWebPending = false;
volatile bool relayWebOn = true;
volatile int relayWebDbm = 1;  // 1 = keep the limit
volatile float relayWebMinV = NAN;  // NAN = keep it

void relayStatusJson(JsonObject r) {
  r["on"] = relay.on;
  r["strongDbm"] = relay.strongDbm;
  r["minVolts"] = relayMinVolts;
  r["battery"] = langleyData.batteryVoltage;
  r["keepingAwake"] = relayDark;  // awake only for the relay right now (LEDs/displays dark)
  r["heard"] = relay.heard;
  r["sent"] = relay.sent;
  r["strong"] = relay.strong;
  r["dup"] = relay.dup;
  r["dropped"] = relay.dropped;
  if (relay.lastAt) {
    r["last"] = relay.lastName;
    r["lastAction"] = relay.lastAction == LORA_RELAY_STRONG ? "strong" : "relayed";
    r["lastRssi"] = relay.lastRssi;
    r["lastAgeSec"] = (millis() - relay.lastAt) / 1000;
  }
}

void relayWebSet(bool on, int dbm, float minv) {
  relayWebOn = on;
  relayWebDbm = dbm;
  relayWebMinV = minv;
  relayWebPending = true;
}

void relaySaveSettings();
void relayApplyWebRequest() {
  if (!relayWebPending) return;
  relayWebPending = false;
  relay.applyCommand(relayWebOn ? "1" : "0", relayWebDbm == 1 ? String("") : String(relayWebDbm));
  float v = relayWebMinV;
  if (!isnan(v) && v >= 9.0f && v <= 15.0f) relayMinVolts = v;
  relaySaveSettings();
}

void relaySaveSettings() {
  Preferences prefs;
  prefs.begin(RELAY_PREFS, false);
  prefs.putBool("on", relay.on);
  prefs.putInt("dbm", relay.strongDbm);
  prefs.putFloat("minv", relayMinVolts);
  prefs.end();
}

void IRAM_ATTR onLoraDio0() {
  loraDio0Fired = true;
}

// Received packet, copied out of the radio the moment DIO0 is seen so the radio can go straight
// back to listening - see Daffodil.ino's loraRxBuf for why (a unit busy for a few seconds missed
// most of Annabelle's weather/time broadcasts). processLoraPacket() reads from here.
uint8_t loraRxBuf[256];
int loraRxLen = 0;
int loraRxPos = 0;
int loraRxRssi = 0;          // of the packet in loraRxBuf (2026-11-03, for Paula's ping)
float loraRxSnr = 0;
uint32_t loraRxAtMs = 0;     // when it was copied out - the ping answer's slot counts from here
bool loraReceived = false;
int loraPacketSize = 0;

size_t loraRxRead(uint8_t *dst, size_t len) {
  size_t n = 0;
  while (n < len && loraRxPos < loraRxLen) dst[n++] = loraRxBuf[loraRxPos++];
  while (n < len) dst[n++] = 0;
  return len;
}

// Copies a packet out of the radio into loraRxBuf and puts it back in receive. Returns its size,
// 0 for a DIO0 without a good packet (CRC error, TxDone).
int loraCopyPacket() {
  int packetSize = LoRa.parsePacket();
  int n = 0;
  if (packetSize > 0) {
    loraRxRssi = LoRa.packetRssi();
    loraRxSnr = LoRa.packetSnr();
    loraRxAtMs = millis();
    while (LoRa.available() && n < (int)sizeof(loraRxBuf)) loraRxBuf[n++] = (uint8_t)LoRa.read();
  }
  loraRxLen = n;
  loraRxPos = 0;
  LoRa_rxMode();
  if (n > 0) relay.note(loraRxBuf, n, loraRxRssi);  // the bytes as received - queued if it needs a relay
  return n;
}

// ─────────────────────────────────────────────────────────────────────────────────────────────
// Field time sync (Annabelle/Field_Time_Sync_Proposal, 2026-10-07), same as Daffodil.ino.
// Annabelle's WeatherForecastUpdate carries its NTP-corrected time (annabelleTime, in the
// getTimeForCodeGeneration() convention) and a TOTP calculated at that time, so the packet can be
// authenticated whatever this RTC says.
#define TIME_SYNC_TOLERANCE_SEC 5      // correct the RTC when it is further off than this
#define TIME_SYNC_MAX_JUMP_SEC  7200   // larger jumps only after the RTC lost power (replay guard)
#define TIME_SYNC_PREFS         "timesync"
#define TIME_SYNC_LAST_KEY      "lastAnnT"

// The PCF8563 lost power since it was last set: VL bit in the seconds register, or a year that
// can't be right.
bool rtcLostPower() {
  Wire.beginTransmission(0x51);
  Wire.write(0x02);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(0x51, 1);
  if (!Wire.available()) return false;
  bool vl = Wire.read() & 0x80;
  return vl || currentTimerRecord.year < 2025;
}

// Writes a getTimeForCodeGeneration()-style time into the RTC. Writing the seconds register
// also clears the VL bit.
void writeRtcFromCodeTime(uint32_t codeTime) {
  time_t fields = (time_t)codeTime + timeManager.timeZoneHours * 3600L;  // back to local wall-clock fields
  struct tm t;
  gmtime_r(&fields, &t);
  auto bcd = [](int v) -> uint8_t { return (uint8_t)(((v / 10) << 4) | (v % 10)); };
  Wire.beginTransmission(0x51);  // PCF8563
  Wire.write(0x02);
  Wire.write(bcd(t.tm_sec));
  Wire.write(bcd(t.tm_min));
  Wire.write(bcd(t.tm_hour));
  Wire.write(bcd(t.tm_mday));
  Wire.write(bcd(t.tm_wday));
  Wire.write(bcd(t.tm_mon + 1));
  Wire.write(bcd(t.tm_year - 100));
  Wire.endTransmission();
}

// Checks a WeatherForecastUpdate and, if it is genuine, sets the RTC from it. Returns whether the
// packet is accepted (forecast included):
//  - the code must match annabelleTime (proves it knows the shared secret)
//  - annabelleTime must be later than the last accepted one (kept in NVS) - no replays
//  - and within TIME_SYNC_MAX_JUMP_SEC of this clock, unless the RTC lost power
bool acceptAnnabelleTime(const WeatherForecastUpdate &u) {
  if (u.annabelleTime == 0 || secretManager.generateCodeAt(u.annabelleTime) != u.totpcode) {
    if (debug) Serial.println(" time sync: code does not match annabelleTime");
    return false;
  }
  Preferences prefs;
  prefs.begin(TIME_SYNC_PREFS, false);
  uint32_t last = prefs.getUInt(TIME_SYNC_LAST_KEY, 0);
  if (u.annabelleTime <= last) {
    prefs.end();
    if (debug) Serial.printf(" time sync: annabelleTime %u not after last accepted %u (replay?)\n", u.annabelleTime, last);
    return false;
  }
  long deviceTime = codeTimeNow();
  long diff = (long)u.annabelleTime - deviceTime;
  if (labs(diff) > TIME_SYNC_MAX_JUMP_SEC && !rtcLostPower()) {
    prefs.end();
    if (debug) Serial.printf(" time sync: clock %ld s off, more than %d s - not corrected (set it by hand)\n", diff, TIME_SYNC_MAX_JUMP_SEC);
    return false;
  }
  prefs.putUInt(TIME_SYNC_LAST_KEY, u.annabelleTime);
  prefs.end();
  if (labs(diff) > TIME_SYNC_TOLERANCE_SEC) {
    writeRtcFromCodeTime(u.annabelleTime);
    currentTimerRecord = timeManager.now();  // so the weather timestamp below is on the corrected clock
    if (debug) Serial.printf(" time sync: RTC corrected by %ld s (Annabelle %s)\n", diff, (u.flags & WEATHER_FLAG_DST) ? "DST" : "standard time");
  }
  return true;
}

// Checks a DeviceCommandRecord and carries it out. Accepted only when:
//  - the checksum matches and it is addressed to this unit's serial
//  - the code matches senderTime (proves the sender knows the shared secret)
//  - senderTime is within DEVICE_COMMAND_MAX_CLOCK_DIFF_SEC of this clock
//  - senderTime is later than the last command accepted from that sender (kept in NVS) - no replays
bool acceptDeviceCommand(const DeviceCommandRecord &c) {
  if (deviceCommandChecksum(c) != c.checksum) return false;
  if (memcmp(c.serialnumberarray, langleyData.serialnumberarray, 8) != 0) return false;  // another unit's
  if (c.senderTime == 0 || secretManager.generateCodeAt(c.senderTime) != c.totpcode) {
    if (debug) Serial.println(" command: code does not match senderTime");
    return false;
  }
  long now = codeTimeNow();
  if (labs((long)c.senderTime - now) > DEVICE_COMMAND_MAX_CLOCK_DIFF_SEC) {
    if (debug) Serial.printf(" command: sender clock %ld s off\n", (long)c.senderTime - now);
    return false;
  }
  char key[12];
  snprintf(key, sizeof(key), "lastCmd%u", c.sender);
  Preferences prefs;
  prefs.begin(TIME_SYNC_PREFS, false);
  uint32_t last = prefs.getUInt(key, 0);
  if (c.senderTime <= last) {
    prefs.end();
    if (debug) Serial.printf(" command: senderTime %u not after last accepted %u (replay?)\n", c.senderTime, last);
    return false;
  }
  prefs.putUInt(key, c.senderTime);
  prefs.end();

  if (c.command == DEVICE_COMMAND_WIFI_ON) {
    int minutes = c.minutes ? min((int)c.minutes, WIFI_ON_DEMAND_MAX_MIN) : WIFI_ON_DEMAND_DEFAULT_MIN;
    wifiOnDemandUntil = (uint32_t)(now + minutes * 60L);
    wifiAutoSuppressedUntil = 0;
    if (debug) Serial.printf(" command: WiFi on for %d min\n", minutes);
  } else if (c.command == DEVICE_COMMAND_WIFI_OFF) {
    wifiOnDemandUntil = 0;
    wifiAutoSuppressedUntil = (uint32_t)(now + WIFI_OFF_SUPPRESS_AUTO_SEC);
    wifiOffRequested = true;
    if (debug) Serial.println(" command: WiFi off");
  } else {
    return false;
  }
  return true;
}

// Handles a packet copied into loraRxBuf: a weather forecast (which also corrects the clock) or a
// device command. Langley never downloads forecasts itself (its WiFi is a local web page only), so
// Annabelle's broadcast or reply is its only source of forecast data. Called from loop() and
// listenForReplies(), never from the DIO0 interrupt.
void processLoraPacket(int packetSize) {
  if (packetSize == 0) return;

  if (packetSize == sizeof(WeatherForecastUpdate)) {
    WeatherForecastUpdate update;
    loraRxRead((uint8_t *)&update, sizeof(update));
    if (!acceptAnnabelleTime(update)) {  // also sets the RTC from Annabelle's time
      if (debug) Serial.println("Weather forecast rejected");
      return;
    }
    memcpy(weatherForecasts, update.forecasts, sizeof(weatherForecasts));
    haveWeatherForecast = true;
    rtc_weatherRssi = (int16_t)loraRxRssi;
    rtc_weatherSnrQ = (int8_t)lroundf(loraRxSnr * 4);
    lastWeatherForecastSeconds = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
    if (solarInfo != nullptr) solarInfo->setWeatherForecast(weatherForecasts, 4);
    if (debug) Serial.println("Weather forecast updated via LoRa");
  } else if (packetSize == sizeof(DeviceCommandRecord)) {
    DeviceCommandRecord command;
    loraRxRead((uint8_t *)&command, sizeof(command));
    if (isBroadcastPing(command.serialnumberarray, command.command)) {
      handlePing(command);  // Paula's signal survey - see handlePing
      return;
    }
    bool accepted = acceptDeviceCommand(command);
    if (debug) Serial.printf("DeviceCommand %u (%u min) from sender %u: %s\n", command.command,
                             command.minutes, command.sender, accepted ? "accepted" : "ignored");
  } else if (packetSize == sizeof(DigitalStablesData)) {
    // A Daffodil (e.g. Pool), for the pump rule. The pump acts on this data, so it must carry this farm's
    // TOTP code (same check as Daffodil.ino does on DigitalStablesData) and a good checksum.
    DigitalStablesData d;
    loraRxRead((uint8_t *)&d, sizeof(d));
    bool codeOk = secretManager.checkCode(d.totpcode, timeManager.getTimeForCodeGeneration(currentTimerRecord));
    if (codeOk && d.checksum == calculateChecksum(d)) heardRecord(PUMP_RULE_TYPE_DAFFODIL, d.devicename, sizeof(d.devicename), &d, sizeof(d));
    else if (debug) Serial.printf("DigitalStablesData from %.12s ignored: %s\n", d.devicename, codeOk ? "bad checksum" : "TOTP code not valid");
  } else if (packetSize == sizeof(ChinampaData)) {
    // Needs a valid TOTP code too. Chinampa did not fill totpcode as of 2026-11-06 (Daffodil.ino), so
    // its packets are ignored until it does.
    ChinampaData c;
    loraRxRead((uint8_t *)&c, sizeof(c));
    if (secretManager.checkCode(c.totpcode, timeManager.getTimeForCodeGeneration(currentTimerRecord)))
      heardRecord(PUMP_RULE_TYPE_CHINAMPA, c.devicename, sizeof(c.devicename), &c, sizeof(c));
    else if (debug) Serial.printf("ChinampaData from %.16s ignored: TOTP code not valid\n", c.devicename);
  }
  // LangleyData has no TOTP code, so Langleys (and other Glorias) cannot be used in the pump rule.
}

// ---- Signal survey ping (2026-11-03), same as Daffodil.ino ----
// Paula's Nearby Devices map pings every device at once (DEVICE_COMMAND_PING): "who hears me from
// here?". Langley answers with a LinkAckRecord - how strongly it heard the ping, how well it hears
// Annabelle (its last forecast), and where it is - in its own slot (linkAckDelayMs, from the serial),
// counted from when the ping arrived (loraRxAtMs), since loop() only handles packets on its 1 s
// tick. Checked like a command (checksum, code, clock); the replay check is in RAM - nothing changes.
uint32_t lastPingSenderTime = 0;
bool linkAckPending = false;
uint32_t linkAckDueMs = 0;
LinkAckRecord pendingLinkAck;

void handlePing(const DeviceCommandRecord &c) {
  if (deviceCommandChecksum(c) != c.checksum) return;
  if (c.senderTime == 0 || secretManager.generateCodeAt(c.senderTime) != c.totpcode) {
    if (debug) Serial.println("ping: code does not match senderTime");
    return;
  }
  long now = codeTimeNow();
  if (labs((long)c.senderTime - now) > DEVICE_COMMAND_MAX_CLOCK_DIFF_SEC) {
    if (debug) Serial.printf("ping: sender clock %ld s off\n", (long)c.senderTime - now);
    return;
  }
  if (c.senderTime <= lastPingSenderTime) return;  // the same ping again
  lastPingSenderTime = c.senderTime;
  LinkAckRecord a;
  a.probeTime = c.senderTime;
  a.probeSeq = (uint16_t)(c.reserved[0] | (c.reserved[1] << 8));
  memcpy(a.serialnumberarray, langleyData.serialnumberarray, 8);
  strncpy(a.devicename, langleyData.devicename, sizeof(a.devicename) - 1);
  a.kind = LINK_ACK_KIND_LANGLEY;
  a.probeRssi = (int16_t)loraRxRssi;
  a.probeSnrQ = (int8_t)lroundf(loraRxSnr * 4);
  if (rtc_weatherRssi != 0 && haveWeatherForecast) {
    long age = timeManager.getCurrentTimeInSeconds(currentTimerRecord) - lastWeatherForecastSeconds;
    a.gatewayRssi = rtc_weatherRssi;
    a.gatewaySnrQ = rtc_weatherSnrQ;
    a.gatewayAgeSec = (uint16_t)constrain(age, 0L, 65534L);
  }
  a.latitude = langleyData.latitude;
  a.longitude = langleyData.longitude;
  linkAckSeal(a);
  pendingLinkAck = a;
  linkAckDueMs = loraRxAtMs + linkAckDelayMs(a.serialnumberarray, false);
  linkAckPending = true;
  if (debug) Serial.printf("ping %u #%u heard at %d dBm - answering\n", c.senderTime, a.probeSeq, loraRxRssi);
}

// The answer, with CAD + back-off like sendLangleyData() (which sends only LangleyData).
void sendLinkAck(const LinkAckRecord &a) {
  if (!loraActive) return;
  LoRa_txMode();
  for (int retries = 0; retries < MAX_RETRIES; retries++) {
    if (performCAD() == LORA_OK) {
      LoRa.beginPacket();
      LoRa.write((const uint8_t *)&a, sizeof(a));
      LoRa.endPacket(false);
      break;
    }
    delay(random(MIN_BACKOFF * (1 << retries), MAX_BACKOFF * (1 << retries)));
  }
  LoRa_rxMode();
}

// Sends the answer once its slot has come. wait: block until then - used before sleeping.
void sendPendingLinkAck(bool wait) {
  if (!linkAckPending) return;
  if (wait) {
    while ((int32_t)(millis() - linkAckDueMs) < 0) delay(5);
  } else if ((int32_t)(millis() - linkAckDueMs) < 0) {
    return;
  }
  linkAckPending = false;
  sendLinkAck(pendingLinkAck);
}

// Weather and commands on request, same as Daffodil.ino's listenForReplies(). A sleeping unit is
// awake only a short window per wake and almost never hears Annabelle's 5-minute broadcast, so:
//  - when the forecast is stale, the pre-sleep data packet says so (opMode bit 1 clear) and
//    Annabelle answers right after this unit's VitalSigns - listen WEATHER_REPLY_LISTEN_MS for it
//  - Annabelle and Paula hold a DeviceCommandRecord for a sleeping unit and send it at that same
//    moment, before any weather reply - with a fresh forecast listen only COMMAND_LISTEN_MS for it
// Called from goToSleep() right after sendVitalSigns().
void listenForReplies() {
  if (!loraActive) return;
  bool weatherStale = !weatherIsFresh();
  bool commandWindow = LISTEN_FOR_COMMANDS_AT_NIGHT || hourlySolarPowerData.efficiency > 0;
  if (!weatherStale && !commandWindow) return;
  unsigned long window = weatherStale ? WEATHER_REPLY_LISTEN_MS : COMMAND_LISTEN_MS;
  bool commandHeard = false;
  LoRa_rxMode();
  unsigned long start = millis();
  while (millis() - start < window) {
    if (loraDio0Fired) {
      loraDio0Fired = false;
      int n = loraCopyPacket();
      if (n == sizeof(WeatherForecastUpdate) || n == sizeof(DeviceCommandRecord)) {
        processLoraPacket(n);
        if (n == sizeof(DeviceCommandRecord)) commandHeard = true;
        // Stale: wait for the forecast (a command comes before it). Fresh: only a command was due.
        if (weatherStale ? weatherIsFresh() : commandHeard) break;
      }
    }
    delay(5);
  }
  if (debug) Serial.printf("listened %lu ms: weather %s, command %s\n", millis() - start,
                           !weatherStale ? "fresh" : weatherIsFresh() ? "accepted" : "not received",
                           commandHeard ? "heard" : "none");
}

// XOR checksum over every byte of T except the "checksum" member itself - same approach as
// Daffodil.ino's calculateChecksum<T>(), simplified since Langley only ever sends one struct type.
template<typename T>
uint8_t calculateChecksum(const T &data) {
  uint8_t checksum = 0;
  const uint8_t *dataPtr = (const uint8_t *)&data;
  size_t checksumOffset = offsetof(T, checksum);
  for (size_t i = 0; i < checksumOffset; i++) checksum ^= dataPtr[i];
  for (size_t i = checksumOffset + sizeof(uint8_t); i < sizeof(T); i++) checksum ^= dataPtr[i];
  return checksum;
}

void LoRa_txMode() {
  LoRa.idle();             // set standby mode
  LoRa.disableInvertIQ();  // normal mode
}

void LoRa_rxMode() {
  LoRa.disableInvertIQ();  // normal mode
  LoRa.receive();          // set receive mode
}

// Channel-Activity-Detect: samples RSSI before transmitting so two field units don't talk
// over each other. Same approach as Daffodil.ino's performCAD().
LoRaError performCAD() {
  if (!loraActive) return LORA_INIT_FAILED;

  LoRa.idle();
  LoRa.receive();

  const int SAMPLES = 4;
  float rssiSum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    rssiSum += LoRa.rssi();
    delayMicroseconds(500);
  }
  avgRssi = rssiSum / SAMPLES;

  if (avgRssi > CAD_RSSI_THRESHOLD) {
    if (debug) {
      Serial.print("Channel Busy! RSSI: ");
      Serial.println(avgRssi);
    }
    LoRa.idle();
    return LORA_CHANNEL_BUSY;
  }

  if (debug) {
    Serial.print("Channel Clear. RSSI: ");
    Serial.println(avgRssi);
  }
  errorManager.clearLoRaError(LORA_CHANNEL_BUSY);
  return LORA_OK;
}

// OLED, laid out like Annabelle's pages: centred header with the device name and RTC time, then
// size-1 lines (21 characters). Replaces Langley's two TM1637 displays; same values, one page.
void centerText(String text, int y) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, y);
  display.print(text);
}

String titleWithTime(String name) {
  char t[6];
  snprintf(t, sizeof(t), "%02d:%02d", currentTimerRecord.hour, currentTimerRecord.minute);
  if (name.length() > 15) name = name.substring(0, 15);
  return name + " " + t;
}

const char *operatingStatusText(float status) {
  switch ((int)status) {
    case OPERATING_STATUS_SLEEP: return "SLEEP";
    case OPERATING_STATUS_NO_LED: return "NO_LED";
    case OPERATING_STATUS_FULL_MODE: return "FULL";
    case OPERATING_STATUS_CLOUDY: return "CLOUDY";
    case OPERATING_STATUS_COMMA: return "COMMA";
    default: return "?";
  }
}

// Start-up screen like Annabelle's: the name large, then the build time, held 2 s
void showSplash() {
  if (!foundOled) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("Gloria", 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 10);
  display.print("Gloria");
  centerText(__DATE__ " " __TIME__, 50);
  display.display();
  delay(2000);
}

void updateDisplays() {
  if (relayDark || !foundOled || !shieldPowered) return;  // awake only for the relay: screen stays dark
  char line[22];
  display.clearDisplay();
  centerText(titleWithTime(langleyData.devicename[0] ? String(langleyData.devicename) : String("Gloria")), 0);
  display.setTextSize(1);
  display.setCursor(0, 10);
  snprintf(line, sizeof(line), "Bat %5.2fV %5dmA", langleyData.batteryVoltage, (int)langleyData.batteryCurrentMa);
  display.println(line);
  snprintf(line, sizeof(line), "Sun %5.2fV %5dmA", langleyData.solarVoltage, (int)langleyData.solarCurrentMa);
  display.println(line);
  snprintf(line, sizeof(line), "%-6s Pump %s", autoMode() ? "AUTO" : "MANUAL", pumpRelayOn ? "ON" : "off");
  display.println(line);
  if (pumpRelayOn) {
    uint32_t sec = (millis() - pumpOnSinceMs) / 1000;
    snprintf(line, sizeof(line), " %s %lum%02lus", pumpReasonText(pumpReason), (unsigned long)(sec / 60), (unsigned long)(sec % 60));
  } else {
    snprintf(line, sizeof(line), " %s", pumpReasonText(pumpReason));
  }
  display.println(line);
  snprintf(line, sizeof(line), "Flow %5.1fL/m %6.0fL", flowRateLpm, flowTotalLitres);
  display.println(line);
  snprintf(line, sizeof(line), "%-6s LoRa %-2s SW1 %s", operatingStatusText(langleyData.operatingStatus), loraActive ? "ok" : "--",
           foundShieldConfigADC ? (emergencyClosed ? "ok" : "X") : "-");
  display.println(line);
  display.display();
}

// LED 3 - RTC backup coin cell (CR1220, nominal 3.0V):
//   Green  | rtcBatVolt >= 2.7V
//   Orange | 2.4V <= rtcBatVolt < 2.7V
//   Red    | rtcBatVolt < 2.4V, or < 0 (read error / floating pin, see readSensorData())
//
// LED 4 - main system battery, same thresholds driving updateOperatingMode()'s COMMA/SLEEP
// decisions (BatteryPowerManager::commaVoltage()/sleepingVoltage(), currently 11.6V/12.0V for
// both chemistries) - so this LED always agrees with what the unit is actually about to do:
//   Green  | batteryVoltage >= sleepingVoltage()
//   Orange | commaVoltage() <= batteryVoltage < sleepingVoltage()
//   Red    | batteryVoltage < commaVoltage()
// Gloria's LEDs (Ari, 2026-11-18):
//   0 WiFi: green = own access point, blue = router with internet, orange = router without internet
//   1 LoRa: green = active, blue = relay mode on, red = LoRa failed
//   2 Annabelle: green = receiving Annabelle (a forecast under 31 min old), red = not
//   3 RTC coin cell: green >= 2.7 V, orange >= 2.4 V, red below (or no reading)
//   4 flow: blue while the flow meter counts
//   5 rain: off until the RS485 rain sensor is fitted
//   6 off
//   7 pump mode: green = AUTO, blue = MANUAL
//   8 pump relay output: green = on
//   all yellow: AUTO could run but a device the rule uses is stale - pump off
//   4-8 red: pump stopped for no flow - press START to clear
void updateStatusLed() {
  // COMMA / NO_LED: battery protection takes priority over status indication - LEDs off (as Langley).
  if (langleyData.operatingStatus == OPERATING_STATUS_COMMA || langleyData.operatingStatus == OPERATING_STATUS_NO_LED) {
    FastLED.clear(true);
    return;
  }
  // CLOUDY: on every other refresh, as Langley/Daffodil, to cut LED power
  bool isCloudy = (langleyData.operatingStatus == OPERATING_STATUS_CLOUDY);
  bool cloudySkip = isCloudy && !cloudyLedCycleOn;
  if (isCloudy) cloudyLedCycleOn = !cloudyLedCycleOn;
  if (cloudySkip) {
    FastLED.clear(true);
    return;
  }

  if (pumpDataStale) {
    fill_solid(statusLeds, NUM_STATUS_LEDS, CRGB(255, 200, 0));
    ledsShow();
    return;
  }

  fill_solid(statusLeds, NUM_STATUS_LEDS, CRGB::Black);
  uint8_t wifi = wifiStatusCode();
  statusLeds[0] = wifi == 1 ? CRGB::Green : wifi == 3 ? CRGB::Blue : wifi == 2 ? CRGB(255, 140, 0) : CRGB::Black;
  statusLeds[1] = !loraActive ? CRGB::Red : relay.on ? CRGB::Blue : CRGB::Green;
  statusLeds[2] = weatherIsFresh() ? CRGB::Green : CRGB::Red;
  // LED 3: RTC backup coin-cell (CR1220, nominal 3.0V). -1 (read error) counts as critical.
  if (langleyData.rtcBatVolt < 0 || langleyData.rtcBatVolt < 2.4f) {
    statusLeds[3] = CRGB::Red;
  } else if (langleyData.rtcBatVolt < 2.7f) {
    statusLeds[3] = CRGB(255, 140, 0);  // orange
  } else {
    statusLeds[3] = CRGB::Green;
  }
  if (flowRateLpm > 0) statusLeds[4] = CRGB::Blue;
  // LED 5: rain - the RS485 rain sensor is not fitted yet
  statusLeds[7] = autoMode() ? CRGB::Green : CRGB::Blue;
  if (pumpRelayOn) statusLeds[8] = CRGB::Green;
  if (pumpDryRunFault) {
    for (int i = 4; i <= 8; i++) statusLeds[i] = CRGB::Red;
  }
  ledsShow();
}

// Refreshes langleyData, stamps a checksum, and sends it - retrying with CAD + random
// backoff on a busy channel. Same structure as Daffodil.ino's sendMessage<T>(), simplified
// to the one struct type Langley sends today.
// TX-summary sampler, called on core 0 while LoRa.endPacket(false) blocks. Valentino's battery
// current is + while charging, so it's negated here (the record wants + = drawn from battery).
bool langleyTxSample(int16_t &batteryDraw_mA, uint16_t &battery_mV) {
  batteryDraw_mA = (int16_t)(-batteryMonitor.getShuntVoltage_mV() / VALENTINO_SHUNT_OHMS);
  battery_mV = (uint16_t)(batteryMonitor.getBusVoltage_V() * 1000.0f);
  return true;
}

// sleepSeconds > 0 marks the pre-sleep packet: Annabelle then holds weather replies and device
// commands for the moment this unit listens after its VitalSigns (see listenForReplies()).
uint8_t sendLangleyData(long sleepSeconds = 0) {
  if (!loraActive) return LORA_INIT_FAILED;
  statusLeds[1] = CRGB::White;  // sending (blue on LED 1 means relay mode)
  ledsShow();

  readSensorData();          // freshest possible reading right before it goes over the air
  langleyData.sleepTime = sleepSeconds;
  langleyData.wifiStatus = wifiStatusCode();
  langleyData.opMode = weatherIsFresh() ? 0x02 : 0x00;  // bit 1, same as Daffodil's opMode

  LangleyData dataToSend = langleyData;
  // sent by this Langley itself - a relay (Paula, Chinampa) puts its own name here (2026-11-11)
  memset(dataToSend.sentbyarray, 0, sizeof(dataToSend.sentbyarray));
  // its own relay mode, for the "Relay" badge on the device views (2026-11-13, LangleyData.relayStatus)
  dataToSend.relayStatus = relay.on ? 1 : 0;
  dataToSend.checksum = 0;
  dataToSend.checksum = calculateChecksum(dataToSend);

  if (debug) {
    Serial.print("Sending LoRa update, battV=");
    Serial.print(dataToSend.batteryVoltage);
    Serial.print(" checksum=0x");
    Serial.println(dataToSend.checksum, HEX);
  }

  LoRa_txMode();

  uint8_t result = LORA_OK;
  int retries = 0;
  bool keepGoing = true;
  while (keepGoing) {
    LoRaError cadResult = performCAD();

    if (cadResult == LORA_OK) {
      LoRa.beginPacket();
      LoRa.write((uint8_t *)&dataToSend, sizeof(dataToSend));
      vitalSigns.beginTx(foundBatteryMonitor ? langleyTxSample : nullptr,
                         foundSolarMonitor ? (int16_t)(solarMonitor.getShuntVoltage_mV() / VALENTINO_SHUNT_OHMS) : -1);
      bool txOk = LoRa.endPacket(false);
      vitalSigns.endTx(txOk);
      result = txOk ? LORA_OK : LORA_TX_FAILED;
      keepGoing = false;
    } else if (cadResult == LORA_CHANNEL_BUSY) {
      int backoff = random(MIN_BACKOFF * (1 << retries), MAX_BACKOFF * (1 << retries));
      if (debug) Serial.println("Busy, waiting " + String(backoff) + "ms");
      delay(backoff);
      retries++;
      keepGoing = (retries < MAX_RETRIES);
      if (!keepGoing) result = LORA_MAX_RETRIES_REACHED;
    } else {
      result = cadResult;
      keepGoing = false;
    }
  }

  if (result != LORA_OK) errorManager.setLoRaError((LoRaError)result, avgRssi);

  LoRa_rxMode();
  statusLeds[1] = relay.on ? CRGB::Blue : CRGB::Green;
  ledsShow();

  return result;
}

// BQ24650 STAT pins are open-drain LEDs fed from the charger's VREF: about 0 V = no input power,
// about 2.5 V = asserted (LED lit), about 3.3 V = released
ChargerState decodeCharger(float stat1, float stat2) {
  auto level = [](float v) { return v < STAT_UNPOWERED_MAX_VOLTS ? 0 : (v < STAT_ASSERTED_MAX_VOLTS ? 1 : 2); };
  int s1 = level(stat1), s2 = level(stat2);
  if (s1 == 0 && s2 == 0) return CHARGER_NO_INPUT;
  if (s1 == 1 && s2 == 2) return CHARGER_CHARGING;
  if (s1 == 2 && s2 == 1) return CHARGER_COMPLETE;
  if (s1 == 2 && s2 == 2) return CHARGER_SUSPENDED;
  return CHARGER_INVALID;
}

void updateChargeStage() {
  if (chargerState == CHARGER_COMPLETE) {  // LiFePO4 only: termination on, STAT2 lit
    chargeStage = STAGE_COMPLETE;
    return;
  }
  if (chargerState != CHARGER_CHARGING) {
    chargeStage = STAGE_NONE;
    lowCurrentSinceMs = 0;
    return;
  }
  if (langleyData.batteryVoltage < CHARGE_REGULATION_VOLTS - CHARGE_CV_MARGIN_VOLTS) {
    chargeStage = STAGE_BULK;  // constant current: as much as J7 (or the panel) allows
    lowCurrentSinceMs = 0;
    return;
  }
  float floatMa = FLOAT_CURRENT_FRACTION * BATTERY_CAPACITY_MAH;
  bool lowCurrent = langleyData.batteryCurrentMa < floatMa;
  if (lowCurrent && lowCurrentSinceMs == 0) lowCurrentSinceMs = millis();
  // Leave float only when the current clearly rises again (a load or a cloud passing), so a little
  // noise around the threshold does not make it flicker
  if (!lowCurrent && (chargeStage != STAGE_FLOAT || langleyData.batteryCurrentMa > 2 * floatMa)) lowCurrentSinceMs = 0;
  bool settled = lowCurrentSinceMs != 0 && millis() - lowCurrentSinceMs >= FLOAT_SETTLE_MS;
  // LiFePO4 has no float: it stays in constant voltage until the charger terminates
  if (settled && langleyData.batteryChemistry == CHEM_LEAD_ACID) chargeStage = STAGE_FLOAT;
  else if (chargeStage != STAGE_FLOAT || lowCurrentSinceMs == 0) chargeStage = STAGE_ABSORPTION;
}

// Fills the web Power panel (the bolt under the cog), same shape as Daffodil.ino's fillPowerData().
// Runs on the web server task, so it only copies values loop() has already read.
void fillPowerData(JsonDocument &json) {
  unsigned long nowMs = millis();
  json["capacityMah"] = BATTERY_CAPACITY_MAH;
  json["uptimeSec"] = nowMs / 1000;
  json["chemistry"] = langleyData.batteryChemistry;
  JsonObject live = json["live"].to<JsonObject>();
  live["batteryV"] = langleyData.batteryVoltage;
  live["batteryMa"] = langleyData.batteryCurrentMa;  // positive = charging
  live["panelV"] = langleyData.solarVoltage;
  live["panelMa"] = langleyData.solarCurrentMa;
  live["socPercent"] = batteryPowerManager ? batteryPowerManager->stateOfCharge(langleyData.batteryVoltage) : -1;
  live["operatingStatus"] = langleyData.operatingStatus;
  long weatherAge = haveWeatherForecast ? langleyData.secondsTime - lastWeatherForecastSeconds : -1;
  live["weatherAgeSec"] = (weatherAge < 0 || weatherAge >= 9999) ? 9999 : weatherAge;  // 9999 = never received, as Daffodil
  long now = codeTimeNow();
  live["wifiOnDemandSec"] = (wifiOnDemandUntil != 0 && now < (long)wifiOnDemandUntil) ? (long)wifiOnDemandUntil - now : 0;
  live["chargerState"] = (int)chargerState;
  live["chargeStage"] = (int)chargeStage;
  live["chargeVolts"] = CHARGE_REGULATION_VOLTS;
  live["stat1V"] = stat1Volts;
  live["stat2V"] = stat2Volts;
  if (batteryThermistorPresent) live["batteryTempC"] = batteryThermistorC;
  live["boardTempC"] = langleyData.temperature;
  live["rtcBatV"] = langleyData.rtcBatVolt;
  live["pumpRelay"] = pumpRelayOn;
  live["emergencyClosed"] = emergencyClosed;
  live["configRaw"] = configLadderRaw;

  VitalSignsRecord r;
  bool have;
  unsigned long builtMs;
  portENTER_CRITICAL(&lastVitalSignsMux);
  have = haveLastVitalSigns;
  r = lastVitalSigns;
  builtMs = lastVitalSignsBuiltMs;
  portEXIT_CRITICAL(&lastVitalSignsMux);
  if (!have) return;  // none built since this boot yet
  JsonObject v = json["vital"].to<JsonObject>();
  uint32_t nowSec = langleyData.secondsTime;
  v["ageSec"] = (nowMs - builtMs) / 1000;
  v["firmwareBuild"] = r.firmwareBuild;
  v["resetCount"] = r.resetCount;
  v["lastResetReason"] = r.lastResetReason;
  v["lastResetAgoSec"] = (r.lastResetTime && nowSec > r.lastResetTime) ? nowSec - r.lastResetTime : 0;
  v["wakeCount"] = r.wakeCount;
  v["earlyWakeCount"] = r.earlyWakeCount;
  v["commaWakeCount"] = r.commaWakeCount;
  v["awakeSec"] = r.awakeSecondsTotal;
  v["sleptSec"] = r.sleptSecondsTotal;
  v["wakeV"] = r.wakeVoltage_mV / 1000.0;
  v["minV"] = r.minVoltageSinceReport_mV / 1000.0;
  v["txMs"] = r.txDurationMs;
  v["txPeakMa"] = r.txBatteryPeak_mA;
  v["txMinV"] = r.txMinVoltage_mV / 1000.0;
  v["loraTxFails"] = r.loraTxFailCount;
  v["seq"] = r.seq;
  v["chargeInMah"] = r.chargeIn_cmAh / 100.0;
  v["dischargeOutMah"] = r.dischargeOut_cmAh / 100.0;
  v["panelInWh"] = r.panelIn_dmWh / 10000.0;
}

// MCP9808 board temperature (Valentino Build 3), read directly over I2C
bool readMcp9808Register(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(MCP9808_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(MCP9808_ADDR, 2) != 2) return false;
  value = (Wire.read() << 8) | Wire.read();
  return true;
}

bool mcp9808Present() {
  uint16_t id;
  return readMcp9808Register(MCP9808_REG_MANUFACTURER, id) && id == MCP9808_MANUFACTURER_ID;
}

// 13-bit two's complement, 0.0625 C per bit; the top three bits are alert flags
float readBoardCelsius() {
  uint16_t raw;
  if (!readMcp9808Register(MCP9808_REG_AMBIENT, raw)) return NAN;
  int16_t counts = raw & 0x0FFF;
  if (raw & 0x1000) counts -= 4096;
  return counts * 0.0625f;
}

// Reads the battery-chemistry jumper JP3 (GPIO39) and maps it to a BatteryChemistry. Valentino
// Build 3: R16 pulls the pin up to 3V3 and JP3 shorts it to GND. The same net drives the BQ24650
// TERM_EN pin, so the jumper sets the charger's behaviour as well:
//   JP3 open   -> HIGH -> LiFePO4   (charge termination on)
//   JP3 fitted -> LOW  -> lead-acid (termination off, float)
// This is the OPPOSITE of Build 1, where LOW meant LiFePO4. GPIO39 is input-only, so firmware can
// read the jumper but not change the charger mode.
BatteryChemistry detectBatteryChemistry() {
  BatteryChemistry chemistry = (digitalRead(CHEMISTRY_PIN) == LOW) ? CHEM_LEAD_ACID : CHEM_LIFEPO4;
  if (debug) Serial.println(chemistry == CHEM_LIFEPO4 ? "Chemistry-ID: LiFePO4" : "Chemistry-ID: lead-acid");
  return chemistry;
}

// Matches the Adafruit INA219 getBusVoltage_V() calculation exactly (register 0x02, uint16_t,
// bits 15:3, 4 mV per LSB) without needing the full library begin()/calibration - used for the
// quick pre-boot COMMA recheck in setup(), same approach as Daffodil.ino's quickReadBusVoltage().
float quickReadBusVoltage() {
  Wire.beginTransmission(INA219_BATTERY_ADDR);
  Wire.write(0x02);  // INA219 bus voltage register
  if (Wire.endTransmission() != 0) return -1;
  Wire.requestFrom(INA219_BATTERY_ADDR, 2);
  if (Wire.available() < 2) return -1;
  uint16_t raw = ((uint16_t)Wire.read() << 8) | (uint8_t)Wire.read();
  return (int16_t)((raw >> 3) * 4) * 0.001f;
}

// ============================================================
// Setup
// ============================================================
// "Sensor found" bits for VitalSignsRecord.i2cDeviceMask. Bits 0, 1 and 4 are Valentino's and mean
// the same as on Langley. Bits 2 and 3 were Langley's energizer INA219 and fence ADC; on Gloria they
// are U1 and U2 (Annabelle still labels them the Langley way). Bit 5 = OLED.
uint8_t buildI2CStatusMask() {
  uint8_t mask = 0;
  if (foundSolarMonitor)      mask |= 0x01;
  if (foundBatteryMonitor)    mask |= 0x02;
  if (foundShieldConfigADC)   mask |= 0x04;
  if (foundShieldSensorADC)   mask |= 0x08;
  if (foundBoardTempSensor)   mask |= 0x10;
  if (foundOled)              mask |= 0x20;
  return mask;
}

// Sends the VitalSignsRecord VITAL_SIGNS_GAP_MS after the data packet (Annabelle has one LoRa
// FIFO). Same CAD + backoff as sendLangleyData(), without the TX summary hooks - each record
// reports the DATA packet's transmit.
void sendVitalSigns() {
  if (!loraActive) return;
  VitalSignsRecord record = vitalSigns.buildRecord(langleyData.serialnumberarray, FIRMWARE_BUILD, buildI2CStatusMask());
  portENTER_CRITICAL(&lastVitalSignsMux);
  lastVitalSigns = record;
  haveLastVitalSigns = true;
  lastVitalSignsBuiltMs = millis();
  portEXIT_CRITICAL(&lastVitalSignsMux);
  record.totpcode = secretManager.generateCode();
  record.checksum = 0;
  record.checksum = calculateChecksum(record);
  delay(VITAL_SIGNS_GAP_MS);
  LoRa_txMode();
  uint8_t result = LORA_OK;
  int retries = 0;
  bool keepGoing = true;
  while (keepGoing) {
    LoRaError cadResult = performCAD();
    if (cadResult == LORA_OK) {
      LoRa.beginPacket();
      LoRa.write((uint8_t *)&record, sizeof(record));
      result = LoRa.endPacket(false) ? LORA_OK : LORA_TX_FAILED;
      keepGoing = false;
    } else if (cadResult == LORA_CHANNEL_BUSY) {
      delay(random(MIN_BACKOFF * (1 << retries), MAX_BACKOFF * (1 << retries)));
      retries++;
      keepGoing = (retries < MAX_RETRIES);
      if (!keepGoing) result = LORA_MAX_RETRIES_REACHED;
    } else {
      result = cadResult;
      keepGoing = false;
    }
  }
  LoRa_rxMode();
  if (result == LORA_OK) vitalSigns.markSent();
  lastVitalSignsMs = millis();
}

void setup() {
  vitalSigns.captureBoot();  // reset reason + wake cause only, no I/O
  Serial.begin(115200);
  delay(100);

  // Pump relay off and the shield rail off before anything else. A deep sleep holds both pins
  // (gpio_deep_sleep_hold_en), so release the holds first or digitalWrite has no effect.
  gpio_hold_dis((gpio_num_t)PUMP_RELAY_PIN);
  gpio_hold_dis((gpio_num_t)SHIELD_POWER_PIN);
  pinMode(PUMP_RELAY_PIN, OUTPUT);
  setPumpRelay(false);
  pinMode(SHIELD_POWER_PIN, OUTPUT);
  digitalWrite(SHIELD_POWER_PIN, SHIELD_POWER_OFF);

  if (debug) Serial.println("Gloria starting...");
  if (debug) {
    Serial.print("LangleyData size=");
    Serial.println(sizeof(LangleyData));
  }

  pinMode(WATCHDOG_DONE, OUTPUT);
  digitalWrite(WATCHDOG_DONE, LOW);
  // The TPL5010 only sends WAKE if it got a DONE during the interval, otherwise it pulses RSTn -
  // pet once here so a slow boot cannot run into the first interval (found on Build 3, 2026-10-21)
  digitalWrite(WATCHDOG_DONE, HIGH);
  delayMicroseconds(100);
  digitalWrite(WATCHDOG_DONE, LOW);
  pinMode(WATCHDOG_WAKE, INPUT);

  pinMode(START_PIN, INPUT);  // pull-ups are on the shield (R11, R3)
  pinMode(MODE_PIN, INPUT);
  pinMode(SENSOR_J3P2_PIN, INPUT);
  pinMode(SENSOR_J3P1_PIN, INPUT);
  pinMode(SENSOR_J6P2_PIN, INPUT);
  pinMode(SENSOR_J6P1_PIN, INPUT);
  pinMode(CHEMISTRY_PIN, INPUT);  // JP3 - see detectBatteryChemistry()

  // ---- Device identity / config ----
  secretManager.getDeviceSensorConfig(langleyData.devicename, langleyData.deviceshortname, sensor1name, sensor2name,
                                      timezone, latitude, longitude, altitude, minimumEfficiencyForLed, minimumEfficiencyForWifi);
  // getDeviceSensorConfig() leaves timezone as the Preferences default "NoData" unless it was
  // ever explicitly set (SetDeviceSensorConfig, or now SetTimezone) - TimeUtils::parseTimezone
  // never got called at all here previously, so baseOffset stayed at its static default of 0
  // and every epoch computed from the RTC treated local wall-clock time as if it were UTC
  // (same bug found and fixed in Daffodil.ino). Fall back to a sane default only if nothing
  // valid has actually been configured, so a real SetTimezone selection persists across reboots.
  if(timezone=="" || timezone=="NoData"){
    timezone = "AEST-10AEDT,M10.1.0,M4.1.0/3";
  }
  TimeUtils::parseTimezone(timezone);
  langleyData.latitude = (float)latitude;
  langleyData.longitude = (float)longitude;
  langleyData.altitude = (float)altitude;
  // Fence topology mapping - see conversation 2026-07-26. Own NVS namespace, set via SetParent#.
  secretManager.getTopologyConfig(langleyData.parentShortname, langleyData.branchLabel);
  if (debug) {
    Serial.print("devicename=");
    Serial.println(langleyData.devicename);
  }

  // ---- Solar / battery power management ----
  // Constructed before Wire/RTC below - neither SolarInfo's nor BatteryPowerManager's constructor
  // touches I2C, so this can happen early enough for the pre-boot COMMA recheck (right after
  // resetI2CDevices() below) to use batteryPowerManager->commaVoltage() without needing the rest
  // of setup() (RTC, INA219 begin(), LoRa) to run first.
  solarInfo = new SolarInfo(Serial, latitude, longitude, altitude);
  BatteryChemistry chemistry = detectBatteryChemistry();
  langleyData.batteryChemistry = (uint8_t)chemistry;  // sent over LoRa - see LangleyData.h
  batteryPowerManager = new BatteryPowerManager(chemistry, BATTERY_CAPACITY_MAH, *solarInfo);

  Wire.end();
  delay(10);  // Give the bus a moment to settle
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  resetI2CDevices();

  // ---- Pre-boot COMMA recheck ----
  // Mirrors Daffodil.ino's rtc_comma_mode handling: if we went to sleep critically low, do a
  // cheap direct-register voltage read before paying for a full boot (RTC/INA219/LoRa init).
  // Still critical -> pet the watchdog and go straight back to sleep; recovered -> clear the
  // flag and fall through to a normal boot. Self-healing, not a permanent latch.
  if (rtc_comma_mode) {
    float _qv = quickReadBusVoltage();
    if (_qv < 0 || _qv < batteryPowerManager->commaVoltage()) {
      if (debug) { Serial.printf("COMMA: %.2fV - returning to sleep.\n", _qv); Serial.flush(); }
      vitalSigns.recordBoot(0);  // RTC not started yet on this path
      vitalSigns.recordCommaWake(_qv, 0, 600);
      digitalWrite(WATCHDOG_DONE, HIGH);
      delayMicroseconds(100);
      digitalWrite(WATCHDOG_DONE, LOW);
      gpio_deep_sleep_hold_en();
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      esp_sleep_enable_timer_wakeup(600ULL * 1000000ULL);  // recheck in 10 min
      esp_deep_sleep_start();
    }
    if (debug) { Serial.printf("COMMA exiting: voltage recovered to %.2fV\n", _qv); Serial.flush(); }
    rtc_comma_mode = false;
  }

  // ---- RTC ----
  pinMode(RTC_CLK_OUT, INPUT_PULLUP);  // set up interrupt pin
  digitalWrite(RTC_CLK_OUT, HIGH);     // turn on pullup resistors
  // attach interrupt to set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);

  timeManager.start();
  timeManager.PCF8563osc1Hz();
  currentTimerRecord = timeManager.now();
  vitalSigns.recordBoot(timeManager.getCurrentTimeInSeconds(currentTimerRecord));
  vitalSigns.recordWakeVoltage(quickReadBusVoltage());  // before LoRa/WiFi add any load
  hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);

  // ---- Current/power monitors ----
  foundSolarMonitor = solarMonitor.begin();
  foundBatteryMonitor = batteryMonitor.begin();
  if (debug) Serial.println(foundSolarMonitor ? "solar INA219 OK" : "solar INA219 NOT FOUND");
  if (debug) Serial.println(foundBatteryMonitor ? "battery INA219 OK" : "battery INA219 NOT FOUND");
  if (!foundSolarMonitor || !foundBatteryMonitor) {
    errorManager.setI2CError(I2C_DEVICE0_ERROR);
  }

  // ---- Gloria shield ADCs (on V50, always powered) ----
  foundShieldSensorADC = shieldSensorADC.begin();
  shieldSensorADC.setGain(0);  // +-6.144 V: the sensors and the ADS1115 run from 5 V
  if (i2cDevicePresent(GLORIA_ADS_CONFIG_ADDR_ALT)) {
    shieldConfigADC = &shieldConfigADCatScl;
    shieldConfigAddr = GLORIA_ADS_CONFIG_ADDR_ALT;
  }
  foundShieldConfigADC = shieldConfigADC->begin();
  shieldConfigADC->setGain(0);
  if (debug) Serial.printf("Gloria U2 0x%02X %s, U1 0x%02X %s\n", GLORIA_ADS_SENSORS_ADDR, foundShieldSensorADC ? "OK" : "NOT FOUND",
                           shieldConfigAddr, foundShieldConfigADC ? "OK" : "NOT FOUND");

  // ---- Valentino ADS1115 (RTC coin cell; also thermistor and charger STAT on AIN0/2/3) ----
  foundValentinoADC = valentinoADC.begin();
  valentinoADC.setGain(VALENTINO_ADS_GAIN_4V096);
  if (debug) Serial.println(foundValentinoADC ? "Valentino ADS1115 OK" : "Valentino ADS1115 NOT FOUND");

  // ---- Onboard temperature ----
  foundBoardTempSensor = mcp9808Present();
  if (debug) Serial.println(foundBoardTempSensor ? "MCP9808 OK" : "MCP9808 NOT FOUND");
  // Unique per-device ID. Build 1 used the DS18B20's ROM address; Build 3 has no DS18B20, so it
  // is the ESP32's factory MAC (6 bytes, the last two stay 0). Without this,
  // langleyData.serialnumberarray stays all-zero and the telepathon's "Serial Number" is blank.
  // A unit moved to a Build 3 board therefore reports a new serial number.
  DeviceSerial::fill(langleyData.serialnumberarray);  // the efuse MAC (2026-10-30)

  // ---- Pump rule and flow meter ----
  pumpRuleLoad();
  if (debug) Serial.println("Pump rule: " + pumpRuleToJson(pumpRule));
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowPulseISR, FALLING);
  pollGloriaInputs();  // so autoMode() reads the switch from the start
  delay(SWITCH_DEBOUNCE_MS + 5);
  pollGloriaInputs();

  // ---- Shield rail, OLED and status LEDs ----
  setShieldPower(true);
  delay(100);  // let V50_SW settle before talking to the OLED
  foundOled = oledBegin();
  if (debug) Serial.println(foundOled ? "OLED OK" : "OLED NOT FOUND");
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(statusLeds, NUM_STATUS_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS_MAX);
  FastLED.clear();
  ledsShow();
  showSplash();

  // ---- LoRa ----
  SPI.begin(SCK, MISO, MOSI);
  pinMode(LoRa_SS, OUTPUT);
  pinMode(LORA_RESET, OUTPUT);
  pinMode(LORA_DIO0, INPUT);
  digitalWrite(LoRa_SS, HIGH);
  LoRa.setPins(LoRa_SS, LORA_RESET, LORA_DIO0);
  if (!LoRa.begin(433E6)) {
    if (debug) Serial.println("Starting LoRa failed!");
    errorManager.setLoRaError(LORA_INIT_FAILED);
  } else {
    loraActive = true;
    LoRa.setSPIFrequency(1000000);
    LoRa.setTxPower(5);
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.enableCrc();
    attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onLoraDio0, RISING);  // see onLoraDio0()
    if (debug) Serial.println("LoRa OK");
    // relay: stamps the shortname (e.g. "LWES"), else the first 8 characters of the device name
    relay.begin(langleyData.deviceshortname[0] ? langleyData.deviceshortname : langleyData.devicename, relaySend, relayBusy);
    relayLoadSettings();
    LoRa_rxMode();  // idle in receive mode between sends
  }

  displayTimer.start();
  sendTimer.start();

  // Show initial hardware state without waiting for the first RTC tick
  updateOperatingMode();
  updateDisplays();
  updateStatusLed();

  if (debug) Serial.println("setup complete");
}

// How long a duty-cycling unit sleeps between wakes - also how often one awake only for the relay
// sends its own packet (2026-11-11), so the relay doesn't make Langley talk more than it would asleep.
long dutyCycleSeconds() {
  if (batteryPowerManager == nullptr) return 1800L;
  long seconds = (long)batteryPowerManager->calculateSleepSeconds(currentTimerRecord, hourlySolarPowerData);
  // TEMPORARY - overnight battery-sizing calibration run (see conversation 2026-07-14).
  // calculateSleepSeconds()'s night formula (the "450" constant) was tuned for Daffodil's
  // small battery, not Langley's 7Ah lead-acid. Overriding to a fixed 3-minute cadence for
  // one night so we get a uniform, fine-grained voltage-sag + coulomb-counted mAh dataset to
  // build a real capacity-aware budget formula from. Remove this override once that's done.
  if (hourlySolarPowerData.efficiency <= 0.3) seconds = 180L;
  return seconds;
}

void goToSleep() {
  // Only on-demand WiFi ending, or the serial goToSleep command, gets here with WiFi up - take it
  // down before the final LoRa send so it can't load the rail during TX (same as Daffodil)
  if (wifiRunning) stopWifi();
  readSensorData();

  bool isComma = (langleyData.operatingStatus == OPERATING_STATUS_COMMA);

  long seconds_sleep;
  if (isComma) {
    // Fixed short recheck cadence while critically low, same as Daffodil.ino's rtc_comma_mode -
    // calculateSleepSeconds()'s day/night formula isn't meant for this case.
    rtc_comma_mode = true;
    seconds_sleep = 600L;
  } else {
    seconds_sleep = dutyCycleSeconds();
  }

  // Overnight mAh accumulator (calibration run - see conversation 2026-07-14). Armed on the
  // first sleep after sunset, frozen again once daytime efficiency resumes. Each cycle adds the
  // awake part - the current measured moments ago by readSensorData() times the time awake
  // since this wake (millis(), every wake is a fresh boot) - plus the sleep part, the upcoming
  // sleep at SLEEP_CURRENT_MA. Counting the awake current (~50 mA with LoRa) over the whole
  // sleep, as before, overstated the 2026-10-21 night about tenfold (494 mAh). The awake part
  // only counts while actually discharging (batteryCurrentMa < 0 - Valentino convention,
  // positive = charging).
  if (!isComma) {
    if (hourlySolarPowerData.efficiency > 0.3) {
      overnightTrackingActive = false;
    } else {
      if (!overnightTrackingActive) {
        overnightMah = 0.0f;
        overnightTrackingActive = true;
        if (debug) Serial.println("Overnight mAh tracking armed (sunset)");
      }
      if (langleyData.batteryCurrentMa < 0) {
        float awakeHours = (millis() / 1000.0f) / 3600.0f;
        overnightMah += (-langleyData.batteryCurrentMa) * awakeHours;
      }
      overnightMah += SLEEP_CURRENT_MA * (seconds_sleep / 3600.0f);
    }
  }
  langleyData.overnightMah = overnightMah;

  uint64_t sleep_time_us = (uint64_t)seconds_sleep * 1000000ULL;
  if (debug) Serial.printf("Sleeping for %ld seconds\n", seconds_sleep);

  // Preserve CLOUDY/NO_LED/COMMA in the transmitted status (they already explain why we're
  // sleeping) - only fall back to a generic SLEEP label if goToSleep() was reached some other
  // way (e.g. the manual serial command while still in FULL_MODE).
  if (langleyData.operatingStatus != OPERATING_STATUS_COMMA &&
      langleyData.operatingStatus != OPERATING_STATUS_CLOUDY &&
      langleyData.operatingStatus != OPERATING_STATUS_NO_LED) {
    langleyData.operatingStatus = OPERATING_STATUS_SLEEP;
  }

  if (loraActive) {
    // Battery critically low - skip the heartbeat send to preserve what charge remains (matches
    // the same skip already done for the regular sendTimer path in loop()).
    if (!isComma) sendLangleyData(seconds_sleep);
    // Close this awake cycle first so the record includes it. Skipped in COMMA (running totals
    // carry over to the first pulse after recovery) - except right after a real reset.
    vitalSigns.recordSleep(timeManager.getCurrentTimeInSeconds(timeManager.now()), (uint32_t)seconds_sleep);
    if (!isComma || vitalSigns.resetReportPending()) {
      sendVitalSigns();
      listenForReplies();  // stale-forecast reply and/or a WiFi command - see listenForReplies()
      sendPendingLinkAck(true);  // a ping heard in that window: answer before sleeping
    }
    // WiFi on demand arrived in that window: wake again in 1 s and stay awake for it (loop() skips
    // sleep while wifiOnDemandAllowed()). The VitalSigns and overnight mAh just recorded already
    // counted the full planned sleep - a rare few minutes overstated, accepted (same as Daffodil).
    if (wifiOnDemandAllowed()) {
      seconds_sleep = 1;
      sleep_time_us = 1000000ULL;
    }
    LoRa.sleep();
  }

  // The WS2812 strip latches its last colour while it has power (Langley, 2026-10-21), and the OLED
  // holds its last frame: clear both, then switch V50_SW off for the sleep. The pump relay is off
  // while asleep (GPIO17 low, held by gpio_deep_sleep_hold_en below) - no pump logic yet.
  setShieldPower(false);
  setPumpRelay(false);

  // Safe-state SPI pins so LoRa stays deselected during sleep
  pinMode(MISO, INPUT_PULLDOWN);
  pinMode(MOSI, INPUT_PULLDOWN);
  pinMode(SCK, INPUT_PULLDOWN);
  pinMode(LoRa_SS, INPUT_PULLUP);

  gpio_deep_sleep_hold_en();

  if (debug) {
    Serial.println("Entering deep sleep.");
    Serial.flush();
  }

  // Pet watchdog immediately before sleeping to get a fresh window on wakeup
  digitalWrite(WATCHDOG_DONE, HIGH);
  delayMicroseconds(100);
  digitalWrite(WATCHDOG_DONE, LOW);

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(sleep_time_us);
  esp_deep_sleep_start();
}

// ============================================================
// Main loop
// ============================================================
void loop() {
  if (loraDio0Fired) {
    loraDio0Fired = false;
    // Copy it out and listen again immediately - see loraRxBuf. A packet still waiting to be
    // processed is replaced by the newer one.
    int n = loraCopyPacket();
    if (n == sizeof(DeviceCommandRecord)) {
      // A command or Paula's ping: handled now, not on the 1 s tick (2026-11-03). loraRxBuf holds one
      // packet, and Annabelle answers a ping 150 ms after it - that answer replaced the ping before the
      // tick ran, so Langley_W never answered a ping. Handling one is only a code check.
      processLoraPacket(n);
    } else if (n > 0) {
      loraPacketSize = n;
      loraReceived = true;
    }
  }
  sendPendingLinkAck(false);  // Paula's ping, answered in this unit's slot (handlePing)
  relay.service(loraDio0Fired);  // packets waiting to be relayed whose time has come
  relayApplyWebRequest();        // the web page's Relay switch

  pollGloriaInputs();  // START and AUTO/MANUAL, debounced

  // Valentino PCB has no pull-up on RTC_CLK_OUT (GPIO34 is input-only, internal pull-up
  // silently ignored). Fall back to millis() so the 1Hz cadence works regardless.
  static unsigned long lastTickMs = 0;
  unsigned long nowMs = millis();
  if (clockTicked || (nowMs - lastTickMs >= 1000)) {
    lastTickMs = nowMs;

    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);

    currentTimerRecord = timeManager.now();  // single per-second RTC read, reused by everything below

    if (loraReceived) {
      loraReceived = false;
      processLoraPacket(loraPacketSize);  // from loraRxBuf - the radio has been listening since it was copied out
    }

    updateFlow();
    readGloriaShield();  // SW1 for the no-flow check, every second
    pumpApplyWebChanges();
    pumpControl();

    displayTimer.tick();
    sendTimer.tick();

    if (displayTimer.status()) {
      displayTimer.reset();
      readSensorData();
      updateOperatingMode();
      manageWifi();
      updateDisplays();
      updateStatusLed();
    }

    // The sleep decision runs every second (2026-11-11 - it used to wait for the send timer, which
    // was fine at 10 s but would keep a duty-cycling unit up for a whole minute at 60 s). From 10 s
    // after boot, so the first readings and operating mode are in.
    // SLEEP/COMMA/CLOUDY/NO_LED all duty-cycle: wake, stay up for AWAKE_WINDOW_SECONDS, then
    // goToSleep() - which sends the wake's packet with the freshest data and never returns - for a
    // length picked by dutyCycleSeconds(). FULL_MODE keeps running, sending every
    // loraSendIntervalSeconds. Awake only for the relay (relayDark), it sends its own packet every
    // dutyCycleSeconds(), as often as it would have asleep.
    if (millis() >= 10000UL) {
      bool isComma = (langleyData.operatingStatus == OPERATING_STATUS_COMMA);
      bool isDutyCycleStatus = (isComma ||
                                langleyData.operatingStatus == OPERATING_STATUS_SLEEP ||
                                langleyData.operatingStatus == OPERATING_STATUS_CLOUDY ||
                                langleyData.operatingStatus == OPERATING_STATUS_NO_LED);
      bool awakeWindowElapsed = millis() >= (uint32_t)AWAKE_WINDOW_SECONDS * 1000UL;
      // WiFi on demand keeps it awake. Its floor is above sleepingVoltage(), so a low battery
      // still sleeps, and COMMA always does.
      if (wifiOnDemandAllowed()) isDutyCycleStatus = false;
      // AUTO has to stay awake to hear the rule's devices, and the relay only holds while awake. A low
      // battery (SLEEP, COMMA) still sleeps - pumpControl() has turned the pump off then.
      bool lowBattery = isComma || langleyData.operatingStatus == OPERATING_STATUS_SLEEP;
      if ((autoMode() || pumpRelayOn) && !lowBattery) isDutyCycleStatus = false;
      // Relay on and the battery above relayMinVolts: stay awake to relay, dark (2026-11-11)
      bool darkBefore = relayDark;
      relayDark = false;
      if (isDutyCycleStatus && !isComma && relayCanKeepAwake()) {
        isDutyCycleStatus = false;
        relayDark = true;
      }
      if (relayDark && !darkBefore) {
        if (foundOled) {
          display.clearDisplay();
          display.display();
        }
        FastLED.clear(true);
      }
      static uint32_t relayDarkSentMs = 0;
      if (!relayDark) relayDarkSentMs = 0;

      bool sendNow = false;
      if (isComma) {
        // Critically low - don't wait out the full awake window, sleep immediately (goToSleep()
        // itself skips the LoRa send in this case to preserve what charge remains).
        goToSleep();
      } else if (isDutyCycleStatus && awakeWindowElapsed) {
        goToSleep();
      } else if (relayDark) {
        if (relayDarkSentMs == 0 || millis() - relayDarkSentMs >= (uint32_t)dutyCycleSeconds() * 1000UL) {
          relayDarkSentMs = millis();
          sendNow = true;
        }
      } else if (sendTimer.status()) {
        sendTimer.reset();
        sendNow = !isDutyCycleStatus;  // a duty-cycling wake sends from goToSleep()
      }
      if (sendNow) {
        sendLangleyData();
        // FULL_MODE never reaches goToSleep(), where vital signs are normally sent. millis()
        // restarts on every wake, so a duty-cycling unit never gets past the first check.
        if (millis() > VITAL_SIGNS_AWAKE_INTERVAL_MS && millis() - lastVitalSignsMs > VITAL_SIGNS_AWAKE_INTERVAL_MS) {
          sendVitalSigns();
        }
      }
    }
  }


  if (Serial.available() != 0) {
    String command = Serial.readString();
    command.trim();

    if (command.startsWith("Relay")) {
      // Relay#1 / Relay#0 / Relay#1#<dBm> / Relay - see the relay section; kept across sleeps
      relay.applyCommand(generalFunctions.getValue(command, '#', 1), generalFunctions.getValue(command, '#', 2));
      {  // Relay#1#<dBm>#<min volts>
        String v = generalFunctions.getValue(command, '#', 3);
        v.trim();
        if (v.length() && v.toFloat() >= 9.0f && v.toFloat() <= 15.0f) relayMinVolts = v.toFloat();
      }
      relaySaveSettings();
      Serial.print("RELAY ");
      relay.printJson(Serial);
      Serial.println();
      Serial.println(F("Ok-Relay"));
    } else if (command.startsWith("SetPumpRule")) {
      // SetPumpRule#<json> - see GloriaPumpRule.h
      String json = command.substring(command.indexOf('#') + 1);
      String e = pumpRuleSet(json);
      if (e.length()) Serial.println("Failure-SetPumpRule-" + e);
      else {
        Serial.println(pumpRuleToJson(pumpRule));
        Serial.println(F("Ok-SetPumpRule"));
      }
    } else if (command.startsWith("GetPumpRule")) {
      Serial.println(pumpRuleToJson(pumpRule));
      Serial.println(F("Ok-GetPumpRule"));
    } else if (command.startsWith("GetPumpStatus")) {
      printPumpStatus();
      Serial.println(F("Ok-GetPumpStatus"));
    } else if (command.startsWith("GetHeardDevices")) {
      printHeardDevices();
      Serial.println(F("Ok-GetHeardDevices"));
    } else if (command.startsWith("ClearPumpFault")) {
      pumpDryRunFault = false;
      pumpSet(false, PUMP_IDLE);
      Serial.println(F("Ok-ClearPumpFault"));
    } else if (command.startsWith("PumpRelay")) {
      // PumpRelay#on / PumpRelay#off - by hand, as START in MANUAL (cutoff time and battery minimum
      // apply; in AUTO the rule takes over again within a second). "Relay" is Langley's LoRa relay mode.
      bool on = generalFunctions.getValue(command, '#', 1) == "on";
      pumpSet(on, on ? PUMP_ON_MANUAL : PUMP_IDLE);
      readGloriaShield();
      Serial.printf("Pump relay output %s, SW1 %s\n", pumpRelayOn ? "ON" : "off",
                    foundShieldConfigADC ? (emergencyClosed ? "closed" : "OPEN - relay cannot close") : "not readable (U1 missing)");
      Serial.println(F("Ok-PumpRelay"));
    } else if (command.startsWith("GetGloriaShield")) {
      readGloriaShield();
      Serial.printf("relay %s  SW1 %s (coil %.2f V)  mode pin %s  START %s  config raw %d\n", pumpRelayOn ? "ON" : "off",
                    emergencyClosed ? "closed" : "OPEN", coilNodeVolts, modeSwitchLow ? "LOW" : "HIGH", startPressed ? "pressed" : "released",
                    configLadderRaw);
      Serial.printf("analog J3.2 %.3f  J3.1 %.3f  J6.2 %.3f  J6.1 %.3f V   digital %u %u %u %u\n", shieldSensorVolts[0], shieldSensorVolts[1],
                    shieldSensorVolts[2], shieldSensorVolts[3], shieldDigital[0], shieldDigital[1], shieldDigital[2], shieldDigital[3]);
      Serial.println(F("Ok-GetGloriaShield"));
    } else if (command.startsWith("Ping")) {
      Serial.println(F("Ok-Ping"));
    } else if (command.startsWith("debug")) {
      debug = (generalFunctions.getValue(command, '#', 1).toInt() > 0);
      Serial.println("Ok-debug");
      Serial.flush();
    } else if (command.startsWith("goToSleep")) {
      Serial.println("Going to sleep...");
      goToSleep();
    } else if (command.startsWith("SetTime")) {
      timeManager.setTime(command);
      Serial.println("Ok-SetTime");
      Serial.flush();
    } else if (command.startsWith("GetTime")) {
      timeManager.printTimeToSerial(currentTimerRecord);
      Serial.flush();
      Serial.println("Ok-GetTime");
      Serial.flush();
    } else if (command.startsWith("SetDeviceSensorConfig")) {
      //SetDeviceSensorConfig#TopTank#TOPT#Flow#Tank#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#
      //SetDeviceSensorConfig#Langley_West#LWES#Fence#NA#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#
      
      String _dn = generalFunctions.getValue(command, '#', 1);
      String _sn = generalFunctions.getValue(command, '#', 2);
      String _s1 = generalFunctions.getValue(command, '#', 3);
      String _s2 = generalFunctions.getValue(command, '#', 4);
      timezone = generalFunctions.getValue(command, '#', 5);
      latitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 6));
      longitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 7));
      altitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 8));
      minimumEfficiencyForLed = generalFunctions.getValue(command, '#', 9).toInt();
      minimumEfficiencyForWifi = generalFunctions.getValue(command, '#', 10).toInt();
      _dn.toCharArray(langleyData.devicename, sizeof(langleyData.devicename));
      _sn.toCharArray(langleyData.deviceshortname, sizeof(langleyData.deviceshortname));
      _s1.toCharArray(sensor1name, sizeof(sensor1name));
      _s2.toCharArray(sensor2name, sizeof(sensor2name));
      secretManager.saveDeviceSensorConfig(_dn, _sn, _s1, _s2, timezone,
                                           latitude, longitude, altitude,
                                           minimumEfficiencyForLed, minimumEfficiencyForWifi);
      Serial.println(F("Ok-SetDeviceSensorConfig"));
    } else if (command.startsWith("GetDeviceSensorConfig")) {
      Serial.print(langleyData.devicename);
      Serial.print("#");
      Serial.print(langleyData.deviceshortname);
      Serial.print("#");
      Serial.print(sensor1name);
      Serial.print("#");
      Serial.print(sensor2name);
      Serial.print("#");
      Serial.print(timezone);
      Serial.print("#");
      Serial.print(latitude, 8);
      Serial.print("#");
      Serial.print(longitude, 8);
      Serial.print("#");
      Serial.print(altitude, 2);
      Serial.print("#");
      Serial.print(minimumEfficiencyForLed);
      Serial.print("#");
      Serial.print(minimumEfficiencyForWifi);
      Serial.print("#");
      Serial.println(F("Ok-GetDeviceSensorConfig"));
    } else if (command.startsWith("SetDeviceName")) {
      generalFunctions.getValue(command, '#', 1).toCharArray(langleyData.devicename, sizeof(langleyData.devicename));
      Serial.println(F("Ok-SetDeviceName"));
    } else if (command.startsWith("SetDeviceShortName")) {
      String sn = generalFunctions.getValue(command, '#', 1);
      sn.toCharArray(langleyData.deviceshortname, sizeof(langleyData.deviceshortname));
      Serial.println(F("Ok-SetDeviceShortName"));
    } else if (command.startsWith("SetTimezone")) {
      // SetTimezone#AEST-10AEDT,M10.1.0,M4.1.0/3
      String tz = generalFunctions.getValue(command, '#', 1);
      secretManager.setTimeZone(tz);
      TimeUtils::parseTimezone(tz);
      Serial.println(F("Ok-SetTimezone"));
    } else if (command.startsWith("SetParent")) {
      // SetParent#<parentShortname>#<branchLabel>#
      // parentShortname: the deviceshortname of the unit immediately upstream toward the
      // energizer. Blank ("SetParent###") marks this unit as the id-0/energizer node itself.
      // branchLabel: single letter, only needed on a node that is one of several children of
      // the same parent (a real Y-split) - leave blank to inherit the parent's label unchanged.
      // See LangleyData.h for the full scheme.
      String _psn = generalFunctions.getValue(command, '#', 1);
      String _bl = generalFunctions.getValue(command, '#', 2);
      _psn.toCharArray(langleyData.parentShortname, sizeof(langleyData.parentShortname));
      _bl.toCharArray(langleyData.branchLabel, sizeof(langleyData.branchLabel));
      secretManager.saveTopologyConfig(_psn, _bl);
      Serial.println(F("Ok-SetParent"));
    } else if (command.startsWith("GetParent")) {
      Serial.print(langleyData.parentShortname);
      Serial.print("#");
      Serial.print(langleyData.branchLabel);
      Serial.print("#");
      Serial.println(F("Ok-GetParent"));
    } else if (command.startsWith("SetGroupId")) {
      String grpId = generalFunctions.getValue(command, '#', 1);
      secretManager.setGroupIdentifier(grpId);
      Serial.println(F("Ok-SetGroupId"));
    } else if (command.startsWith("SetSecret")) {
      String secret = generalFunctions.getValue(command, '#', 1);
      int numberDigits = generalFunctions.getValue(command, '#', 2).toInt();
      int periodSeconds = generalFunctions.getValue(command, '#', 3).toInt();
      secretManager.saveSecret(secret, numberDigits, periodSeconds);
      Serial.println("Ok-SetSecret");
      Serial.flush();
    } else if (command.startsWith("GetSecret")) {
      Serial.println(secretManager.readSecret());
      Serial.println("Ok-GetSecret");
      Serial.flush();
    } else if (command.startsWith("SetProductDefinition")) {
      // SetProductDefinition#<name>#<powerSource>#<battery>#<pcbs>#<firmware>
      String pdName = generalFunctions.getValue(command, '#', 1);
      String pdPowerSource = generalFunctions.getValue(command, '#', 2);
      String pdBattery = generalFunctions.getValue(command, '#', 3);
      String pdPcbs = generalFunctions.getValue(command, '#', 4);
      String pdFirmware = generalFunctions.getValue(command, '#', 5);
      secretManager.saveProductDefinition(pdName, pdPowerSource, pdBattery, pdPcbs, pdFirmware);
      Serial.println("Ok-SetProductDefinition");
      Serial.flush();
    } else if (command.startsWith("GetProductDefinition")) {
      String pdName, pdPowerSource, pdBattery, pdPcbs, pdFirmware;
      secretManager.getProductDefinition(pdName, pdPowerSource, pdBattery, pdPcbs, pdFirmware);
      unsigned long commissionDate = secretManager.getCommissionDate();
      // 1704067200 = 2024-01-01 - anything before that is the RTC's power-on/reset default
      // (e.g. year 2000), not a real commission date. Treat it the same as never-set and
      // recapture, instead of permanently keeping whatever garbage the RTC had the first
      // time this was called (e.g. before SetTime had ever actually run correctly).
      if (commissionDate < 1704067200UL) {
        commissionDate = timeManager.getCurrentTimeInSeconds(timeManager.now());
        secretManager.setCommissionDate(commissionDate);
      }
      // Same response format as Daffodil's so the backend parser doesn't need to branch.
      String pdSSID = secretManager.getSSID();
      String pdWifiPassword = secretManager.getWifiPassword();
      String pdSoftAPSSID = secretManager.getSoftAPSSID();
      String pdSoftAPPassword = secretManager.getSoftAPPASS();
      String pdHostName = secretManager.getHostName();
      String pdStationMode = secretManager.getStationMode() ? "Station" : "AccessPoint";
      unsigned long pdCurrentTime = timeManager.getCurrentTimeInSeconds(timeManager.now());
      String pdDeviceName = String(langleyData.devicename);
      String pdDeviceShortName = String(langleyData.deviceshortname);
      char pdSerialNumberBuf[13];
      snprintf(pdSerialNumberBuf, sizeof(pdSerialNumberBuf), "%012llx", ESP.getEfuseMac());
      String pdSerialNumber = String(pdSerialNumberBuf);
      Serial.println("Ok-GetProductDefinition#" + pdName + "#" + pdPowerSource + "#" + pdBattery + "#" + pdPcbs + "#" + pdFirmware + "#" + String(commissionDate) + "#" + pdSSID + "#" + pdWifiPassword + "#" + pdSoftAPSSID + "#" + pdSoftAPPassword + "#" + pdHostName + "#" + pdStationMode + "#" + String(pdCurrentTime) + "#" + pdDeviceName + "#" + pdDeviceShortName + "#" + pdSerialNumber);
      Serial.flush();
    } else if (command.startsWith("GetCommandCode")) {
      long code = secretManager.generateCode();
      if (code < 100000) Serial.print("0");
      Serial.println(code);
      Serial.flush();
    } else if (command.startsWith("GetHourlySolarPowerData")) {
      HourlySolarPowerData hspd = solarInfo->calculateActualPower(currentTimerRecord);
      Serial.print("efficiency=");
      Serial.println(hspd.efficiency);
      Serial.print("actualPower=");
      Serial.println(hspd.actualPower);
      Serial.print("irradiance=");
      Serial.println(hspd.irradiance);
      Serial.print("temperature=");
      Serial.println(hspd.temperature);
    } else if (command.startsWith("GetDailySolarPowerSchedule")) {
      DailySolarPowerSchedule schedules[48];
      solarInfo->calculateDailySolarPowerSchedule(schedules, currentTimerRecord);
      for (int i = 0; i < 48; i++) {
        Serial.print(schedules[i].time);
        Serial.print(",");
        Serial.print(TimeUtils::epochToString(schedules[i].time));
        Serial.print(",");
        Serial.print(schedules[i].efficiency);
        Serial.print(",");
        Serial.println(schedules[i].power);
      }
      Serial.println("Ok-DailySolarPowerSchedule");
      Serial.flush();
    } else if (command.startsWith("ConfigWifiSTA")) {
      // ConfigWifiSTA#ssid#password#hostname#
      String ssid = generalFunctions.getValue(command, '#', 1);
      String password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      wifiManager.configWifiSTA(ssid, password, hostname);
      // configWifiSTA() connects immediately - don't leave the radio on if WiFi isn't supposed to be up
      if (!wifiRunning) {
        wifiManager.stop();
        WiFi.disconnect(true);
      }
      Serial.println("Ok-ConfigWifiSTA");
    } else if (command.startsWith("ConfigWifiAP")) {
      // ConfigWifiAP#soft_ap_ssid#soft_ap_password#hostname#
      String soft_ap_ssid = generalFunctions.getValue(command, '#', 1);
      String soft_ap_password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool stat = wifiManager.configWifiAP(soft_ap_ssid, soft_ap_password, hostname);
      Serial.print("ConfigWifiAP result=");
      Serial.println(stat);
      if (!wifiRunning) {
        wifiManager.stop();
        WiFi.disconnect(true);
      }
      Serial.println("Ok-ConfigWifiAP");
    } else if (command.startsWith("GetIpAddress")) {
      Serial.println(ipAddress);
      Serial.println("Ok-GetIpAddress");
      Serial.flush();
    } else if (command.startsWith("RestartWifi")) {
      startWifi();
      Serial.println("Ok-RestartWifi");
      Serial.flush();
    } else {
      Serial.println("Failure-Command Not Found-" + command);
      Serial.flush();
    }
  }

  // Pet the TPL5010 hardware watchdog every loop iteration
  digitalWrite(WATCHDOG_DONE, HIGH);
  delayMicroseconds(100);
  digitalWrite(WATCHDOG_DONE, LOW);
}
