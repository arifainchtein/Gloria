// GloriaPumpRule.h - the pump rule the user writes for AUTO mode (2026-11-18).
//
// Gloria keeps a table of every device it hears over LoRa (name, packet type, last packet), the
// same way Daffodil's Viewer does (Daffodil/ViewerConfig.h). The user builds the rule from those
// devices and the variables of their packet type:
//
//   start: ALL of these hold -> pump on       e.g. Pool > Water level (%) < 90  AND  Pool > Water level (%) > 60
//   stop:  ANY of these holds -> pump off     e.g. Pool > Water level (%) >= 95
//   With no stop conditions, the pump stops as soon as the start conditions no longer all hold.
//
// Protections the firmware adds on top (not part of the rule, cannot be switched off):
//   - a device the rule uses not heard for "stale" seconds: pump off, all LEDs yellow
//   - pump on and the flow still 0 after "dryRunSec": pump off, LEDs 4-8 red, latched until START
//     is pressed or ClearPumpFault
//   - battery in SLEEP or COMMA: pump off (Gloria goes to sleep)
//   - MANUAL mode: START toggles the pump; it stops after "manualMin" minutes or below "manualMinV"
//
// Stored as JSON in Preferences ("gloria"/"rule"), set with the serial command SetPumpRule#<json>:
//   {"stale":600,"dryRunSec":5,"manualMin":30,"manualMinV":12.0,"qfactor":4.5,"autoWhenLow":false,
//    "start":[{"t":"daffodil","d":"Pool","f":"levelPct","op":"<","v":90},
//             {"t":"daffodil","d":"Pool","f":"levelPct","op":">","v":60}],
//    "stop":[{"t":"daffodil","d":"Pool","f":"levelPct","op":">=","v":95}]}
//   t = device type (PUMP_RULE_TYPES), d = device name as it sends it, f = field id of that type
//   (PUMP_RULE_FIELDS), op = < <= > >=, v = the value.
//   qfactor: flow meter pulses per second per L/min (4.5 for the YF-S201 type, see GloriaTankFlowPumpData.h)
//   autoWhenLow: which SW2 position is AUTO (pin LOW = AUTO when true) - set when the case is built
//
// Every packet the rule uses must carry this farm's TOTP code (checked in processLoraPacket()); a device
// whose packets fail the check never appears in the heard-device table.
//
// Adding a device type: a struct with a LoRa size no other packet uses, a PUMP_RULE_TYPE_ entry, its
// fields below, a branch in processLoraPacket() that calls heardRecord(), and its values in
// heardFieldValue() in Gloria.ino.
#ifndef GLORIA_PUMP_RULE_H
#define GLORIA_PUMP_RULE_H
#include <Arduino.h>

#define PUMP_RULE_MAX_CONDITIONS 4  // per list (start, stop)
#define HEARD_MAX_DEVICES 8
#define HEARD_NAME_LEN 17           // 16 characters + terminator (ChinampaData/LangleyData names are 16)

#define PUMP_RULE_TYPE_DAFFODIL 0   // DigitalStablesData (every Daffodil, e.g. Pool)
#define PUMP_RULE_TYPE_CHINAMPA 1   // ChinampaData
#define PUMP_RULE_TYPE_LANGLEY 2    // LangleyData - NOT recorded: it has no TOTP code, and the pump only acts on authenticated data

struct PumpRuleType {
  const char *id;    // in the rule JSON
  const char *name;  // shown to the user
};

static const PumpRuleType PUMP_RULE_TYPES[] = {
  { "daffodil", "Daffodil" },
  { "chinampa", "Chinampa" },
  { "langley", "Langley" },
};
#define PUMP_RULE_TYPE_COUNT (sizeof(PUMP_RULE_TYPES) / sizeof(PUMP_RULE_TYPES[0]))

// Field indexes - must match the order of PUMP_RULE_FIELDS below.
enum {
  PF_DA_LEVEL_CM,
  PF_DA_LEVEL_PCT,
  PF_DA_FLOW1,
  PF_DA_FLOW2,
  PF_DA_OUTDOOR_TEMP,
  PF_CH_SUMP_LEVEL_CM,
  PF_CH_SUMP_LEVEL_PCT,
  PF_CH_FISH_LEVEL_CM,
  PF_CH_FISH_LEVEL_PCT,
  PF_CH_PUMP_FLOW,
  PF_CH_FISH_OUTFLOW,
  PF_CH_OUTDOOR_TEMP,
  PF_LA_TEMP,
  PF_LA_BATTERY,
};

struct PumpRuleField {
  uint8_t type;
  const char *id;     // unique within its type
  const char *label;  // without the device - the device is picked first
  const char *units;
};

static const PumpRuleField PUMP_RULE_FIELDS[] = {
  { PUMP_RULE_TYPE_DAFFODIL, "levelCm", "Water level", "cm" },
  { PUMP_RULE_TYPE_DAFFODIL, "levelPct", "Water level", "%" },
  { PUMP_RULE_TYPE_DAFFODIL, "flow1", "Flow 1", "L/min" },
  { PUMP_RULE_TYPE_DAFFODIL, "flow2", "Flow 2", "L/min" },
  { PUMP_RULE_TYPE_DAFFODIL, "outdoorTemp", "Outdoor temperature", "C" },
  { PUMP_RULE_TYPE_CHINAMPA, "sumpLevelCm", "Sump trough level", "cm" },
  { PUMP_RULE_TYPE_CHINAMPA, "sumpLevelPct", "Sump trough level", "%" },
  { PUMP_RULE_TYPE_CHINAMPA, "fishLevelCm", "Fish tank level", "cm" },
  { PUMP_RULE_TYPE_CHINAMPA, "fishLevelPct", "Fish tank level", "%" },
  { PUMP_RULE_TYPE_CHINAMPA, "pumpFlow", "Pump flow", "L/min" },
  { PUMP_RULE_TYPE_CHINAMPA, "fishOutflow", "Fish tank outflow", "L/min" },
  { PUMP_RULE_TYPE_CHINAMPA, "outdoorTemp", "Outdoor temperature", "C" },
  { PUMP_RULE_TYPE_LANGLEY, "temperature", "Temperature", "C" },
  { PUMP_RULE_TYPE_LANGLEY, "battery", "Battery voltage", "V" },
};
#define PUMP_RULE_FIELD_COUNT (sizeof(PUMP_RULE_FIELDS) / sizeof(PUMP_RULE_FIELDS[0]))

#define PUMP_OP_LT 0
#define PUMP_OP_LE 1
#define PUMP_OP_GT 2
#define PUMP_OP_GE 3
static const char *const PUMP_OP_TEXT[] = { "<", "<=", ">", ">=" };

struct PumpCondition {
  uint8_t field = 0;               // index into PUMP_RULE_FIELDS (which also gives the type)
  char device[HEARD_NAME_LEN] = {};
  uint8_t op = PUMP_OP_LT;
  float value = 0;
};

struct PumpRule {
  uint8_t startCount = 0;
  PumpCondition start[PUMP_RULE_MAX_CONDITIONS];
  uint8_t stopCount = 0;
  PumpCondition stop[PUMP_RULE_MAX_CONDITIONS];
  uint16_t staleSec = 600;
  uint8_t dryRunSec = 5;
  uint16_t manualMin = 30;
  float manualMinV = 12.0f;
  float qfactor = 4.5f;
  bool autoWhenLow = false;
};

// Why the pump is on or off - shown on the OLED and by GetPumpStatus.
enum PumpReason {
  PUMP_IDLE,             // off, nothing wants it on
  PUMP_ON_AUTO,          // on: the start conditions hold
  PUMP_ON_MANUAL,        // on: START in MANUAL
  PUMP_OFF_STOP_RULE,    // off: a stop condition holds (or the start conditions no longer do)
  PUMP_OFF_NO_RULE,      // AUTO with no start conditions
  PUMP_OFF_STALE,        // a device the rule uses is not heard (stale)
  PUMP_OFF_DRY_RUN,      // pump was on with no flow - latched until START / ClearPumpFault
  PUMP_OFF_LOW_BATTERY,  // SLEEP or COMMA
  PUMP_OFF_MANUAL_TIME,  // MANUAL cutoff time reached
  PUMP_OFF_MANUAL_LOW_V  // MANUAL battery minimum
};

// No conditions: the user adds them from the devices Gloria hears (nothing is assumed to exist).
#define PUMP_RULE_DEFAULT "{\"stale\":600,\"dryRunSec\":5,\"manualMin\":30,\"manualMinV\":12.0,\"qfactor\":4.5,\"autoWhenLow\":false,\"start\":[],\"stop\":[]}"

#endif
