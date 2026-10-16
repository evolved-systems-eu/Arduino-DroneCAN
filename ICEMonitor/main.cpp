/*
 * ICEMonitor – Fuel Injection Signal Monitor
 *
 * Measures RPM and injection duty cycle from a Delphi MT05 ECU fuel injector
 * signal, then broadcasts the results as a DroneCAN
 * uavcan.equipment.ice.reciprocating.Status message.
 *
 * Rising edge = start of injection, falling edge = end of injection.
 * Engine: single cylinder, 4-stroke → 1 injection per 2 crankshaft revolutions.
 *
 *   RPM  = 120,000,000 / period_us
 *   duty = 100 × high_us / period_us   (placed in fuel_consumption_rate_cm3pm)
 *
 * Both measurements pass through an EMA filter then a linear transform:
 *   filtered[n] = alpha × raw[n] + (1 − alpha) × filtered[n−1]
 *   output      = A1 × filtered + A0
 *
 * Design assumption: reboot required for any configuration change
 * ---------------------------------------------------------------
 *   Any change to a signal source (THR_SRC, CLT_SRC), pin assignment
 *   (FUELINJ_PIN, THR_PIN, CLT_PIN, TMP0_PIN, TMP1_PIN), INA228 I2C address
 *   (THR_I2C_ADDR, CLT_I2C_ADDR), RPM_MAX, or RPM_K_WIN takes effect only
 *   after a reboot.  All pin, source, and bounds initialisation is performed
 *   once in setup().
 *   The 1 Hz tick only refreshes scalar parameters (EMA alphas, scale/offset,
 *   broadcast intervals, RPM_HOLD_MS, RPM_STOP_MS, RPM_ACCEL, RPM_K_ACC)
 *   which are safe to update live.
 *
 * Pin encoding (shared by all PIN parameters)
 * --------------------------------------------
 *   Negative value  → input disabled / not connected
 *   0=A0  1=A1  4=A4  8=A8  9=A9  10=A10  11=A11  100=B0  101=B1
 *   ADC-capable: 0 1 4 100 101    Interrupt-capable (FUELINJ_PIN): 8 9 10 11 100 101
 *
 * ICE Status (uavcan.equipment.ice.reciprocating.Status)
 * -------------------------------------------------------
 *   Broadcast when at least one source is configured: FUELINJ_PIN ≥ 0,
 *   THR_SRC=2 with a valid I2C address, THR_SRC=1 with THR_PIN ≥ 0,
 *   CLT_SRC=2 with a valid I2C address, or CLT_SRC=1 with CLT_PIN ≥ 0.
 *   Because CLT_SRC defaults to 2 and CLT_I2C_ADDR defaults to 0x40,
 *   the message is broadcast by default even without FUELINJ_PIN or THR.
 *
 *   NODEID     – DroneCAN node ID (default 5)
 *   BAUD       – CAN baudrate kbps: 500 or 1000 (default 500; stored in flash, effective after restart)
 *   FUELINJ_PIN  – injection input pin (default 8 = A8); <0 = disabled
 *   BCAST_MS   – ICE Status broadcast interval ms (default 10)
 *   RPM_MAX    – max measurable RPM; periods faster than this are rejected (default 8000; requires restart)
 *   RPM_ALPHA  – EMA alpha for RPM, 0.01–1.0 (default 0.2)
 *   RPM_A1     – RPM linear scale  (default 1.0)
 *   RPM_A0     – RPM linear offset (default 0.0)
 *   RPM_HOLD_MS – ms no injection signal before holding last RPM + flagging CRANKSHAFT_SENSOR_ERROR (default 500)
 *   RPM_STOP_MS – ms no injection signal before zeroing RPM and reporting STOPPED (default 3000); must be > RPM_HOLD_MS
 *   RPM_ACCEL  – max plausible RPM/s change; samples exceeding this are rejected by the plausibility filter (default 5000)
 *   RPM_K_WIN  – plausibility sliding window size (default 5; requires restart)
 *   RPM_K_ACC  – passes required within the window to accept a sample into the EMA (default 3)
 *   DUTY_ALPHA – EMA alpha for duty cycle (default 0.3)
 *   DUTY_A1    – duty cycle linear scale  (default 1.0)
 *   DUTY_A0    – duty cycle linear offset (default 0.0)
 *   THR_SRC    – throttle source: 0=disabled 1=ADC 2=I2C/INA228 (default 1; requires restart)
 *   THR_I2C_ADDR – INA228 I2C address when THR_SRC=2 (default 0x40; requires restart)
 *   THR_PIN    – ADC input pin when THR_SRC=1 (default -1 = disabled; requires restart)
 *   THR_ALPHA  – EMA alpha for throttle (default 1.0)
 *   THR_A1     – linear scale:  throttle_pct = A1×voltage + A0 (default 1.0)
 *   THR_A0     – linear offset (default 0.0)
 *   CLT_SRC    – coolant temp source: 0=disabled 1=ADC 2=I2C/INA228 (default 2; requires restart)
 *   CLT_I2C_ADDR – INA228 I2C address when CLT_SRC=2 (default 0x40; requires restart)
 *   CLT_PIN    – ADC input pin when CLT_SRC=1 (default -1 = disabled; requires restart)
 *   CLT_ALPHA  – EMA alpha for coolant temperature (default 1.0)
 *   CLT_A1     – linear scale:  coolant_temp = A1×voltage + A0 (default 1.0)
 *   CLT_A0     – linear offset (default 0.0)
 *
 * Temperature sensors (uavcan.equipment.device.Temperature)
 * ---------------------------------------------------------
 *   TMP0_PIN   – analog pin (default -1 = disabled); ≥0 enables sensor 0
 *   TMP0_ID    – device_id in Temperature message (default 0)
 *   TMP0_BCAST – broadcast interval ms (default 1000)
 *   TMP0_ALPHA – EMA alpha 0.01–1.0 (default 1.0 = no filtering)
 *   TMP0_A1    – linear scale:  temp = A1×voltage + A0 (default 1.0)
 *   TMP0_A0    – linear offset (default 0.0)
 *   TMP1_PIN   – analog pin (default -1 = disabled); ≥0 enables sensor 1
 *   TMP1_ID    – device_id in Temperature message (default 1)
 *   TMP1_BCAST – broadcast interval ms (default 1000)
 *   TMP1_ALPHA – EMA alpha 0.01–1.0 (default 1.0 = no filtering)
 *   TMP1_A1    – linear scale  (default 1.0)
 *   TMP1_A0    – linear offset (default 0.0)
 *
 * See ICEMonitor/README.txt for full specification.
 */

#include <Arduino.h>
#include <dronecan.h>
#include <Adafruit_INA228.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Loopback test: define LOOPBACK_TEST to generate a PWM signal on PA1 and
// wire PA1 → injection input pin (default PA8) for self-testing without ECU.
// LB_EN / LB_DUTY / LB_FREQ are DroneCAN parameters, changeable live from GUI.
// Remove this define (and the #ifdef sections below) before production use.
// ---------------------------------------------------------------------------
#define LOOPBACK_TEST
#ifdef LOOPBACK_TEST
  #define LOOPBACK_PIN  PA1   // wire this to the injection input pin
#endif
// ---------------------------------------------------------------------------

// PIN_NOT_FOUND is returned by resolve_pin for unknown or conflicting values.
// All call sites treat it as disabled, same as a negative parameter value.
#define PIN_NOT_FOUND  UINT32_MAX

// Injection period bounds for a 4-stroke single-cylinder engine (1 injection per 2 rev):
//   period_us = 120,000,000 / RPM
// min_period_us is computed from RPM_MAX at boot (requires restart to change).
// RPM above RPM_MAX is reported as RPM_MAX (periods shorter than min_period_us are rejected).
#define MAX_PERIOD_US  400000u   //     300 RPM lower limit
static uint32_t min_period_us = 15000u; // 8,000 RPM default; set from RPM_MAX in setup()

// ---------------------------------------------------------------------------
// Pin map: parameter value → Arduino pin number
// Encoding: port×100 + pin_number  (port A=0, port B=1)
// Board labels: A0 A1 A4 A8 A9 A10 A11 B0 B1  (max 3.3 V on all pins)
// ADC-capable:       0(A0) 1(A1) 4(A4) 100(B0) 101(B1)
// Interrupt-capable: 8(A8) 9(A9) 10(A10) 11(A11) 100(B0) 101(B1)
// Note: A0/A1 are also PWM-capable (TIM2). B0/B1 used for BATT sensing in AP_Periph.
// ---------------------------------------------------------------------------
struct PinMapEntry {
    int      param_val;
    uint32_t arduino_pin;
    bool     adc_capable;       // true = has ADC channel
    bool     interrupt_capable; // true = suitable for FUELINJ_PIN interrupt input
};

static const PinMapEntry pin_map[] = {
    //  val  pin    adc    int   label  ADC      PWM        INT   max    notes
    {0,   PA0,  true,  false},  // A0    Ch.5     TIM2_CH1   Y     3.3V   RCOut Ch.1
    {1,   PA1,  true,  false},  // A1    Ch.6     TIM2_CH2   Y     3.3V   RCOut Ch.2; loopback output
    {4,   PA4,  true,  false},  // A4    Ch.9     —          Y     3.3V   ADC only
    {8,   PA8,  false, true },  // A8    —        TIM1_CH1   Y     3.3V   RCOut Ch.3
    {9,   PA9,  false, true },  // A9    —        TIM1_CH2   Y     3.3V   RCOut Ch.4
    {10,  PA10, false, true },  // A10   —        TIM1_CH3   Y     3.3V   RCOut Ch.5
    {11,  PA11, false, true },  // A11   —        TIM1_CH4   Y     3.3V   RCOut Ch.6
    {100, PB0,  true,  true },  // B0    Ch.15    —          Y     3.3V   BATT_VOLT in AP_Periph
    {101, PB1,  true,  true },  // B1    Ch.16    —          Y     3.3V   BATT_CURR in AP_Periph
};
static const int PIN_MAP_COUNT = (int)(sizeof(pin_map) / sizeof(pin_map[0]));

// Returns the Arduino pin for param_val, or PIN_NOT_FOUND if:
//   - param_val is negative (disabled)
//   - param_val is not in the map (unknown value)
//   - param_val appears more than once in the map (duplicate/corrupt map entry)
//   - adc_only=true and the pin has no ADC channel
//   - interrupt_only=true and the pin is not interrupt-capable
// Cross-parameter conflicts (same pin assigned to two inputs) are detected
// separately in refresh_pin_params().
static uint32_t resolve_pin(int param_val, bool adc_only = false, bool interrupt_only = false)
{
    if (param_val < 0)
        return PIN_NOT_FOUND;
    uint32_t found = PIN_NOT_FOUND;
    for (int i = 0; i < PIN_MAP_COUNT; i++) {
        if (pin_map[i].param_val == param_val) {
            if (found != PIN_NOT_FOUND)
                return PIN_NOT_FOUND; // duplicate entry → conflict
            if (adc_only && !pin_map[i].adc_capable)
                return PIN_NOT_FOUND; // pin has no ADC channel
            if (interrupt_only && !pin_map[i].interrupt_capable)
                return PIN_NOT_FOUND; // pin not suitable for interrupt input
            found = pin_map[i].arduino_pin;
        }
    }
    return found;
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------
std::vector<DroneCAN::parameter> custom_parameters = {
    {"NODEID",     DroneCAN::INT,  5,      0,          127       },
    {"BAUD",       DroneCAN::INT,  500,    500,        1000      }, // requires restart
    {"FUELINJ_PIN",    DroneCAN::INT,  8,      -1,         101       }, // <0=disabled
    {"BCAST_MS",   DroneCAN::INT,  10,     10,         5000      },
    {"RPM_ALPHA",  DroneCAN::REAL, 0.2f,   0.01f,      1.0f      },
    {"RPM_A1",     DroneCAN::REAL, 1.0f,   -10000.0f,  10000.0f  },
    {"RPM_A0",     DroneCAN::REAL, 0.0f,   -100000.0f, 100000.0f },
    {"RPM_MAX",    DroneCAN::INT,  8000,   100,        20000     }, // max measurable RPM; requires restart
    {"RPM_HOLD_MS",DroneCAN::INT,  500,    100,        5000      }, // ms no signal -> hold RPM + error flag
    {"RPM_STOP_MS",DroneCAN::INT,  3000,   500,        30000     }, // ms no signal -> zero RPM, STOPPED
    {"RPM_ACCEL",  DroneCAN::REAL, 5000.0f, 100.0f,   100000.0f }, // max plausible RPM/s; plausibility filter
    {"RPM_K_WIN",  DroneCAN::INT,  5,      2,          10        }, // sliding window size; requires restart
    {"RPM_K_ACC",  DroneCAN::INT,  3,      1,          10        }, // passes in window required to update EMA
    {"DUTY_ALPHA", DroneCAN::REAL, 0.3f,   0.01f,      1.0f      },
    {"DUTY_A1",    DroneCAN::REAL, 1.0f,   -10000.0f,  10000.0f  },
    {"DUTY_A0",    DroneCAN::REAL, 0.0f,   -100000.0f, 100000.0f },
    {"THR_SRC",    DroneCAN::INT,  1,      0,          2         }, // 0=disabled 1=ADC 2=I2C/INA228; requires restart
    {"THR_I2C_ADDR", DroneCAN::INT, 0x40, 0,          127       }, // INA228 I2C addr when THR_SRC=2; requires restart
    {"THR_PIN",    DroneCAN::INT,  -1,     -1,         101       }, // ADC pin when THR_SRC=1; requires restart
    {"THR_ALPHA",  DroneCAN::REAL, 1.0f,   0.01f,      1.0f      },
    {"THR_A1",     DroneCAN::REAL, 1.0f,   -10000.0f,  10000.0f  },
    {"THR_A0",     DroneCAN::REAL, 0.0f,   -100000.0f, 100000.0f },
    {"CLT_SRC",    DroneCAN::INT,  2,      0,          2         }, // 0=disabled 1=ADC 2=I2C/INA228; requires restart
    {"CLT_I2C_ADDR", DroneCAN::INT, 0x40, 0,          127       }, // INA228 I2C addr when CLT_SRC=2; requires restart
    {"CLT_PIN",    DroneCAN::INT,  -1,     -1,         101       }, // ADC pin when CLT_SRC=1; requires restart
    {"CLT_ALPHA",  DroneCAN::REAL, 1.0f,   0.01f,      1.0f      },
    {"CLT_A1",     DroneCAN::REAL, 1.0f,   -10000.0f,  10000.0f  },
    {"CLT_A0",     DroneCAN::REAL, 0.0f,   -100000.0f, 100000.0f },
    // Temperature sensor 0: <0 pin = disabled
    {"TMP0_PIN",   DroneCAN::INT,  -1,     -1,         101       }, // 0=A0 1=A1 4=A4 100=B0 101=B1
    {"TMP0_ID",    DroneCAN::INT,  0,      0,          65535     },
    {"TMP0_BCAST", DroneCAN::INT,  1000,   10,         5000      },
    {"TMP0_ALPHA", DroneCAN::REAL, 1.0f,   0.01f,      1.0f      },
    {"TMP0_A1",    DroneCAN::REAL, 1.0f,   -10000.0f,  10000.0f  },
    {"TMP0_A0",    DroneCAN::REAL, 0.0f,   -100000.0f, 100000.0f },
    // Temperature sensor 1: <0 pin = disabled
    {"TMP1_PIN",   DroneCAN::INT,  -1,     -1,         101       }, // 0=A0 1=A1 4=A4 100=B0 101=B1
    {"TMP1_ID",    DroneCAN::INT,  1,      0,          65535     },
    {"TMP1_BCAST", DroneCAN::INT,  1000,   10,         5000      },
    {"TMP1_ALPHA", DroneCAN::REAL, 1.0f,   0.01f,      1.0f      },
    {"TMP1_A1",    DroneCAN::REAL, 1.0f,   -10000.0f,  10000.0f  },
    {"TMP1_A0",    DroneCAN::REAL, 0.0f,   -100000.0f, 100000.0f },
#ifdef LOOPBACK_TEST
    {"LB_EN",      DroneCAN::INT,  1,      0,          1         }, // 0=off 1=on
    {"LB_DUTY",    DroneCAN::INT,  50,     0,          100       }, // [%]
    {"LB_FREQ",    DroneCAN::INT,  60,     1,          20000     }, // [Hz]
#endif
};

DroneCAN dronecan;

// ---------------------------------------------------------------------------
// FuelInjState – ISR accumulators (volatile) + pin assignment
// Written from fuelinj_isr(); read with interrupts disabled in the broadcast loop.
// ---------------------------------------------------------------------------
#define RPM_K_WIN_MAX  10

struct FuelInjState {
    volatile uint32_t rising_us    = 0; // micros() at the last rising edge
    volatile uint32_t pending_high = 0; // pulse width of the cycle in progress (not yet committed)
    volatile uint32_t period_sum   = 0; // accumulator: sum of committed period_us values
    volatile uint32_t high_sum     = 0; // accumulator: sum of committed high_us values
    volatile uint32_t sample_count = 0; // number of complete cycles committed since last broadcast
    volatile uint32_t last_edge_ms = 0; // millis() at the last edge
    int      pin_param = INT32_MIN;     // sentinel: not yet initialised; triggers attach on first refresh_pin_params()
    uint32_t pin       = PA8;
};
static FuelInjState fuel_inj;

// ---------------------------------------------------------------------------
// RpmFilterState – EMA filter + plausibility sliding window for RPM/duty
// ---------------------------------------------------------------------------
struct RpmFilterState {
    float    filtered      = 0.0f;    // EMA-filtered RPM
    float    duty_filtered = 0.0f;    // EMA-filtered duty cycle
    bool     seeded        = false;   // true after first valid sample; reset on engine stop
    bool     plaus_win[RPM_K_WIN_MAX] = {}; // circular window: true = sample passed plausibility check
    int      plaus_head    = 0;       // next write index
    int      plaus_filled  = 0;       // valid entries so far (0 .. rpm_k_win)
    int      plaus_passes  = 0;       // count of true entries in active window
    float    plaus_ref_rpm = 0.0f;    // RPM of last individually-plausible sample
    uint32_t plaus_ref_ms  = 0;       // millis() of last individually-plausible sample
};
static RpmFilterState rpm;

// ---------------------------------------------------------------------------
// VoltageSensor – shared state for throttle (thr) and coolant temperature (clt)
// src, i2c_addr, and pin are fixed at boot; the rest are updated live.
// ---------------------------------------------------------------------------
struct VoltageSensor {
    int             src       = 0;    // 0=disabled 1=ADC 2=I2C/INA228; fixed at boot
    int             i2c_addr  = 0;    // INA228 I2C address; fixed at boot
    int             pin_param = -1;   // raw parameter value; <0 = disabled; fixed at boot
    uint32_t        pin       = PA0;  // resolved Arduino pin; fixed at boot
    Adafruit_INA228 meter;
    bool            ina228_ok = false;
    float           alpha     = 1.0f;
    float           a1        = 1.0f;
    float           a0        = 0.0f;
    float           filtered  = 0.0f;
    bool            seeded    = false; // reset when pin assignment changes or sensor is lost
};
static VoltageSensor thr; // throttle position
static VoltageSensor clt; // coolant temperature

// ---------------------------------------------------------------------------
// TmpSensor – state for temperature sensor channels (tmp0, tmp1)
// pin is fixed at boot; the rest are updated live.
// ---------------------------------------------------------------------------
struct TmpSensor {
    int      pin_param = -1;   // raw parameter value; <0 = disabled; fixed at boot
    uint32_t pin       = 0;    // resolved Arduino pin; fixed at boot
    int      id        = 0;    // device_id in Temperature message
    uint32_t bcast     = 1000; // broadcast interval ms
    float    alpha     = 1.0f;
    float    a1        = 1.0f;
    float    a0        = 0.0f;
    float    filtered  = 0.0f;
    bool     seeded    = false; // reset when pin assignment changes
    uint32_t loop_time = 0;    // millis() of last broadcast
};
static TmpSensor tmp0; // temperature sensor 0
static TmpSensor tmp1; // temperature sensor 1

// ---------------------------------------------------------------------------
// Cached scalar parameters – refreshed at 1 Hz from dronecan.getParameter()
// ---------------------------------------------------------------------------
static uint32_t bcast_ms      = 10;
static float    rpm_alpha     = 1.0f;
static float    rpm_a1        = 1.0f;
static float    rpm_a0        = 0.0f;
static uint32_t rpm_hold_ms   = 500;
static uint32_t rpm_stop_ms   = 3000;
static float    rpm_max_accel = 5000.0f;
static int      rpm_k_win     = 5;    // set from RPM_K_WIN at boot; requires restart
static int      rpm_k_acc     = 3;
static float    duty_alpha    = 1.0f;
static float    duty_a1       = 1.0f;
static float    duty_a0       = 0.0f;

// ---------------------------------------------------------------------------
// Broadcast timers
// ---------------------------------------------------------------------------
static uint32_t loop_bcast = 0;
static uint32_t loop_1hz   = 0;

// ---------------------------------------------------------------------------
// ISR
// ---------------------------------------------------------------------------
static void fuelinj_isr()
{
    const uint32_t now_us = micros();
    fuel_inj.last_edge_ms = millis();

    if (digitalRead(fuel_inj.pin) == HIGH) {
        // Rising edge: commit the just-completed cycle as a matched pair into the
        // accumulators, but only if we have a valid pulse width for that cycle.
        if (fuel_inj.rising_us != 0 && fuel_inj.pending_high != 0) {
            const uint32_t period = now_us - fuel_inj.rising_us;
            if (period >= min_period_us && period <= MAX_PERIOD_US) {
                fuel_inj.period_sum   += period;
                fuel_inj.high_sum     += fuel_inj.pending_high;
                fuel_inj.sample_count++;
            }
            fuel_inj.pending_high = 0;
        }
        fuel_inj.rising_us = now_us;
    } else {
        // Falling edge: record pulse width of the cycle in progress
        if (fuel_inj.rising_us != 0) {
            fuel_inj.pending_high = now_us - fuel_inj.rising_us;
        }
    }
}

// Reset ISR state and attach interrupt to fuel_inj.pin (caller must set fuel_inj.pin first).
static void attach_fuelinj_pin()
{
    noInterrupts();
    fuel_inj.rising_us    = 0;
    fuel_inj.pending_high = 0;
    fuel_inj.period_sum   = 0;
    fuel_inj.high_sum     = 0;
    fuel_inj.sample_count = 0;
    fuel_inj.last_edge_ms = 0;
    interrupts();
    pinMode(fuel_inj.pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(fuel_inj.pin), fuelinj_isr, CHANGE);
}

// ---------------------------------------------------------------------------
// Attempt to (re-)initialise an INA228 sensor. Sets vs.ina228_ok.
// Wire.begin() must already have been called.
// ---------------------------------------------------------------------------
static void probe_ina228(VoltageSensor& vs)
{
    vs.ina228_ok = vs.meter.begin((uint8_t)vs.i2c_addr);
    if (vs.ina228_ok) {
        vs.meter.setAveragingCount(INA228_COUNT_16);
        vs.meter.setVoltageConversionTime(INA228_TIME_1052_us);
        vs.meter.setMode(INA228_MODE_CONT_BUS);
    }
}

// ---------------------------------------------------------------------------
// Resolve all pin parameters, check for conflicts, and apply.
// Called once from setup(); pin assignments are fixed for the lifetime of the
// session (any change requires a reboot).
// ---------------------------------------------------------------------------
static void refresh_pin_params()
{
    static const char *param_names[] = {
        "FUELINJ_PIN", "THR_PIN", "CLT_PIN", "TMP0_PIN", "TMP1_PIN"
    };
    static const int NPIN_PARAMS = (int)(sizeof(param_names) / sizeof(param_names[0]));
    int      param_vals[NPIN_PARAMS];
    uint32_t pins[NPIN_PARAMS];

    // --- Step 1: resolve; write back -1 for any unrecognised positive value ---
    // FUELINJ_PIN (index 0) needs interrupt capability; the rest need ADC.
    for (int i = 0; i < NPIN_PARAMS; i++) {
        param_vals[i] = (int)dronecan.getParameter(param_names[i]);
        pins[i]       = resolve_pin(param_vals[i], /*adc_only=*/ i > 0, /*interrupt_only=*/ i == 0);
        if (param_vals[i] >= 0 && pins[i] == PIN_NOT_FOUND) {
            dronecan.setParameter(param_names[i], -1);
            param_vals[i] = -1;
        }
    }

    // THR_PIN (index 1) is only used when THR_SRC=1; mask it out otherwise to
    // prevent spurious conflicts with CLT_PIN or TMP pins.
    if (thr.src != 1) {
        param_vals[1] = -1;
        pins[1]       = PIN_NOT_FOUND;
    }

    // CLT_PIN (index 2) is only used when CLT_SRC=1; mask it out otherwise to
    // prevent spurious conflicts with THR_PIN or TMP pins.
    if (clt.src != 1) {
        param_vals[2] = -1;
        pins[2]       = PIN_NOT_FOUND;
    }

    // --- Step 2: conflict check; write back -1 for both conflicting pins ---
    for (int i = 0; i < NPIN_PARAMS; i++) {
        if (pins[i] == PIN_NOT_FOUND) continue;
        for (int j = i + 1; j < NPIN_PARAMS; j++) {
            if (pins[j] == PIN_NOT_FOUND) continue;
            if (pins[i] == pins[j]) {
                dronecan.setParameter(param_names[i], -1);
                dronecan.setParameter(param_names[j], -1);
                param_vals[i] = param_vals[j] = -1;
                pins[i]       = pins[j]       = PIN_NOT_FOUND;
            }
        }
    }

#ifdef LOOPBACK_TEST
    // LOOPBACK_PIN is driven as PWM output when LB_EN=1; disable any input assigned to it
    if ((int)dronecan.getParameter("LB_EN")) {
        for (int i = 0; i < NPIN_PARAMS; i++) {
            if (pins[i] == LOOPBACK_PIN) {
                dronecan.setParameter(param_names[i], -1);
                param_vals[i] = -1;
                pins[i]       = PIN_NOT_FOUND;
            }
        }
    }
#endif

    // --- Step 3: apply pin changes ---

    // FUELINJ_PIN: detach/reattach interrupt if changed
    if (param_vals[0] != fuel_inj.pin_param) {
        if (fuel_inj.pin_param >= 0)
            detachInterrupt(digitalPinToInterrupt(fuel_inj.pin));
        fuel_inj.pin_param = param_vals[0];
        if (fuel_inj.pin_param >= 0) {
            fuel_inj.pin = pins[0];
            attach_fuelinj_pin();
        }
    }

    // Analog pins: store actual param value (mirroring fuel_inj.pin_param convention).
    // Reset the EMA seed flag whenever the pin assignment changes so the first
    // reading from the new pin is used directly rather than blended from stale state.
    if (param_vals[1] != thr.pin_param)  thr.seeded = false;
    thr.pin_param = param_vals[1];
    if (pins[1] != PIN_NOT_FOUND) { thr.pin = pins[1]; pinMode(thr.pin, INPUT_ANALOG); }

    if (param_vals[2] != clt.pin_param)  clt.seeded = false;
    clt.pin_param = param_vals[2];
    if (pins[2] != PIN_NOT_FOUND) { clt.pin = pins[2]; pinMode(clt.pin, INPUT_ANALOG); }

    if (param_vals[3] != tmp0.pin_param) tmp0.seeded = false;
    tmp0.pin_param = param_vals[3];
    if (pins[3] != PIN_NOT_FOUND) { tmp0.pin = pins[3]; pinMode(tmp0.pin, INPUT_ANALOG); }

    if (param_vals[4] != tmp1.pin_param) tmp1.seeded = false;
    tmp1.pin_param = param_vals[4];
    if (pins[4] != PIN_NOT_FOUND) { tmp1.pin = pins[4]; pinMode(tmp1.pin, INPUT_ANALOG); }

#ifdef LOOPBACK_TEST
    if ((int)dronecan.getParameter("LB_EN")) {
        analogWriteFrequency((int)dronecan.getParameter("LB_FREQ"));
        analogWrite(LOOPBACK_PIN, (int)(dronecan.getParameter("LB_DUTY") * 255.0f / 100.0f));
    }
    // LB_EN=0: leave pin unconfigured (high-impedance INPUT)
#endif
}

static void refresh_params()
{
    bcast_ms      = (uint32_t)constrain((int)dronecan.getParameter("BCAST_MS"), 10, 5000);
    rpm_alpha     = dronecan.getParameter("RPM_ALPHA");
    rpm_a1        = dronecan.getParameter("RPM_A1");
    rpm_a0        = dronecan.getParameter("RPM_A0");
    rpm_hold_ms   = (uint32_t)constrain((int)dronecan.getParameter("RPM_HOLD_MS"), 100, 5000);
    rpm_stop_ms   = (uint32_t)constrain((int)dronecan.getParameter("RPM_STOP_MS"), 500, 30000);
    rpm_max_accel = dronecan.getParameter("RPM_ACCEL");
    rpm_k_acc     = (int)constrain((int)dronecan.getParameter("RPM_K_ACC"), 1, rpm_k_win);
    duty_alpha    = dronecan.getParameter("DUTY_ALPHA");
    duty_a1       = dronecan.getParameter("DUTY_A1");
    duty_a0       = dronecan.getParameter("DUTY_A0");
    thr.alpha     = dronecan.getParameter("THR_ALPHA");
    thr.a1        = dronecan.getParameter("THR_A1");
    thr.a0        = dronecan.getParameter("THR_A0");
    clt.alpha     = dronecan.getParameter("CLT_ALPHA");
    clt.a1        = dronecan.getParameter("CLT_A1");
    clt.a0        = dronecan.getParameter("CLT_A0");
    tmp0.alpha    = dronecan.getParameter("TMP0_ALPHA");
    tmp0.a1       = dronecan.getParameter("TMP0_A1");
    tmp0.a0       = dronecan.getParameter("TMP0_A0");
    tmp0.id       = (int)dronecan.getParameter("TMP0_ID");
    tmp0.bcast    = (uint32_t)constrain((int)dronecan.getParameter("TMP0_BCAST"), 10, 5000);
    tmp1.alpha    = dronecan.getParameter("TMP1_ALPHA");
    tmp1.a1       = dronecan.getParameter("TMP1_A1");
    tmp1.a0       = dronecan.getParameter("TMP1_A0");
    tmp1.id       = (int)dronecan.getParameter("TMP1_ID");
    tmp1.bcast    = (uint32_t)constrain((int)dronecan.getParameter("TMP1_BCAST"), 10, 5000);
}

void setup()
{
    app_setup();
    IWatchdog.begin(2000000);
    Serial.begin(115200);
    analogReadResolution(12); // 12-bit ADC: raw 0–4095 matches the 3.3/4095 conversion
    dronecan.init(custom_parameters, "BR-ICEMonitor");
    refresh_params();
    // RPM_MAX and RPM_K_WIN are intentionally NOT inside refresh_params().
    // refresh_params() is also called every 1 Hz in the loop, so anything
    // placed there is re-applied live.  These two parameters require a restart:
    //   min_period_us is read inside fuelinj_isr() — changing it live while
    //   the ISR is running would produce inconsistent period filtering mid-flight.
    //   rpm_k_win sets the circular window size; changing it live would corrupt
    //   the plaus_head/plaus_filled/plaus_passes counters for the new size.
    // Reading them once here at boot keeps them stable for the session lifetime.
    {
        const int rpm_max = (int)dronecan.getParameter("RPM_MAX");
        min_period_us = (rpm_max > 0) ? (120000000u / (uint32_t)rpm_max) : 15000u;
        rpm_k_win = (int)constrain((int)dronecan.getParameter("RPM_K_WIN"), 2, RPM_K_WIN_MAX);
        rpm_k_acc = (int)constrain(rpm_k_acc, 1, rpm_k_win); // re-clamp after rpm_k_win is known
    }

    thr.src      = (int)dronecan.getParameter("THR_SRC");
    thr.i2c_addr = (int)dronecan.getParameter("THR_I2C_ADDR");
    clt.src      = (int)dronecan.getParameter("CLT_SRC");
    clt.i2c_addr = (int)dronecan.getParameter("CLT_I2C_ADDR");
    // Two INA228s on the same bus must have different addresses.
    // If both are configured as I2C with the same address, disable both at runtime;
    // the user must set different addresses (via THR_I2C_ADDR / CLT_I2C_ADDR) and restart.
    if (thr.src == 2 && clt.src == 2 &&
        thr.i2c_addr > 0 && clt.i2c_addr > 0 &&
        thr.i2c_addr == clt.i2c_addr) {
        dronecan.debug("THR/CLT INA228 addr conflict", 0);
        thr.src = 0;
        clt.src = 0;
    }
    if ((thr.src == 2 && thr.i2c_addr > 0) || (clt.src == 2 && clt.i2c_addr > 0))
        Wire.begin();
    if (thr.src == 2 && thr.i2c_addr > 0) {
        probe_ina228(thr);
        if (!thr.ina228_ok)
            dronecan.debug("THR INA228 not found", 0);
    }
    if (clt.src == 2 && clt.i2c_addr > 0) {
        probe_ina228(clt);
        if (!clt.ina228_ok)
            dronecan.debug("CLT INA228 not found", 0);
    }

    refresh_pin_params();
}

void loop()
{
    const uint32_t now = millis();

    // -----------------------------------------------------------------------
    // ICE Status broadcast: at least one source must be configured
    // -----------------------------------------------------------------------
    const bool thr_configured = (thr.src == 2 && thr.i2c_addr > 0) ||
                                 (thr.src == 1 && thr.pin_param >= 0);
    const bool clt_configured = (clt.src == 2 && clt.i2c_addr > 0) ||
                                 (clt.src == 1 && clt.pin_param >= 0);
    if ((fuel_inj.pin_param >= 0 || thr_configured || clt_configured) && (now - loop_bcast >= bcast_ms)) {
        loop_bcast = now;

        uavcan_equipment_ice_reciprocating_Status pkt{};

        // --- Injection (RPM + duty cycle) ---
        if (fuel_inj.pin_param >= 0) {
            // Snapshot and reset accumulators atomically
            noInterrupts();
            const uint32_t period_sum = fuel_inj.period_sum;
            const uint32_t high_sum   = fuel_inj.high_sum;
            const uint32_t count      = fuel_inj.sample_count;
            const uint32_t last_edge  = fuel_inj.last_edge_ms;
            fuel_inj.period_sum   = 0;
            fuel_inj.high_sum     = 0;
            fuel_inj.sample_count = 0;
            interrupts();

            // Two-tier stale detection.
            // The injection signal is absent during fuel cut-off even though the
            // engine is still spinning.  Two timeouts let the consumer distinguish
            // "engine running, data temporarily unavailable" from "engine stopped":
            //   no_signal < RPM_HOLD_MS  : RUNNING  – fresh data, normal operation
            //   no_signal < RPM_STOP_MS  : RUNNING  – hold last RPM; CRANKSHAFT_SENSOR_ERROR set
            //   no_signal >= RPM_STOP_MS : STOPPED  – zero RPM; engine truly stopped
            const int32_t  elapsed_i    = (last_edge == 0) ? INT32_MAX
                                                            : (int32_t)(now - last_edge);
            const uint32_t no_signal_ms = (elapsed_i < 0) ? 0u : (uint32_t)elapsed_i;
            const bool stopped = (last_edge == 0) || (no_signal_ms >= rpm_stop_ms);
            const bool held    = !stopped && (no_signal_ms >= rpm_hold_ms);
            const bool running = !stopped;

            if (stopped) {
                // Engine truly stopped: zero outputs and reset all filter state
                rpm.filtered      = 0.0f;
                rpm.duty_filtered = 0.0f;
                rpm.seeded        = false;
                rpm.plaus_head    = 0;
                rpm.plaus_filled  = 0;
                rpm.plaus_passes  = 0;
                rpm.plaus_ref_ms  = 0;
                rpm.plaus_ref_rpm = 0.0f;
            } else if (!held && count > 0) {
                // Fresh signal: apply plausibility filter before updating EMA.
                //
                // Each sample's implied RPM/s change is compared against RPM_ACCEL.
                // This rejects: noise/bounce edges within the period bounds, skip-
                // injection (every-other-cycle strategy implying an instant RPM/2
                // change far above any physical limit), and anomalous post-cut-off
                // first samples whose period spans the entire cut duration.
                //
                // A sliding window of RPM_K_WIN results is maintained.  The EMA is
                // updated only when RPM_K_ACC of the last RPM_K_WIN samples pass AND
                // the current sample itself passes, so a rejected sample never feeds
                // the EMA regardless of window history.  When fewer than RPM_K_ACC
                // pass, the EMA is frozen at the last accepted value.
                //
                // Note – BCAST_MS dependency:
                //   The plausibility check operates on the mean period of all cycles
                //   committed within one broadcast window (period_sum / count).  If
                //   BCAST_MS is large enough that a normal and a skip/spurious cycle
                //   both land in the same window, the anomaly is diluted in the
                //   average and may appear less extreme to the filter.  At the default
                //   BCAST_MS=10 ms and RPM ≤ 8000 (period ≥ 15 ms) at most one cycle
                //   lands per window, so mixing does not occur in practice.
                const float mean_period = (float)period_sum / (float)count;
                const float mean_high   = (float)high_sum   / (float)count;
                const float rpm_raw     = 120000000.0f / mean_period;
                const float duty_raw    = 100.0f * mean_high / mean_period;

                bool plaus_pass;
                if (rpm.plaus_ref_ms == 0) {
                    // No reference yet (first sample after start/stop): accept unconditionally
                    plaus_pass = true;
                } else {
                    const float dt_s  = (float)(now - rpm.plaus_ref_ms) * 0.001f;
                    const float accel = fabsf(rpm_raw - rpm.plaus_ref_rpm) / fmaxf(dt_s, 0.001f);
                    plaus_pass = (accel <= rpm_max_accel);
                }
                if (plaus_pass) {
                    rpm.plaus_ref_rpm = rpm_raw;
                    rpm.plaus_ref_ms  = now;
                }

                // Update circular sliding window
                if (rpm.plaus_filled < rpm_k_win) {
                    // Window filling: append without overwriting
                    rpm.plaus_win[rpm.plaus_head] = plaus_pass;
                    if (plaus_pass) rpm.plaus_passes++;
                    rpm.plaus_head = (rpm.plaus_head + 1) % rpm_k_win;
                    rpm.plaus_filled++;
                } else {
                    // Window full: overwrite oldest entry
                    if (rpm.plaus_win[rpm.plaus_head]) rpm.plaus_passes--;
                    rpm.plaus_win[rpm.plaus_head] = plaus_pass;
                    if (plaus_pass)                rpm.plaus_passes++;
                    rpm.plaus_head = (rpm.plaus_head + 1) % rpm_k_win;
                }

                // Commit to EMA when the current sample individually passes AND
                // the window confirms enough recent passes.  A rejected sample
                // must never feed the EMA even if old True entries remain in
                // the window (prevents skip-injection / spurious edges from
                // contaminating the EMA while the window is still draining).
                if (plaus_pass && rpm.plaus_passes >= rpm_k_acc) {
                    if (!rpm.seeded) {
                        // Seed with first accepted sample to avoid ramping up from 0
                        rpm.filtered      = rpm_raw;
                        rpm.duty_filtered = duty_raw;
                        rpm.seeded        = true;
                    } else {
                        rpm.filtered      = rpm_alpha  * rpm_raw  + (1.0f - rpm_alpha)  * rpm.filtered;
                        rpm.duty_filtered = duty_alpha * duty_raw + (1.0f - duty_alpha) * rpm.duty_filtered;
                    }
                }
                // else: plausibility window not passing threshold — freeze EMA
            }
            // held || count==0: EMA frozen, last value held

            pkt.state = running
                ? UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_STATE_RUNNING
                : UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_STATE_STOPPED;
            pkt.engine_speed_rpm            = (uint32_t)constrain(rpm_a1 * rpm.filtered + rpm_a0, 0.0f, 131071.0f);
            pkt.fuel_consumption_rate_cm3pm = duty_a1 * rpm.duty_filtered + duty_a0;

            // FLAG_CRANKSHAFT_SENSOR_ERROR: set when RPM data is held (no fresh signal)
            // or when the plausibility filter is actively rejecting samples.
            const bool plaus_rejecting = !held
                                      && (rpm.plaus_filled >= rpm_k_win)
                                      && (rpm.plaus_passes  < rpm_k_acc);
            const bool crank_error = running && (held || plaus_rejecting);
            pkt.flags =
                UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED
                | (crank_error
                    ? UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_FLAG_CRANKSHAFT_SENSOR_ERROR
                    : 0u);

            //Serial.print(running ? "RUNNING" : "STOPPED");
            //Serial.print("  RPM: "); Serial.print(pkt.engine_speed_rpm);
            //Serial.print("  Duty: "); Serial.print(pkt.fuel_consumption_rate_cm3pm, 1); Serial.print(" %");
        }

        // --- Throttle position ---
        {
            bool  thr_valid   = false;
            float thr_voltage = 0.0f;
            if (thr.src == 2 && thr.ina228_ok) {
                thr_voltage = thr.meter.getBusVoltage_V();
                thr_valid   = true;
            } else if (thr.src == 1 && thr.pin_param >= 0) {
                thr_voltage = analogRead(thr.pin) * (3.3f / 4095.0f);
                thr_valid   = true;
            }
            if (thr_valid) {
                if (!thr.seeded) { thr.filtered = thr_voltage; thr.seeded = true; }
                else thr.filtered = thr.alpha * thr_voltage + (1.0f - thr.alpha) * thr.filtered;
                const float thr_pct = thr.a1 * thr.filtered + thr.a0;
                pkt.throttle_position_percent = (uint8_t)constrain(roundf(thr_pct), 0.0f, 100.0f);
                //Serial.print("  Thr: "); Serial.print(thr_pct, 1); Serial.print(" %");
            } else {
                pkt.throttle_position_percent = 0x7F;  // uint7 has no invalid sentinel; 127 is used to identify the invalid value.
            }
        }

        // --- Coolant temperature ---
        {
            bool  clt_valid   = false;
            float clt_voltage = 0.0f;
            if (clt.src == 2 && clt.ina228_ok) {
                clt_voltage = clt.meter.getBusVoltage_V();
                clt_valid   = true;
            } else if (clt.src == 1 && clt.pin_param >= 0) {
                clt_voltage = analogRead(clt.pin) * (3.3f / 4095.0f);
                clt_valid   = true;
            }
            if (clt_valid) {
                if (!clt.seeded) { clt.filtered = clt_voltage; clt.seeded = true; }
                else clt.filtered = clt.alpha * clt_voltage + (1.0f - clt.alpha) * clt.filtered;
                pkt.coolant_temperature = clt.a1 * clt.filtered + clt.a0;
                //Serial.print("  CLT: "); Serial.print(pkt.coolant_temperature, 1);
            }
            else
            {
                pkt.coolant_temperature = -273.15;  // Separate from unavailable.
            }
        }

        //Serial.println();
        sendUavcanMsg(dronecan.canard, pkt);
    }

    // -----------------------------------------------------------------------
    // Temperature sensor 0 broadcast (when TMP0_PIN >= 0)
    // -----------------------------------------------------------------------
    if (tmp0.pin_param >= 0 && (now - tmp0.loop_time >= tmp0.bcast)) {
        tmp0.loop_time = now;

        const float voltage = analogRead(tmp0.pin) * (3.3f / 4095.0f);
        if (!tmp0.seeded) { tmp0.filtered = voltage; tmp0.seeded = true; }
        else tmp0.filtered = tmp0.alpha * voltage + (1.0f - tmp0.alpha) * tmp0.filtered;
        const float temp0 = tmp0.a1 * tmp0.filtered + tmp0.a0;

        uavcan_equipment_device_Temperature pkt{};
        pkt.device_id   = (uint16_t)tmp0.id;
        pkt.temperature = temp0;
        pkt.error_flags = 0;
        sendUavcanMsg(dronecan.canard, pkt);

        //Serial.print("TMP0: ");
        //Serial.print(temp0, 2);
        //Serial.println(" (A1*V+A0)");
    }

    // -----------------------------------------------------------------------
    // Temperature sensor 1 broadcast (when TMP1_PIN >= 0)
    // -----------------------------------------------------------------------
    if (tmp1.pin_param >= 0 && (now - tmp1.loop_time >= tmp1.bcast)) {
        tmp1.loop_time = now;

        const float voltage = analogRead(tmp1.pin) * (3.3f / 4095.0f);
        if (!tmp1.seeded) { tmp1.filtered = voltage; tmp1.seeded = true; }
        else tmp1.filtered = tmp1.alpha * voltage + (1.0f - tmp1.alpha) * tmp1.filtered;
        const float temp1 = tmp1.a1 * tmp1.filtered + tmp1.a0;

        uavcan_equipment_device_Temperature pkt{};
        pkt.device_id   = (uint16_t)tmp1.id;
        pkt.temperature = temp1;
        pkt.error_flags = 0;
        sendUavcanMsg(dronecan.canard, pkt);

        //Serial.print("TMP1: ");
        //Serial.print(temp1, 2);
        //Serial.println(" (A1*V+A0)");
    }

    // -----------------------------------------------------------------------
    // 1 Hz: refresh scalar parameters (EMA alphas, scale/offset, intervals).
    // Pin assignments and signal sources are fixed at boot; no pin refresh here.
    // -----------------------------------------------------------------------
    if (now - loop_1hz >= 1000) {
        loop_1hz = now;

        refresh_params();

        // INA228 health-check and retry
        if (thr.src == 2 && thr.i2c_addr > 0) {
            if (!thr.ina228_ok) {
                probe_ina228(thr);
                if (thr.ina228_ok)
                    dronecan.debug("THR INA228 found", 0);
            } else if (!thr.meter.conversionReady()) {
                // conversionReady() reads DIAG_ALRT/CNVR — see CLT block comment below.
                thr.ina228_ok = false;
                thr.seeded    = false;
                dronecan.debug("THR INA228 lost", 0);
            }
        }
        if (clt.src == 2 && clt.i2c_addr > 0) {
            if (!clt.ina228_ok) {
                // Sensor was missing at boot or lost: retry
                probe_ina228(clt);
                if (clt.ina228_ok)
                    dronecan.debug("CLT INA228 found", 0);
            } else if (!clt.meter.conversionReady()) {
                // How this check works:
                //   conversionReady() reads the DIAG_ALRT register and tests the CNVR bit.
                //   The INA228 hardware sets CNVR on every completed conversion (~17 ms in
                //   CONT_BUS mode with 16 averages at 1052 µs each).  CNVR is cleared only
                //   when DIAG_ALRT is read — getBusVoltage_V() reads only the VBUS register
                //   and does NOT clear it.  So after one second of normal operation, CNVR is
                //   guaranteed to be set on a live sensor.  If conversionReady() returns false
                //   here, it means the I2C read returned 0 (NACK), i.e. the sensor is gone.
                //   Detection latency is up to ~1 s (one tick of the 1 Hz block).
                clt.ina228_ok = false;
                clt.seeded    = false;
                dronecan.debug("CLT INA228 lost", 0);
            }
        }

    }

    dronecan.cycle();
    IWatchdog.reload();
}
