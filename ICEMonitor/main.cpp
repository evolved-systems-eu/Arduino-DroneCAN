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
 * Pin encoding (shared by all PIN parameters)
 * --------------------------------------------
 *   Negative value  → input disabled / not connected
 *   0=A0  1=A1  4=A4  8=A8  9=A9  10=A10  11=A11  100=B0  101=B1
 *   ADC-capable: 0 1 4 100 101    Interrupt-capable (FUELINJ_PIN): 8 9 10 11 100 101
 *
 * ICE Status (uavcan.equipment.ice.reciprocating.Status)
 * -------------------------------------------------------
 *   Broadcast when at least one source is configured: FUELINJ_PIN ≥ 0,
 *   THR_PIN ≥ 0, CLT_SRC=2 with a valid I2C address, or CLT_SRC=1 with
 *   CLT_PIN ≥ 0. Because CLT_SRC defaults to 2 and CLT_I2C_ADDR defaults
 *   to 0x40, the message is broadcast by default even without FUELINJ_PIN
 *   or THR_PIN.
 *
 *   NODEID     – DroneCAN node ID (default 5)
 *   BAUD       – CAN baudrate kbps: 500 or 1000 (default 500; stored in flash, effective after restart)
 *   FUELINJ_PIN    – injection input pin (default 8 = A8); <0 = disabled
 *   BCAST_MS   – ICE Status broadcast interval ms (default 10)
 *   RPM_ALPHA  – EMA alpha for RPM, 0.01–1.0 (default 0.2)
 *   RPM_A1     – RPM linear scale  (default 1.0)
 *   RPM_A0     – RPM linear offset (default 0.0)
 *   DUTY_ALPHA – EMA alpha for duty cycle (default 0.3)
 *   DUTY_A1    – duty cycle linear scale  (default 1.0)
 *   DUTY_A0    – duty cycle linear offset (default 0.0)
 *   THR_PIN    – throttle analog input pin (default -1 = disabled)
 *   THR_ALPHA  – EMA alpha for throttle (default 1.0)
 *   THR_A1     – linear scale:  throttle_pct = A1×voltage + A0 (default 1.0)
 *   THR_A0     – linear offset (default 0.0)
 *   CLT_SRC    – coolant temp source: 0=disabled 1=ADC 2=I2C/INA228 (default 2; requires restart)
 *   CLT_I2C_ADDR – INA228 I2C address when CLT_SRC=2 (default 0x40; requires restart)
 *   CLT_PIN    – ADC input pin when CLT_SRC=1 (default -1 = disabled; live-updated)
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
#define MIN_PERIOD_US   12000u   //  10,000 RPM upper limit
#define MAX_PERIOD_US  400000u   //     300 RPM lower limit

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
    {"DUTY_ALPHA", DroneCAN::REAL, 0.3f,   0.01f,      1.0f      },
    {"DUTY_A1",    DroneCAN::REAL, 1.0f,   -10000.0f,  10000.0f  },
    {"DUTY_A0",    DroneCAN::REAL, 0.0f,   -100000.0f, 100000.0f },
    {"THR_PIN",    DroneCAN::INT,  -1,     -1,         101       }, // <0=disabled; ADC pin
    {"THR_ALPHA",  DroneCAN::REAL, 1.0f,   0.01f,      1.0f      },
    {"THR_A1",     DroneCAN::REAL, 1.0f,   -10000.0f,  10000.0f  },
    {"THR_A0",     DroneCAN::REAL, 0.0f,   -100000.0f, 100000.0f },
    {"CLT_SRC",    DroneCAN::INT,  2,      0,          2         }, // 0=disabled 1=ADC 2=I2C/INA228; requires restart
    {"CLT_I2C_ADDR", DroneCAN::INT, 0x40, 0,          127       }, // INA228 I2C addr when CLT_SRC=2; requires restart
    {"CLT_PIN",    DroneCAN::INT,  -1,     -1,         101       }, // ADC pin when CLT_SRC=1; live-updated
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
// ISR state – written only from the ISR, read with interrupts disabled
// ---------------------------------------------------------------------------
static volatile uint32_t fuelinj_rising_us    = 0; // micros() at the last rising edge
static volatile uint32_t fuelinj_pending_high = 0; // pulse width of the cycle in progress (not yet committed)
static volatile uint32_t fuelinj_period_sum   = 0; // accumulator: sum of committed period_us values
static volatile uint32_t fuelinj_high_sum     = 0; // accumulator: sum of committed high_us values
static volatile uint32_t fuelinj_sample_count = 0; // number of complete cycles committed since last broadcast
static volatile uint32_t fuelinj_last_edge_ms = 0; // millis() at the last edge

static int      fuelinj_pin_param = INT32_MIN; // sentinel: not yet initialised; triggers attach on first refresh_pin_params()
static uint32_t active_pin    = PA8;

static void fuelinj_isr()
{
    const uint32_t now_us = micros();
    fuelinj_last_edge_ms = millis();

    if (digitalRead(active_pin) == HIGH) {
        // Rising edge: commit the just-completed cycle as a matched pair into the
        // accumulators, but only if we have a valid pulse width for that cycle.
        if (fuelinj_rising_us != 0 && fuelinj_pending_high != 0) {
            const uint32_t period = now_us - fuelinj_rising_us;
            if (period >= MIN_PERIOD_US && period <= MAX_PERIOD_US) {
                fuelinj_period_sum   += period;
                fuelinj_high_sum     += fuelinj_pending_high;
                fuelinj_sample_count++;
            }
            fuelinj_pending_high = 0;
        }
        fuelinj_rising_us = now_us;
    } else {
        // Falling edge: record pulse width of the cycle in progress
        if (fuelinj_rising_us != 0) {
            fuelinj_pending_high = now_us - fuelinj_rising_us;
        }
    }
}

// Reset ISR state and attach interrupt to active_pin (caller must set active_pin first).
static void attach_fuelinj_pin()
{
    noInterrupts();
    fuelinj_rising_us    = 0;
    fuelinj_pending_high = 0;
    fuelinj_period_sum   = 0;
    fuelinj_high_sum     = 0;
    fuelinj_sample_count = 0;
    fuelinj_last_edge_ms = 0;
    interrupts();
    pinMode(active_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(active_pin), fuelinj_isr, CHANGE);
}

// ---------------------------------------------------------------------------
// EMA filter state – persists between broadcast cycles
// ---------------------------------------------------------------------------
static float rpm_filtered   = 0.0f;
static float duty_filtered  = 0.0f;
static bool  fuelinj_seeded = false; // true after first valid sample; reset on engine stop
static float thr_filtered   = 0.0f;
static bool  thr_seeded     = false; // reset when pin assignment changes
static float clt_filtered   = 0.0f;
static bool  clt_seeded     = false; // reset on sensor loss or pin change
static float tmp0_filtered  = 0.0f;
static bool  tmp0_seeded    = false; // reset when pin assignment changes
static float tmp1_filtered  = 0.0f;
static bool  tmp1_seeded    = false; // reset when pin assignment changes

// ---------------------------------------------------------------------------
// Cached parameters – refreshed at 1 Hz from dronecan.getParameter()
// ---------------------------------------------------------------------------
static uint32_t bcast_ms   = 10;
static float    rpm_alpha  = 1.0f;
static float    rpm_a1     = 1.0f;
static float    rpm_a0     = 0.0f;
static float    duty_alpha = 1.0f;
static float    duty_a1    = 1.0f;
static float    duty_a0    = 0.0f;

// Throttle
static int      thr_pin_param = -1;  // raw parameter value; <0 = disabled
static uint32_t thr_pin       = PA0;
static float    thr_alpha     = 1.0f;
static float    thr_a1        = 1.0f;
static float    thr_a0        = 0.0f;

// Coolant temperature
static int      clt_pin_param = -1;  // raw parameter value; <0 = disabled
static uint32_t clt_pin       = PA1;
static float    clt_alpha     = 1.0f;
static float    clt_a1        = 1.0f;
static float    clt_a0        = 0.0f;
static Adafruit_INA228 clt_meter;
static bool            clt_ina228_ok = false;
static int             clt_src       = 2;  // read once at startup; requires reboot to change
static int             clt_i2c_addr  = 0x40;

// Temperature sensor 0
static int      tmp0_pin_param = -1; // raw parameter value; <0 = disabled
static uint32_t tmp0_pin       = PB0;
static int      tmp0_id        = 0;
static uint32_t tmp0_bcast     = 1000;
static float    tmp0_alpha     = 1.0f;
static float    tmp0_a1        = 1.0f;
static float    tmp0_a0        = 0.0f;

// Temperature sensor 1
static int      tmp1_pin_param = -1; // raw parameter value; <0 = disabled
static uint32_t tmp1_pin       = PB1;
static int      tmp1_id        = 1;
static uint32_t tmp1_bcast     = 1000;
static float    tmp1_alpha     = 1.0f;
static float    tmp1_a1        = 1.0f;
static float    tmp1_a0        = 0.0f;

// ---------------------------------------------------------------------------
// last report time.
static uint32_t loop_bcast = 0;
static uint32_t loop_tmp0  = 0;
static uint32_t loop_tmp1  = 0;
static uint32_t loop_1hz   = 0;

// ---------------------------------------------------------------------------
// Resolve all pin parameters, check for conflicts, and apply.
// Called from setup() and the 1 Hz refresh tick.
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

    // CLT_PIN (index 2) is only used when CLT_SRC=1; mask it out otherwise to
    // prevent spurious conflicts with THR_PIN or TMP pins.
    if (clt_src != 1) {
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
    if (param_vals[0] != fuelinj_pin_param) {
        if (fuelinj_pin_param >= 0)
            detachInterrupt(digitalPinToInterrupt(active_pin));
        fuelinj_pin_param = param_vals[0];
        if (fuelinj_pin_param >= 0) {
            active_pin = pins[0];
            attach_fuelinj_pin();
        }
    }

    // Analog pins: store actual param value (mirroring fuelinj_pin_param convention).
    // Reset the EMA seed flag whenever the pin assignment changes so the first
    // reading from the new pin is used directly rather than blended from stale state.
    if (param_vals[1] != thr_pin_param)  thr_seeded  = false;
    thr_pin_param = param_vals[1];
    if (pins[1] != PIN_NOT_FOUND) { thr_pin  = pins[1]; pinMode(thr_pin,  INPUT_ANALOG); }

    if (param_vals[2] != clt_pin_param)  clt_seeded  = false;
    clt_pin_param = param_vals[2];
    if (pins[2] != PIN_NOT_FOUND) { clt_pin  = pins[2]; pinMode(clt_pin,  INPUT_ANALOG); }

    if (param_vals[3] != tmp0_pin_param) tmp0_seeded = false;
    tmp0_pin_param = param_vals[3];
    if (pins[3] != PIN_NOT_FOUND) { tmp0_pin = pins[3]; pinMode(tmp0_pin, INPUT_ANALOG); }

    if (param_vals[4] != tmp1_pin_param) tmp1_seeded = false;
    tmp1_pin_param = param_vals[4];
    if (pins[4] != PIN_NOT_FOUND) { tmp1_pin = pins[4]; pinMode(tmp1_pin, INPUT_ANALOG); }

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
    bcast_ms   = (uint32_t)constrain((int)dronecan.getParameter("BCAST_MS"), 10, 5000);
    rpm_alpha  = dronecan.getParameter("RPM_ALPHA");
    rpm_a1     = dronecan.getParameter("RPM_A1");
    rpm_a0     = dronecan.getParameter("RPM_A0");
    duty_alpha = dronecan.getParameter("DUTY_ALPHA");
    duty_a1    = dronecan.getParameter("DUTY_A1");
    duty_a0    = dronecan.getParameter("DUTY_A0");
    thr_alpha  = dronecan.getParameter("THR_ALPHA");
    thr_a1     = dronecan.getParameter("THR_A1");
    thr_a0     = dronecan.getParameter("THR_A0");
    clt_alpha  = dronecan.getParameter("CLT_ALPHA");
    clt_a1     = dronecan.getParameter("CLT_A1");
    clt_a0     = dronecan.getParameter("CLT_A0");
    tmp0_alpha = dronecan.getParameter("TMP0_ALPHA");
    tmp0_a1    = dronecan.getParameter("TMP0_A1");
    tmp0_a0    = dronecan.getParameter("TMP0_A0");
    tmp0_id    = (int)dronecan.getParameter("TMP0_ID");
    tmp0_bcast = (uint32_t)constrain((int)dronecan.getParameter("TMP0_BCAST"), 10, 5000);
    tmp1_alpha = dronecan.getParameter("TMP1_ALPHA");
    tmp1_a1    = dronecan.getParameter("TMP1_A1");
    tmp1_a0    = dronecan.getParameter("TMP1_A0");
    tmp1_id    = (int)dronecan.getParameter("TMP1_ID");
    tmp1_bcast = (uint32_t)constrain((int)dronecan.getParameter("TMP1_BCAST"), 10, 5000);
}

// Attempt to (re-)initialise the CLT INA228. Sets clt_ina228_ok.
// Wire.begin() must already have been called.
static void probe_clt_ina228()
{
    clt_ina228_ok = clt_meter.begin((uint8_t)clt_i2c_addr);
    if (clt_ina228_ok) {
        clt_meter.setAveragingCount(INA228_COUNT_16);
        clt_meter.setVoltageConversionTime(INA228_TIME_1052_us);
        clt_meter.setMode(INA228_MODE_CONT_BUS);
    }
}

void setup()
{
    app_setup();
    IWatchdog.begin(2000000);
    Serial.begin(115200);
    analogReadResolution(12); // 12-bit ADC: raw 0–4095 matches the 3.3/4095 conversion
    dronecan.init(custom_parameters, "BR-ICEMonitor");
    refresh_params();

    clt_src      = (int)dronecan.getParameter("CLT_SRC");
    clt_i2c_addr = (int)dronecan.getParameter("CLT_I2C_ADDR");
    if (clt_src == 2 && clt_i2c_addr > 0) {
        Wire.begin();
        probe_clt_ina228();
        if (!clt_ina228_ok)
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
    const bool clt_configured = (clt_src == 2 && clt_i2c_addr > 0) ||
                                 (clt_src == 1 && clt_pin_param >= 0);
    if ((fuelinj_pin_param >= 0 || thr_pin_param >= 0 || clt_configured) && (now - loop_bcast >= bcast_ms)) {
        loop_bcast = now;

        uavcan_equipment_ice_reciprocating_Status pkt{};

        // --- Injection (RPM + duty cycle) ---
        if (fuelinj_pin_param >= 0) {
            // Snapshot and reset accumulators atomically
            noInterrupts();
            const uint32_t period_sum   = fuelinj_period_sum;
            const uint32_t high_sum     = fuelinj_high_sum;
            const uint32_t count        = fuelinj_sample_count;
            const uint32_t last_edge    = fuelinj_last_edge_ms;
            fuelinj_period_sum   = 0;
            fuelinj_high_sum     = 0;
            fuelinj_sample_count = 0;
            interrupts();

            // Stale: no edge seen for 2000 ms → engine stopped
            const bool stale   = (last_edge == 0) || ((int32_t)(now - last_edge) >= 2000);
            const bool running = !stale;

            if (stale) {
                // No edge for 2000 ms: engine stopped, reset EMA
                rpm_filtered   = 0.0f;
                duty_filtered  = 0.0f;
                fuelinj_seeded = false;
            } else if (count > 0) {
                const float mean_period = (float)period_sum / (float)count;
                const float mean_high   = (float)high_sum   / (float)count;
                const float rpm_raw  = 120000000.0f / mean_period;
                const float duty_raw = 100.0f * mean_high / mean_period;
                if (!fuelinj_seeded) {
                    // Seed with first sample to avoid ramping up from 0
                    rpm_filtered   = rpm_raw;
                    duty_filtered  = duty_raw;
                    fuelinj_seeded = true;
                } else {
                    rpm_filtered  = rpm_alpha  * rpm_raw  + (1.0f - rpm_alpha)  * rpm_filtered;
                    duty_filtered = duty_alpha * duty_raw + (1.0f - duty_alpha) * duty_filtered;
                }
            }
            // count==0 and not stale: signal active but no complete cycle in this
            // window (broadcast interval shorter than signal period) — keep previous EMA

            pkt.state = running
                ? UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_STATE_RUNNING
                : UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_STATE_STOPPED;
            pkt.engine_speed_rpm            = (uint32_t)constrain(rpm_a1 * rpm_filtered + rpm_a0, 0.0f, 131071.0f);
            pkt.fuel_consumption_rate_cm3pm = duty_a1 * duty_filtered + duty_a0;

            //Serial.print(running ? "RUNNING" : "STOPPED");
            //Serial.print("  RPM: "); Serial.print(pkt.engine_speed_rpm);
            //Serial.print("  Duty: "); Serial.print(pkt.fuel_consumption_rate_cm3pm, 1); Serial.print(" %");
        }

        // --- Throttle position ---
        if (thr_pin_param >= 0) {
            const float voltage = analogRead(thr_pin) * (3.3f / 4095.0f);
            if (!thr_seeded) { thr_filtered = voltage; thr_seeded = true; }
            else thr_filtered = thr_alpha * voltage + (1.0f - thr_alpha) * thr_filtered;
            const float thr_pct = thr_a1 * thr_filtered + thr_a0;
            pkt.throttle_position_percent = (uint8_t)constrain(roundf(thr_pct), 0.0f, 100.0f);

            //Serial.print("  Thr: "); Serial.print(thr_pct, 1); Serial.print(" %");
        }
        else
        {
            pkt.throttle_position_percent = 0x7F;  // uint7 has no invalid sentinel; 127 is used to identify the invalid value.
        }

        // --- Coolant temperature ---
        {
            bool  clt_valid   = false;
            float clt_voltage = 0.0f;
            if (clt_src == 2 && clt_ina228_ok) {
                clt_voltage = clt_meter.getBusVoltage_V();
                clt_valid   = true;
            } else if (clt_src == 1 && clt_pin_param >= 0) {
                clt_voltage = analogRead(clt_pin) * (3.3f / 4095.0f);
                clt_valid   = true;
            }
            if (clt_valid) {
                if (!clt_seeded) { clt_filtered = clt_voltage; clt_seeded = true; }
                else clt_filtered = clt_alpha * clt_voltage + (1.0f - clt_alpha) * clt_filtered;
                pkt.coolant_temperature = clt_a1 * clt_filtered + clt_a0;
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
    if (tmp0_pin_param >= 0 && (now - loop_tmp0 >= tmp0_bcast)) {
        loop_tmp0 = now;

        const float voltage = analogRead(tmp0_pin) * (3.3f / 4095.0f);
        if (!tmp0_seeded) { tmp0_filtered = voltage; tmp0_seeded = true; }
        else tmp0_filtered = tmp0_alpha * voltage + (1.0f - tmp0_alpha) * tmp0_filtered;
        const float temp0 = tmp0_a1 * tmp0_filtered + tmp0_a0;

        uavcan_equipment_device_Temperature pkt{};
        pkt.device_id   = (uint16_t)tmp0_id;
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
    if (tmp1_pin_param >= 0 && (now - loop_tmp1 >= tmp1_bcast)) {
        loop_tmp1 = now;

        const float voltage = analogRead(tmp1_pin) * (3.3f / 4095.0f);
        if (!tmp1_seeded) { tmp1_filtered = voltage; tmp1_seeded = true; }
        else tmp1_filtered = tmp1_alpha * voltage + (1.0f - tmp1_alpha) * tmp1_filtered;
        const float temp1 = tmp1_a1 * tmp1_filtered + tmp1_a0;

        uavcan_equipment_device_Temperature pkt{};
        pkt.device_id   = (uint16_t)tmp1_id;
        pkt.temperature = temp1;
        pkt.error_flags = 0;
        sendUavcanMsg(dronecan.canard, pkt);

        //Serial.print("TMP1: ");
        //Serial.print(temp1, 2);
        //Serial.println(" (A1*V+A0)");
    }

    // -----------------------------------------------------------------------
    // 1 Hz: refresh cached parameters and handle FUELINJ_PIN / loopback changes
    // -----------------------------------------------------------------------
    if (now - loop_1hz >= 1000) {
        loop_1hz = now;

        refresh_params();

        // INA228 health-check and retry
        if (clt_src == 2 && clt_i2c_addr > 0) {
            if (!clt_ina228_ok) {
                // Sensor was missing at boot or lost: retry
                probe_clt_ina228();
                if (clt_ina228_ok)
                    dronecan.debug("CLT INA228 found", 0);
            } else if (!clt_meter.conversionReady()) {
                // How this check works:
                //   conversionReady() reads the DIAG_ALRT register and tests the CNVR bit.
                //   The INA228 hardware sets CNVR on every completed conversion (~17 ms in
                //   CONT_BUS mode with 16 averages at 1052 µs each).  CNVR is cleared only
                //   when DIAG_ALRT is read — getBusVoltage_V() reads only the VBUS register
                //   and does NOT clear it.  So after one second of normal operation, CNVR is
                //   guaranteed to be set on a live sensor.  If conversionReady() returns false
                //   here, it means the I2C read returned 0 (NACK), i.e. the sensor is gone.
                //   Detection latency is up to ~1 s (one tick of the 1 Hz block).
                clt_ina228_ok = false;
                clt_seeded    = false;
                dronecan.debug("CLT INA228 lost", 0);
            }
        }

        refresh_pin_params();
    }

    dronecan.cycle();
    IWatchdog.reload();
}
