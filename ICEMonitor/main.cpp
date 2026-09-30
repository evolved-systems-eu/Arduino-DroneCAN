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
 * Parameters
 * ----------
 *   NODEID     – DroneCAN node ID (default 100)
 *   BAUD       – CAN baudrate kbps: 500 or 1000 (default 1000, requires restart)
 *   INJ_PIN    – injection input pin, port×100+pin (default 8 = PA8)
 *                  8=PA8  9=PA9  10=PA10  100=PB0  101=PB1
 *   BCAST_MS   – broadcast interval ms (default 100)
 *   RPM_ALPHA  – EMA alpha for RPM, 0.01–1.0 (default 1.0 = no filtering)
 *   RPM_A1     – RPM linear scale  (default 1.0)
 *   RPM_A0     – RPM linear offset (default 0.0)
 *   DUTY_ALPHA – EMA alpha for duty cycle (default 1.0)
 *   DUTY_A1    – duty cycle linear scale  (default 1.0)
 *   DUTY_A0    – duty cycle linear offset (default 0.0)
 *
 * See ICEMonitor/README.txt for full specification.
 */

#include <Arduino.h>
#include <dronecan.h>

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

// ---------------------------------------------------------------------------
// Pin map: INJ_PIN parameter value → Arduino pin number
// Encoding: port×100 + pin_number  (port A=0, port B=1)
// ---------------------------------------------------------------------------
struct PinMapEntry {
    int      param_val;
    uint32_t arduino_pin;
};

static const PinMapEntry pin_map[] = {
    {8,   PA8},
    {9,   PA9},
    {10,  PA10},
    {100, PB0},
    {101, PB1},
};
static const int PIN_MAP_COUNT = (int)(sizeof(pin_map) / sizeof(pin_map[0]));

static uint32_t resolve_pin(int param_val)
{
    for (int i = 0; i < PIN_MAP_COUNT; i++) {
        if (pin_map[i].param_val == param_val)
            return pin_map[i].arduino_pin;
    }
    return PA8; // invalid value → fall back to default
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------
std::vector<DroneCAN::parameter> custom_parameters = {
    {"NODEID",     DroneCAN::INT,  5,      0,          127       },
    {"BAUD",       DroneCAN::INT,  500,    500,        1000      }, // requires restart
    {"INJ_PIN",    DroneCAN::INT,  8,      8,          101       }, // 8=PA8 9=PA9 10=PA10 100=PB0 101=PB1
    {"BCAST_MS",   DroneCAN::INT,  1000,   10,         5000      },
    {"RPM_ALPHA",  DroneCAN::REAL, 0.2f,   0.01f,      1.0f      },
    {"RPM_A1",     DroneCAN::REAL, 1.0f,   -10000.0f,  10000.0f  },
    {"RPM_A0",     DroneCAN::REAL, 0.0f,   -100000.0f, 100000.0f },
    {"DUTY_ALPHA", DroneCAN::REAL, 0.3f,   0.01f,      1.0f      },
    {"DUTY_A1",    DroneCAN::REAL, 1.0f,   -10000.0f,  10000.0f  },
    {"DUTY_A0",    DroneCAN::REAL, 0.0f,   -100000.0f, 100000.0f },
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
static volatile uint32_t inj_rising_us    = 0; // micros() at the last rising edge
static volatile uint32_t inj_high_us      = 0; // injection pulse width (µs)
static volatile uint32_t inj_period_us    = 0; // period, rising-to-rising (µs)
static volatile uint32_t inj_last_edge_ms = 0; // millis() at the last edge

static uint32_t active_pin = PA8;

static void inj_isr()
{
    const uint32_t now_us = micros();
    inj_last_edge_ms = millis();

    if (digitalRead(active_pin) == HIGH) {
        // Rising edge: record period from previous rising edge, then latch start
        if (inj_rising_us != 0) {
            inj_period_us = now_us - inj_rising_us;
        }
        inj_rising_us = now_us;
    } else {
        // Falling edge: record injection pulse width
        if (inj_rising_us != 0) {
            inj_high_us = now_us - inj_rising_us;
        }
    }
}

// Reset ISR state and attach interrupt to new pin.
static void attach_inj_pin(uint32_t pin)
{
    active_pin = pin;
    noInterrupts();
    inj_rising_us    = 0;
    inj_high_us      = 0;
    inj_period_us    = 0;
    inj_last_edge_ms = 0;
    interrupts();
    pinMode(active_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(active_pin), inj_isr, CHANGE);
}

// ---------------------------------------------------------------------------
// EMA filter state – persists between broadcast cycles
// ---------------------------------------------------------------------------
static float rpm_filtered  = 0.0f;
static float duty_filtered = 0.0f;

// ---------------------------------------------------------------------------
// Cached parameters – refreshed at 1 Hz from dronecan.getParameter()
// ---------------------------------------------------------------------------
static uint32_t bcast_ms   = 100;
static float    rpm_alpha  = 1.0f;
static float    rpm_a1     = 1.0f;
static float    rpm_a0     = 0.0f;
static float    duty_alpha = 1.0f;
static float    duty_a1    = 1.0f;
static float    duty_a0    = 0.0f;

// ---------------------------------------------------------------------------

static uint32_t loop_bcast = 0;
static uint32_t loop_1hz   = 0;

void setup()
{
    app_setup();
    IWatchdog.begin(2000000);
    Serial.begin(115200);
    dronecan.init(custom_parameters, "BR-ICEMonitor");

    // Activate the injection input pin saved in parameters
    attach_inj_pin(resolve_pin((int)dronecan.getParameter("INJ_PIN")));

#ifdef LOOPBACK_TEST
    analogWriteFrequency((int)dronecan.getParameter("LB_FREQ"));
    analogWrite(LOOPBACK_PIN,
                (int)dronecan.getParameter("LB_EN")
                    ? (int)(dronecan.getParameter("LB_DUTY") * 255.0f / 100.0f)
                    : 0);
#endif
}

void loop()
{
    const uint32_t now = millis();

    // -----------------------------------------------------------------------
    // Broadcast at the rate set by BCAST_MS
    // -----------------------------------------------------------------------
    if (now - loop_bcast >= bcast_ms) {
        loop_bcast = now;

        // Snapshot ISR state atomically
        noInterrupts();
        const uint32_t high_us   = inj_high_us;
        const uint32_t period_us = inj_period_us;
        const uint32_t last_edge = inj_last_edge_ms;
        interrupts();

        // Stale check: no rising edge within 2000 ms → engine considered stopped
        // 2000 ms covers down to ~60 RPM (period between injections = 2 s)
        // Require high_us > 0 so the EMA is never fed a partial first-cycle
        // sample where period_us is set but high_us has not been updated yet.
        const bool running = (last_edge != 0) && ((now - last_edge) < 2000)
                             && (period_us > 0) && (high_us > 0);

        if (running) {
            // Raw measurements
            const float rpm_raw  = 120000000.0f / (float)period_us;
            const float duty_raw = 100.0f * (float)high_us / (float)period_us;

            // EMA filter
            rpm_filtered  = rpm_alpha  * rpm_raw  + (1.0f - rpm_alpha)  * rpm_filtered;
            duty_filtered = duty_alpha * duty_raw + (1.0f - duty_alpha) * duty_filtered;
        } else {
            // Reset filters so they don't ramp up slowly when the engine restarts
            rpm_filtered  = 0.0f;
            duty_filtered = 0.0f;
        }

        // Linear transform
        const float rpm_out  = rpm_a1  * rpm_filtered  + rpm_a0;
        const float duty_out = duty_a1 * duty_filtered + duty_a0;

        // Build DroneCAN message
        uavcan_equipment_ice_reciprocating_Status pkt{};
        pkt.state = running
            ? UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_STATE_RUNNING
            : UAVCAN_EQUIPMENT_ICE_RECIPROCATING_STATUS_STATE_STOPPED;
        pkt.flags                       = 0;
        pkt.engine_load_percent         = 0;
        pkt.engine_speed_rpm            = (uint32_t)constrain(rpm_out, 0.0f, 131071.0f);
        pkt.fuel_consumption_rate_cm3pm = duty_out; // injection duty cycle [%]

        sendUavcanMsg(dronecan.canard, pkt);

        Serial.print(running ? "RUNNING" : "STOPPED");
        Serial.print("  RPM: ");
        Serial.print(pkt.engine_speed_rpm);
        Serial.print("  Duty: ");
        Serial.print(duty_out, 1);
        Serial.println(" %");
    }

    // -----------------------------------------------------------------------
    // 1 Hz: refresh cached parameters and handle INJ_PIN / loopback changes
    // -----------------------------------------------------------------------
    if (now - loop_1hz >= 1000) {
        loop_1hz = now;

        bcast_ms   = (uint32_t)constrain((int)dronecan.getParameter("BCAST_MS"), 10, 5000);
        rpm_alpha  = dronecan.getParameter("RPM_ALPHA");
        rpm_a1     = dronecan.getParameter("RPM_A1");
        rpm_a0     = dronecan.getParameter("RPM_A0");
        duty_alpha = dronecan.getParameter("DUTY_ALPHA");
        duty_a1    = dronecan.getParameter("DUTY_A1");
        duty_a0    = dronecan.getParameter("DUTY_A0");

        const uint32_t new_pin = resolve_pin((int)dronecan.getParameter("INJ_PIN"));
        if (new_pin != active_pin) {
            detachInterrupt(digitalPinToInterrupt(active_pin));
            attach_inj_pin(new_pin);
        }

#ifdef LOOPBACK_TEST
        const int lb_en = (int)dronecan.getParameter("LB_EN");
        analogWriteFrequency((int)dronecan.getParameter("LB_FREQ"));
        analogWrite(LOOPBACK_PIN,
                    lb_en ? (int)(dronecan.getParameter("LB_DUTY") * 255.0f / 100.0f)
                           : 0);
#endif
    }

    dronecan.cycle();
    IWatchdog.reload();
}
