/*
 * PWM Duty Cycle Measurement
 *
 * Measures the duty cycle of a PWM signal on a selectable GPIO pin using
 * hardware interrupts, then broadcasts the result as a DroneCAN
 * uavcan.equipment.device.Temperature message at 10 Hz.
 *
 * The `temperature` field carries the duty cycle in percent (0.0 – 100.0).
 * This is an intentional repurposing of the message type; the receiving
 * ArduPilot/PX4 driver should be configured accordingly (e.g. via a
 * TEMP_SENSx_TYPE that maps device_id to a duty-cycle input).
 *
 * Parameters
 * ----------
 *   NODEID    – DroneCAN node ID (default 100)
 *   DEVICE_ID – device_id field in the Temperature message (default 0)
 *   PWM_PIN   – pin encoded as port*100 + pin_number (default 8 = PA8)
 *                 8   → PA8
 *                 9   → PA9
 *                 10  → PA10
 *                 100 → PB0
 *                 101 → PB1
 *
 * All listed pins support external interrupts on the Micro Node / CAN Node
 * family (STM32L431) and the CAN Node Plus (STM32H723).
 */

#include <Arduino.h>
#include <dronecan.h>

#define LOOPBACK_TEST


// ---------------------------------------------------------------------------
// Pin map: PWM_PIN parameter value → Arduino pin number
// Encoding: port*100 + pin_number  (port A=0, port B=1)
//   8   → PA8  (0*100 + 8)
//   9   → PA9
//   10  → PA10
//   100 → PB0  (1*100 + 0)
//   101 → PB1
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

// Returns the Arduino pin for a given parameter value, or PA8 if invalid.
static uint32_t resolve_pin(int param_val)
{
    for (int i = 0; i < PIN_MAP_COUNT; i++) {
        if (pin_map[i].param_val == param_val)
            return pin_map[i].arduino_pin;
    }
    return PA8; // invalid value → fall back to default
}

std::vector<DroneCAN::parameter> custom_parameters = {
    {"NODEID",    DroneCAN::INT, 100, 0,     127},
    {"DEVICE_ID", DroneCAN::INT,   0, 0,     127},
    {"PWM_PIN",   DroneCAN::INT,   8, 8,     101}, // 8=PA8 9=PA9 10=PA10 100=PB0 101=PB1
#ifdef LOOPBACK_TEST
    {"LB_DUTY",   DroneCAN::INT,  50, 0,     100}, // loopback duty cycle [%]
    {"LB_FREQ",   DroneCAN::INT, 1000, 1, 20000}, // loopback frequency [Hz]
#endif
};

DroneCAN dronecan;

// ---------------------------------------------------------------------------
// ISR state – written only from the ISR, read with interrupts disabled
// ---------------------------------------------------------------------------
static volatile uint32_t pwm_rising_us    = 0; // micros() at the last rising edge
static volatile uint32_t pwm_high_us      = 0; // duration of the last HIGH pulse (µs)
static volatile uint32_t pwm_period_us    = 0; // last full period, rising-to-rising (µs)
static volatile uint32_t pwm_last_edge_ms = 0; // millis() at the last edge (stale detection)

static uint32_t active_pin = PA8;

static void pwm_isr()
{
    const uint32_t now_us = micros();
    pwm_last_edge_ms = millis();

    if (digitalRead(active_pin) == HIGH) {
        // Rising edge: record period from previous rising edge, then latch start
        if (pwm_rising_us != 0) {
            pwm_period_us = now_us - pwm_rising_us;
        }
        pwm_rising_us = now_us;
    } else {
        // Falling edge: record high-pulse duration
        if (pwm_rising_us != 0) {
            pwm_high_us = now_us - pwm_rising_us;
        }
    }
}

// Detach existing interrupt (if any), reset state, and attach to new pin.
static void attach_pwm_pin(uint32_t pin)
{
    active_pin = pin;
    noInterrupts();
    pwm_rising_us    = 0;
    pwm_high_us      = 0;
    pwm_period_us    = 0;
    pwm_last_edge_ms = 0;
    interrupts();
    pinMode(active_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(active_pin), pwm_isr, CHANGE);
}

// ---------------------------------------------------------------------------
// Loopback test – define LOOPBACK_TEST to generate a PWM signal on PA1 and
// wire PA1 → measurement pin (default PA8) to verify the measurement.
// LB_DUTY [%] and LB_FREQ [Hz] are exposed as DroneCAN parameters so they
// can be changed live from the GUI without reflashing.
// Remove this block (and the #ifdef sections below) when testing is done.
// ---------------------------------------------------------------------------
#ifdef LOOPBACK_TEST
  #define LOOPBACK_PIN  PA1   // output pin – wire this to the measurement pin
#endif
// ---------------------------------------------------------------------------

static uint32_t loop_10hz = 0;
static uint32_t loop_1hz  = 0;
static int      device_id = 0;

void setup()
{
    app_setup();
    IWatchdog.begin(2000000);
    Serial.begin(115200);
    dronecan.init(custom_parameters, "BR-Node-PWMDuty");

    // Activate the pin saved in parameters (may differ from default after first boot)
    attach_pwm_pin(resolve_pin((int)dronecan.getParameter("PWM_PIN")));

#ifdef LOOPBACK_TEST
    analogWriteFrequency((int)dronecan.getParameter("LB_FREQ"));
    analogWrite(LOOPBACK_PIN, (int)(dronecan.getParameter("LB_DUTY") * 255.0f / 100.0f));
#endif
}

void loop()
{
    const uint32_t now = millis();

    // -----------------------------------------------------------------------
    // 10 Hz: compute duty cycle and broadcast Temperature message
    // -----------------------------------------------------------------------
    if (now - loop_10hz > 100) {
        loop_10hz = millis();

        // Snapshot ISR state atomically
        noInterrupts();
        const uint32_t high_us   = pwm_high_us;
        const uint32_t period_us = pwm_period_us;
        const uint32_t last_edge = pwm_last_edge_ms;
        interrupts();

        // No edge in the last 500 ms → signal absent, report 0 %
        float duty_pct = 0.0f;
        if ((last_edge != 0) && ((now - last_edge) < 500) && (period_us > 0)) {
            duty_pct = 100.0f * (float)high_us / (float)period_us;
            duty_pct = constrain(duty_pct, 0.0f, 100.0f);
        }

        uavcan_equipment_device_Temperature pkt{};
        pkt.device_id   = (uint16_t)device_id;
        pkt.temperature = duty_pct; // duty cycle [%] carried in the temperature field
        sendUavcanMsg(dronecan.canard, pkt);

        Serial.print("PWM duty: ");
        Serial.print(duty_pct, 1);
        Serial.println(" %");
    }

    // -----------------------------------------------------------------------
    // 1 Hz: refresh parameters and reconfigure input pin if PWM_PIN changed
    // -----------------------------------------------------------------------
    if (now - loop_1hz > 1000) {
        loop_1hz = millis();

        device_id = (int)dronecan.getParameter("DEVICE_ID");

        const uint32_t new_pin = resolve_pin((int)dronecan.getParameter("PWM_PIN"));
        if (new_pin != active_pin) {
            detachInterrupt(digitalPinToInterrupt(active_pin));
            attach_pwm_pin(new_pin);
        }

#ifdef LOOPBACK_TEST
        analogWriteFrequency((int)dronecan.getParameter("LB_FREQ"));
        analogWrite(LOOPBACK_PIN, (int)(dronecan.getParameter("LB_DUTY") * 255.0f / 100.0f));
#endif
    }

    dronecan.cycle();
    IWatchdog.reload();
}
