ICEMonitor – Fuel Injection Signal Monitor
==========================================

Monitors a fuel injection signal from a Delphi MT05 ECU, extracts RPM and
injection duty cycle, and broadcasts the results over DroneCAN as
uavcan.equipment.ice.reciprocating.Status.


INPUT SIGNAL
------------

Source : Delphi MT05 ECU fuel injector output
Logic  : Active HIGH = fuel is being injected
           Rising edge  → injection starts
           Falling edge → injection ends
Engine : Single cylinder, 4-stroke
           → 1 injection per 2 crankshaft revolutions

Signal characteristics:
  - Idle RPM range  : ~1000–3000 RPM → period 40–120 ms
  - Max RPM range   : up to ~10000 RPM → period ~12 ms
  - Pulse width     : typically 2–20 ms depending on load
  - Logic level     : 3.3 V or 5 V (all measurement pins are 5 V tolerant)


RPM CALCULATION
---------------

The ISR timestamps each rising edge. The period between two consecutive
rising edges equals one full injection cycle = 2 crankshaft revolutions.

    period_us = time between two consecutive rising edges [µs]

    RPM = 2 revolutions / period_seconds * 60 s/min
        = 120 / period_seconds
        = 120,000,000 / period_us

Example: period_us = 20,000 µs → RPM = 120,000,000 / 20,000 = 6000 RPM

Signal stale timeout: 2000 ms (covers down to ~60 RPM engine speed, at which
point period = 2,000,000 µs = 2 s). If no rising edge is seen within 2000 ms,
the engine is considered stopped: RPM = 0, state = STATE_STOPPED.


DUTY CYCLE CALCULATION
----------------------

    high_us   = time from rising edge to falling edge [µs]
    period_us = time between two consecutive rising edges [µs]

    duty_pct = 100 × high_us / period_us   [%]

This represents the injection pulse width as a fraction of the total cycle.
It is placed in fuel_consumption_rate_cm3pm (repurposed field; see message
mapping below).


EMA FILTER
----------

An exponential moving average (EMA) is applied independently to RPM and duty:

    filtered[n] = alpha × raw[n] + (1 − alpha) × filtered[n−1]

    alpha = 1.0  → no filtering, raw value passes through (default)
    alpha → 0.0  → heavy smoothing, slow to respond
    Typical value: 0.1 – 0.5 for a smooth but responsive signal

Parameters: RPM_ALPHA and DUTY_ALPHA (see parameter table below).


LINEAR TRANSFORM
----------------

After filtering, a linear transform is applied for calibration or unit conversion:

    engine_speed_rpm          = RPM_A1  × RPM_filtered  + RPM_A0
    fuel_consumption_rate     = DUTY_A1 × duty_filtered + DUTY_A0

Defaults: A1 = 1.0, A0 = 0.0 (passthrough). Adjust if the ECU signal needs
scaling (e.g. gear ratio correction) or offset.


PARAMETER TABLE
---------------

  Name       | Type | Default | Min      | Max      | Description
  -----------|------|---------|----------|----------|----------------------------------
  NODEID     | INT  | 100     | 0        | 127      | DroneCAN node ID
  BAUD       | INT  | 1000    | 500      | 1000     | CAN baudrate in kbps (see note)
  INJ_PIN    | INT  | 8       | 8        | 101      | Injection input pin (see pin table)
  BCAST_MS   | INT  | 100     | 10       | 5000     | Broadcast interval [ms]
  RPM_ALPHA  | REAL | 1.0     | 0.01     | 1.0      | EMA alpha for RPM
  RPM_A1     | REAL | 1.0     | -10000.0 | 10000.0  | RPM linear scale
  RPM_A0     | REAL | 0.0     | -100000.0| 100000.0 | RPM linear offset
  DUTY_ALPHA | REAL | 1.0     | 0.01     | 1.0      | EMA alpha for duty cycle
  DUTY_A1    | REAL | 1.0     | -10000.0 | 10000.0  | Duty cycle linear scale
  DUTY_A0    | REAL | 0.0     | -100000.0| 100000.0 | Duty cycle linear offset

INJ_PIN encoding: port × 100 + pin_number  (port A = 0, port B = 1)
  See PIN TABLE section below. Same scheme as examples/PWM_Duty_Meas.

BAUD note: see CAN BAUDRATE section below.


PIN TABLE (INJ_PIN)
-------------------

  INJ_PIN | STM32 pin | Arduino# | EXTI line | Notes
  --------|-----------|----------|-----------|---------------------------
     8    | PA8       |    8     | EXTI8     | TIM1_CH1
     9    | PA9       |    9     | EXTI9     | TIM1_CH2
    10    | PA10      |   10     | EXTI10    | TIM1_CH3
   100    | PB0       |  PIN_A8  | EXTI0     | ADC1_IN15
   101    | PB1       |  PIN_A9  | EXTI1     | ADC1_IN16

All pins are 5 V tolerant (FT type). See examples/PWM_Duty_Meas/README.txt
for full pin verification notes.


DRONECAN MESSAGE MAPPING
------------------------

Message: uavcan.equipment.ice.reciprocating.Status  (ID 1120)

  Field                          | Value
  -------------------------------|----------------------------------------------
  state                          | STATE_RUNNING (2) when signal active
                                 | STATE_STOPPED (0) when stale (no signal)
  flags                          | 0
  engine_load_percent            | 0
  engine_speed_rpm               | RPM after EMA filter and linear transform
  fuel_consumption_rate_cm3pm    | Duty cycle [%] after EMA filter and transform
  spark_dwell_time_ms            | 0.0
  atmospheric_pressure_kpa       | 0.0
  intake_manifold_pressure_kpa   | 0.0
  intake_manifold_temperature    | 0.0
  coolant_temperature            | 0.0
  oil_pressure                   | 0.0
  oil_temperature                | 0.0
  fuel_pressure                  | 0.0
  estimated_consumed_fuel_cm3    | 0.0
  throttle_position_percent      | 0
  ecu_index                      | 0
  spark_plug_usage               | SPARK_PLUG_SINGLE (0)
  cylinder_status.len            | 0  (no per-cylinder data)

Note: fuel_consumption_rate_cm3pm is repurposed to carry injection duty cycle
in percent. This is intentional; configure the receiving autopilot driver
accordingly.

Note: engine_speed_rpm is a 17-bit field in the wire format (max 131071 RPM).


CAN BAUDRATE (BAUD parameter)
------------------------------

The library (libArduinoDroneCAN v2.0.0) hardcodes 1 Mbps in dronecan.cpp:

    return CANInit(CAN_1000KBPS, 2);  // dronecan.cpp

The underlying canL431.h defines CAN_500KBPS and CAN_1000KBPS as valid enum
values, so the hardware supports 500 kbps. Two options to implement runtime
selection:

  Option A (modify library): change dronecan.cpp to read the BAUD parameter
    from flash before calling CANInit, and pass CAN_500KBPS or CAN_1000KBPS
    accordingly. Requires forking libArduinoDroneCAN.

  Option B (pre-init workaround): call CANInit(CAN_500KBPS, 2) directly in
    the application before dronecan.init(). The library will then re-call
    CANInit inside init() overwriting it with 1 Mbps — this does NOT work
    without Option A.

Status: BAUD parameter is included in the parameter table and stored in flash,
but runtime selection is not implemented in v1. The node always runs at 1 Mbps.
Implement Option A when 500 kbps support is required.

Changing BAUD always requires a node restart to take effect regardless of the
implementation chosen.


TEST FEATURE (LOOPBACK_TEST)
-----------------------------

Define LOOPBACK_TEST to generate a PWM signal on PA1 (TIM2_CH2) and loop it
back to the injection input pin for self-test without real ECU hardware.

When enabled, three extra DroneCAN parameters appear:

  LB_EN    [INT, 0–1]       : 0 = PWM output off, 1 = PWM output on (default 0)
  LB_DUTY  [INT, 0–100]     : loopback signal duty cycle [%]  (default 50)
  LB_FREQ  [INT, 1–20000]   : loopback signal frequency [Hz]  (default 1000)

All three can be changed live from the DroneCAN GUI without reflashing.
LB_EN = 0 stops the PWM output immediately (analogWrite duty set to 0).
LB_DUTY and LB_FREQ are updated at 1 Hz but only take effect when LB_EN = 1.
Wire: PA1 → INJ_PIN (default PA8).

PA1 is chosen because:
  - It is a dedicated PWM output pin on the CAN Node header (TIM2_CH2)
  - It does not conflict with CAN (PA11/PA12), Serial (PA2/PA3), or any
    measurement pin (PA8–PA10, PB0, PB1)
  - PA0 was avoided due to a reported connectivity issue on some carrier boards

Remove the #define LOOPBACK_TEST and the LB_DUTY / LB_FREQ parameters when
deploying in production.


EXTENSIBILITY
-------------

The codebase is structured so additional measurements can be added with minimal
changes:

  1. New interrupt-based sensor: add its own volatile state variables and ISR,
     attach in setup(), read atomically in the loop.

  2. New DroneCAN message: construct and send alongside the existing ICE Status
     broadcast in the loop.

  3. New parameters: append to custom_parameters[] — they are automatically
     handled by the DroneCAN GetSet protocol and persisted to flash.

Fields already present in uavcan_equipment_ice_reciprocating_Status that could
be populated in future versions:
  - coolant_temperature       (NTC sensor via ADC)
  - oil_pressure              (pressure sensor via ADC)
  - throttle_position_percent (potentiometer via ADC)
  - atmospheric_pressure_kpa  (barometric sensor via I2C/SPI)
