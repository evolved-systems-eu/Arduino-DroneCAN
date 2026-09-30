PWM_Duty_Meas – Pin Verification Notes (STM32L431, CAN Node)
=============================================================

Reference: DS_stm32l431rb.pdf (Table 15 pin definitions, Table 16/17 alternate functions)
Board variant: ~/.platformio/platforms/br-stm32/variants/MicroNode/variant_MICRONODE.h


PIN TABLE
---------

pwm_pin_table[] = { PA8, PA9, PA10, PB0, PB1 }

  Index | Pin  | Arduino# | EXTI line | Key alternate functions
  ------|------|----------|-----------|-------------------------------------------
    0   | PA8  |    8     | EXTI8     | TIM1_CH1, MCO, USART1_CK, SAI1_SCK_A
    1   | PA9  |    9     | EXTI9     | TIM1_CH2, USART1_TX, I2C1_SCL, SAI1_FS_A
    2   | PA10 |   10     | EXTI10    | TIM1_CH3, USART1_RX, I2C1_SDA, SAI1_SD_A
    3   | PB0  |  PIN_A8  | EXTI0     | TIM1_CH2N, ADC1_IN15, USART3_CK
    4   | PB1  |  PIN_A9  | EXTI1     | TIM1_CH3N, ADC1_IN16, USART3_RTS_DE

  PIN_A8 and PIN_A9 resolve to Arduino pin numbers 46 and 47 on this board
  (analog-only pins start at NUM_DIGITAL_PINS = 38).
  All pins are I/O FT (5 V tolerant) on STM32L431.


ALL PINS SUPPORT EXTERNAL INTERRUPTS
-------------------------------------

All GPIO pins on STM32L431 can generate EXTI interrupts.
The five EXTI lines used above (0, 1, 8, 9, 10) are all distinct – no conflict
between pins within this table.

EXTI line sharing rule: PA8, PB8, PC8, ... all share EXTI8 – only one can be
active at a time. Since the code detaches before re-attaching, this is safe.
However, if the user application uses another interrupt on the same EXTI line
(e.g. a button on PB8 while PA8 is selected), there will be a conflict.


SERIAL DOES NOT CONFLICT WITH PA9 / PA10
-----------------------------------------

PA9 = USART1_TX and PA10 = USART1_RX per the datasheet. This initially looks
like a conflict with Serial.begin(). However, the board variant configures:

    SERIAL_UART_INSTANCE = 101   (LPUART, not USART1)
    PIN_SERIAL_TX        = PA2
    PIN_SERIAL_RX        = PA3

So Serial uses LPUART on PA2/PA3. PA9 and PA10 are free for GPIO use.

Other serial instances on this board:
    Serial1  – TX=PB6,  RX=PB7   (USART1)
    Serial3  – TX=PB10, RX=PB11  (USART3)


CAN DOES NOT CONFLICT WITH ANY TABLE PIN
-----------------------------------------

CAN1 alternate functions on STM32L431:
    CAN1_RX = PA11 (AF9)  or  PB8 (AF9)
    CAN1_TX = PA12 (AF9)  or  PB9 (AF9)

None of these overlap with PA8, PA9, PA10, PB0, or PB1.


PIN STORAGE TYPE
-----------------

pwm_pin_table and active_pin use uint32_t (not uint8_t).
Rationale: STM32duino Arduino pin API (pinMode, digitalRead, attachInterrupt,
digitalPinToInterrupt) all take uint32_t. PB0 = PIN_A8 = 46 and PB1 = PIN_A9 = 47
fit in uint8_t numerically, but using uint32_t avoids implicit truncation and
matches the API type correctly.


INPUT VOLTAGE LIMITS
---------------------

All five pins are I/O FT type (5 V tolerant) per datasheet Table 15.

  Max input voltage : 5.5 V
      From Table 22: VIN_max = min(VDD + 3.6V, 5.5V) = min(6.9V, 5.5V) = 5.5V
  Min input voltage : -0.3 V (absolute)
  Min logic HIGH (VIH) : ~2.31 V  (0.7 x VDD = 0.7 x 3.3V)
  Max logic LOW  (VIL) : ~0.99 V  (0.3 x VDD = 0.3 x 3.3V)

  3.3 V and 5 V PWM signals work directly.
  1.8 V logic does NOT reliably meet VIH and requires a pull-up or level shifter.
  Never exceed 5.5 V on any pin.

  Note: PB0 and PB1 are FT_a (FT + analog switch tied to VDDA). When used as
  digital interrupt inputs (our case) they behave as standard FT GPIO. If the
  ADC is also active on these pins at the same time, limit input to VDDA (3.3V).


PWM FREQUENCY LIMITS
---------------------

The software ISR approach (used here) has two constraints:

1. ISR execution time – upper frequency bound
   Each edge fires the ISR. At 80 MHz the ISR (micros + millis + digitalRead +
   comparison) takes ~30-50 CPU cycles plus ~12-15 cycles interrupt latency,
   totalling ~600-800 ns per edge. With CAN traffic also generating interrupts
   and occasionally delaying entry, the practical safe upper limit is:

       ~20 kHz maximum recommended

   Above this, missed edges and corrupted period/duty readings become likely.

2. micros() resolution – accuracy vs. frequency
   micros() on STM32 Arduino has ~1 µs resolution. Measurement error grows
   proportionally at higher frequencies:

     PWM frequency | Period  | Resolution error
     --------------|---------|------------------
     1 kHz         | 1000 us | < 0.1 %
     10 kHz        |  100 us | ~1 %
     20 kHz        |   50 us | ~2 %
     50 kHz        |   20 us | ~5 %  (unreliable)

   Best accuracy is achieved below 10 kHz.

3. Minimum frequency
   The stale timeout in the code is 500 ms. The signal needs at least one full
   rising-falling-rising cycle within that window:

       ~2 Hz minimum

   Increase the 500 ms constant in main.cpp if slower signals are needed.


WHY PWM_PIN IS AN INTEGER, NOT A STRING
-----------------------------------------

DroneCAN::STRING is declared as a type constant in dronecan.h but is not
implemented in the library. In dronecan.cpp, handle_param_GetSet() only handles
INTEGER, REAL, and BOOLEAN — a string value falls to the default branch and is
discarded (valid = false), so the parameter is never updated:

    switch (req.value.union_tag) {
    case INTEGER: set_value = req.value.integer_value; break;
    case REAL:    set_value = req.value.real_value;    break;
    case BOOLEAN: set_value = ...;                     break;
    default:      valid = false;   // string lands here
    }

getParameter() also returns float only — there is no way to read a string back.

A string parameter ("PA8", "PA9", ...) would appear in the DroneCAN GUI but
could never be written or read in firmware. The port*100+pin integer encoding
(8=PA8, 9=PA9, 10=PA10, 100=PB0, 101=PB1) is the best option within the
library's current capabilities.

If the library is later updated to support string parameters, PWM_PIN could be
changed to DroneCAN::STRING with values "PA8", "PA9", etc.


HARDWARE INPUT CAPTURE (future improvement)
---------------------------------------------

PA8-PA10 are TIM1_CH1/CH2/CH3. These support hardware input capture mode, which
would give more accurate timing than the software ISR approach used here.
For most applications (PWM frequencies below ~20 kHz) the ISR approach is
sufficient. Consider hardware capture if higher accuracy or higher frequency
measurement is needed. Hardware capture on TIM1 can handle up to ~40 MHz.
