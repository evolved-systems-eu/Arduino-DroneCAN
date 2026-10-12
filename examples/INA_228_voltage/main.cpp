#include <Arduino.h>
#include <dronecan.h>
#include <Adafruit_INA228.h>
#include <uavcan.protocol.debug.KeyValue.h>
#include <Wire.h>
#include <string.h>

// uavcan.protocol.debug.KeyValue is not pre-registered in the library.
REGISTER_UAVCAN_TYPE(uavcan_protocol_debug_KeyValue,
    uavcan_protocol_debug_KeyValue_encode,
    UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID,
    UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE,
    UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MAX_SIZE);

std::vector<DroneCAN::parameter> custom_parameters = {
    {"NODEID", DroneCAN::INT, 100, 0, 127},
};

DroneCAN dronecan;
Adafruit_INA228 meter;

uint32_t looptime = 0;

void setup()
{
    app_setup();
    IWatchdog.begin(2000000);
    Serial.begin(115200);
    dronecan.init(custom_parameters, "BR-INA228-Voltage");

    Wire.begin();
    if (!meter.begin(0x40))
    {
        uint32_t deadloop = 0;
        while (1)
        {
            const uint32_t now = millis();
            if (now - deadloop > 1000)
            {
                deadloop = millis();
                dronecan.debug("INA228 not found", 0);
                Serial.println("Sensor not found. Check wiring!");
            }
            dronecan.cycle();
            IWatchdog.reload();
        }
    }

    meter.setAveragingCount(INA228_COUNT_16);
    meter.setVoltageConversionTime(INA228_TIME_1052_us);
    meter.setMode(INA228_MODE_CONT_BUS);
}

void loop()
{
    const uint32_t now = millis();

    if (now - looptime > 200)
    {
        looptime = millis();

        uavcan_protocol_debug_KeyValue pkt{};
        pkt.value = meter.getBusVoltage_V();
        const char label[] = "voltage_V";
        pkt.key.len = sizeof(label) - 1;
        memcpy(pkt.key.data, label, pkt.key.len);
        sendUavcanMsg(dronecan, pkt, CANARD_TRANSFER_PRIORITY_LOW);
    }

    dronecan.cycle();
    IWatchdog.reload();
}
