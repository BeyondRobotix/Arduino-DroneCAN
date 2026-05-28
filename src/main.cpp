#include <Arduino.h>
#include <dronecan.h>

std::vector<DroneCAN::parameter> params_port1 = {
    {"NODEID", DroneCAN::INT, 100, 1, 127},
    {"PARM_1", DroneCAN::REAL, 0.0f, 0.0f, 100.0f},
    {"PARM_2", DroneCAN::REAL, 0.0f, 0.0f, 100.0f},
};

DroneCAN can1;

uint32_t looptime1 = 0;

void setup()
{
    app_setup(); // This should always be at the start
    IWatchdog.begin(2000000);
    Serial.begin(115200);

    can1.init(params_port1, "Beyond Robotix Node", DroneCAN::CanMode::Classic, DroneCAN::CanPort::PORT1);

    Serial.print("Port1 NODEID: "); Serial.println(can1.getParameter("NODEID"));
}

void loop()
{
    const uint32_t now = millis();

    // PORT1: broadcast BatteryInfo at 10 Hz
    if (now - looptime1 > 100)
    {
        looptime1 = now;

        int32_t vref     = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
        int32_t cpu_temp = __LL_ADC_CALC_TEMPERATURE(vref, analogRead(ATEMP), LL_ADC_RESOLUTION_12B);

        uavcan_equipment_power_BatteryInfo pkt{};
        pkt.voltage     = analogRead(PA1);
        pkt.current     = analogRead(PA0);
        pkt.temperature = cpu_temp;

        sendUavcanMsg(can1, pkt, CANARD_TRANSFER_PRIORITY_LOW);
    }

    can1.cycle();
    IWatchdog.reload();
}
