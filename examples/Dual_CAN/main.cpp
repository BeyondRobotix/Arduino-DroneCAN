/*
This example ONLY works on our H7 based nodes. it will not work on the Micro CAN node which is L431 one based.
*/

#include <Arduino.h>
#include <dronecan.h>

// PORT1 instance — FDCAN1 on MicroNodePlus (PD_0/PD_1)
//                  FDCAN2 on CoreNode      (PB_5/PB_6)
std::vector<DroneCAN::parameter> params_port1 = {
    {"NODEID", DroneCAN::INT, 100, 1, 127},
    {"PARM_1", DroneCAN::REAL, 0.0f, 0.0f, 100.0f},
    {"PARM_2", DroneCAN::REAL, 0.0f, 0.0f, 100.0f},
};

// PORT2 instance — FDCAN2 on MicroNodePlus (PB_5/PB_6 — confirm from schematic)
//                  FDCAN1 on CoreNode      (PB_8/PB_9 — confirm from schematic)
std::vector<DroneCAN::parameter> params_port2 = {
    {"NODEID", DroneCAN::INT, 101, 1, 127},
    {"PARM_1", DroneCAN::REAL, 0.0f, 0.0f, 100.0f},
    {"PARM_2", DroneCAN::REAL, 0.0f, 0.0f, 100.0f},
};

DroneCAN can1;
DroneCAN can2;

uint32_t looptime1 = 0;
uint32_t looptime2 = 0;

void setup()
{
    app_setup();
    IWatchdog.begin(2000000);
    Serial.begin(115200);
    Serial.println("Starting dual-port!");

    // Each instance gets its own canard context, memory pool, and flash page.
    // storage_page defaults: PORT1 → last sector, PORT2 → second-to-last sector.
    can1.init(params_port1, "Beyond Robotix Node/Port1",
              DroneCAN::CanMode::FD, DroneCAN::CanPort::PORT1);
    can2.init(params_port2, "Beyond Robotix Node/Port2",
              DroneCAN::CanMode::FD, DroneCAN::CanPort::PORT2);

    Serial.print("Port1 NODEID: "); Serial.println(can1.getParameter("NODEID"));
    Serial.print("Port2 NODEID: "); Serial.println(can2.getParameter("NODEID"));
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

    // PORT2: broadcast BatteryInfo at 10 Hz (independent node, independent transfer IDs)
    if (now - looptime2 > 100)
    {
        looptime2 = now;

        int32_t vref     = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
        int32_t cpu_temp = __LL_ADC_CALC_TEMPERATURE(vref, analogRead(ATEMP), LL_ADC_RESOLUTION_12B);

        uavcan_equipment_power_BatteryInfo pkt{};
        pkt.voltage     = analogRead(PA1);
        pkt.current     = analogRead(PA0);
        pkt.temperature = cpu_temp;

        sendUavcanMsg(can2, pkt, CANARD_TRANSFER_PRIORITY_LOW);
    }

    can1.cycle();
    can2.cycle();
    IWatchdog.reload();
}
