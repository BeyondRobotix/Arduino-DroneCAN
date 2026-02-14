#include <Arduino.h>
#include <dronecan.h>
#include <IWatchdog.h>
#include <app.h>
#include <vector>
#include <simple_dronecanmessages.h>


// set up your parameters here with default values. NODEID should be kept
std::vector<DroneCAN::parameter> custom_parameters = {
    { "NODEID", DroneCAN::INT,   100,  0, 127 },
    { "PARM_1", DroneCAN::REAL,  0.0f, 0.0f, 100.0f },
    { "PARM_2", DroneCAN::REAL,  0.0f, 0.0f, 100.0f },
};

DroneCAN dronecan;

uint32_t looptime = 0;


void setup()
{   
    // the following block of code should always run first. Adjust it at your own peril!
    app_setup();
    IWatchdog.begin(2000000);
    Serial.begin(115200);
    Serial.println("Starting!");
    dronecan.init(
        custom_parameters,
        "Beyond Robotix Node"
    );
    // end of important starting code
    
    // an example of getting and setting parameters within the code
    dronecan.setParameter("PARM_1", 50.0f); // set a parameter to 50.0
    Serial.print("PARM_1 value: ");
    Serial.println(dronecan.getParameter("PARM_1"));

    // we use a while true loop instead of the arduino "loop" function since that causes issues.
    while (true)
    {
        const uint32_t now = millis();

        // send our battery message at 10Hz
        // Don't use delay() since we need to call dronecan.cycle() as much as possible
        if (now - looptime > 100)
        {
            looptime = millis();

            // collect MCU core temperature data
            int32_t vref = __LL_ADC_CALC_VREFANALOG_VOLTAGE(analogRead(AVREF), LL_ADC_RESOLUTION_12B);
            int32_t cpu_temp = __LL_ADC_CALC_TEMPERATURE(vref, analogRead(ATEMP), LL_ADC_RESOLUTION_12B);

            // construct dronecan packet
            uavcan_equipment_power_BatteryInfo pkt{};
            pkt.voltage = analogRead(PA1);
            pkt.current = analogRead(PA0);
            pkt.temperature = cpu_temp;

            sendUavcanMsg(dronecan.canard, pkt);
        }

        dronecan.cycle();
        IWatchdog.reload();
    }
}

void loop()
{
    // Doesn't work coming from bootloader ? use while loop in setup
}
