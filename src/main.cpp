#include <Arduino.h>
#include <YC_Moisture_Monitor.h>

// variables:
bool calibration{true}; //true if want to calibrate all sensors and store parameters in EEPROM. false if just to use parameters stored in EEPROM.
bool usePumps{false};     //true if want to use the setupPumps(), pumpStart() functions for watering. the following arrays pumpOnTime[] and pumpSensorDryThreshold[]
                         //in YC_Moisture_Monitor.cpp need to be configured manually.

void setup()
{
  #ifdef STM32F103xB//need to setup EEPROM emulation in flash if mcu is STM32F103xB
  setupEEPROM();
  #endif
  if (calibration)
    delay(20000); //wait for 20 sec so that you have time to connect bluetooth for calibration. (notice that the sensor monitor must be calibrated
                  //in bluetooth mode. Otherwise, the calibration will not be accurate.)
  Serial.begin(9600);
 // while (!Serial)
  {
    ; //wait until serial port is ready
  }
  Serial.println("YCC: serial monitor successfully connected...");
  setupSensors(calibration);
  setupDH22();
  if (usePumps)
    setupPumps();
  //setupLora();
}
void loop()
{
  monitorSensors(usePumps);
}