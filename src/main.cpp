#include <Arduino.h>
#include <YC_Moisture_Monitor.h>

// variables:
bool calibration{false}; //true if want to calibrate all sensors and store parameters in EEPROM. false if just to use parameters stored in EEPROM.
bool usePumps{false};   //true if want to use the setupPumps(), pumpStart() functions for watering. the following arrays pumpOnTime[] and pumpSensorDryThreshold[]
                        //in YC_Moisture_Monitor.cpp need to be configured manually.

void setup()
{
#ifdef STM32F103xB
#define Serial_YC Serial3 //default serial port is usart1 with TX=A9 and RX=A10. so, connect GND, A9 and A10 of STM32F103C8 to FTDI will establish a valid serial port. likewise, \
                          //connect A9 and A10 to a bluetooth module will port the serial output via bluetooth. so, stm32 boards can achieve the same thing as an avr board.      \
                          //note that Serial1 does not work with maple core. so, need to switch to other serial pins such as Serial3, which is TX=B10 and RX=B11
  setupEEPROM(calibration);          //need to setup EEPROM emulation in flash if mcu is STM32F103xB
#elif defined(ARDUINO_AVR_NANO)
#define Serial_YC Serial
#endif
  if (calibration)
    delay(20000); //wait for 20 sec so that you have time to connect bluetooth for calibration. (notice that the sensor monitor must be calibrated
                  //in bluetooth mode. Otherwise, the calibration will not be accurate.)
  Serial_YC.begin(9600);
  while (!Serial_YC)
  {
    ; //wait until serial port is ready
  }
  Serial_YC.println("YCC: serial monitor successfully connected...");
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