#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include <EEPROM.h>
#include <PS2X_lib.h>
#include <Servo.h>

#define END_OF_MEMORY (255)
// Begin class with selected address
// available addresses (selected by jumper on board) 
// default is ADDRESS_HIGH

//  ADDRESS_HIGH = 0x76
//  ADDRESS_LOW  = 0x77

MS5803 sensor(ADDRESS_HIGH);
int pressure_addr = 0;
int pr_address = 0;
int write__ = 0;
int read__ = 1;

//Controller 
PS2X ps2x;
int error = 0; 
byte type = 0;
byte vibrate = 0;

//Create variables to store results
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;

// Create Variable to store altitude in (m) for calculations;
double base_altitude = 1655.0; // Altitude of SparkFun's HQ in Boulder, CO. in (m)

int pressure_val = 0;
float divider = 100.00;

void setup() {
    Serial.begin(9600);
    
    //Retrieve calibrssqaation constants for conversion math.
    sensor.reset();
    
    pressure_baseline = sensor.getPressure(ADC_4096);

    //Controller 
     error = ps2x.config_gamepad(5,4,3,2, true, true);
     
   if(error == 0){
    Serial.println("Found Controller, configured successful");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Go to www.billporter.info for updates and to report bugs.");
   }
   
  else if(error == 1)
   Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
   Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
   
  else if(error == 3)
   Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
      
   type = ps2x.readType(); 
     switch(type) {
       case 0:
        Serial.println("Unknown Controller type");
       break;
       case 1:
        Serial.println("DualShock Controller Found");
       break;
       case 2:
         Serial.println("GuitarHero Controller Found");
       break;
     }
}
int n = 0;
void loop() {
  ps2x.read_gamepad(false, vibrate);

  if (ps2x.ButtonPressed(PSB_GREEN)){
    pressure_baseline = sensor.getPressure(ADC_4096);
  }
  
  // Read pressure from the sensor in mbar.
  if (ps2x.ButtonPressed(PSB_PINK)){
    pressure_abs = (sensor.getPressure(ADC_4096));
    int diff = pressure_abs - pressure_baseline;
    //pressure_abs -= 900; 
    //int next = fmod(pressure_abs, 100);

    if (pressure_addr >= (EEPROM.length()-1)){
      EEPROM.write(pressure_addr, END_OF_MEMORY);
    } else {
      EEPROM.write(pressure_addr, diff);
      //pressure_addr = pressure_addr + 1;
      //EEPROM.write(pressure_addr, next);
    }
    
    pressure_addr = pressure_addr + 1;
    
    Serial.print("Pressure abs (mbar)= ");
    Serial.println(pressure_abs);
    Serial.println(" Mod: ");
    //Serial.println(next);
    n++;
    delay(300);
   } 

   if (read__){
    byte read_value;
    //for (int x = 0; x\60; x++){
      read_value = EEPROM.read(pr_address);

      Serial.print(pr_address);
      Serial.print(" Value: ");
      Serial.println(read_value);
      
  
      pr_address = pr_address + 1;
    //}
      //read__ = 0;
   }

   Serial.println("Outside loop value: ");

//   if (true && pressure_addr<EEPROM.length()){
//    EEPROM.write(pressure_addr, 0);
//    pressure_addr++;
//   }
  }
