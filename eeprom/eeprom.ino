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
int address = 0;
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

//PHOTORESISTOR 
const int pResistor = A0; // Photoresistor at Arduino analog pin A0
const int ledPin=9;       // Led pin at Arduino pin 9
int photo_value = 0;          // Store value from photoresistor (0-1023)
bool photo = false; 

void setup() {
    Serial.begin(9600);
    
    //Retrieve calibration constants for conversion math.
    sensor.reset();
    sensor.begin();
    
    pressure_baseline = sensor.getPressure(ADC_4096);

    pinMode(pResistor, INPUT);

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

//  if (address < (EEPROM.length())){
//      EEPROM.write(address, 255);
//  }
//    address++;
  ps2x.read_gamepad(false, vibrate);

  if(ps2x.ButtonPressed(PSB_START)){
     EEPROM.write(address, 0);
     address++; 
     EEPROM.write(address, 0);
     address++;
     EEPROM.write(address, 0);
     address++; 
     photo = !photo; 
  }

  // Read pressure from the sensor in mbar.
  if (ps2x.ButtonPressed(PSB_PINK) && !photo){
//  if(n<=500){
    pressure_abs = (sensor.getPressure(ADC_4096)); 
    int next = fmod(pressure_abs, 10);

    if (address >= (EEPROM.length()-1)){
      EEPROM.write(address, END_OF_MEMORY);
    } else {
      EEPROM.write(address, pressure_abs);
      address = address + 1;
      EEPROM.write(address, next);
    }
    
    address = address + 1;
    
    Serial.print("Pressure abs (mbar)= ");
    Serial.println(pressure_abs);
    Serial.println(" Mod: ");
    Serial.println(next);
    n++;
    delay(500);
   } 
   
   if(ps2x.ButtonPressed(PSB_RED) && photo){
     //photoresistor
     photo_value = analogRead(pResistor);
     delay(500); //Small delay
     Serial.print("The photoresistor value is: ");
     Serial.println(photo_value);
       
     int next = fmod(photo_value, 10);
      
     if (address >= (EEPROM.length()-1)){
       EEPROM.write(address, END_OF_MEMORY);
     } else {
       EEPROM.write(address, (photo_value/10));
       address = address + 1;
       EEPROM.write(address, next);
     }
      
     address = address + 1;
      
     Serial.print("Photo resistor value= ");
     Serial.println(photo_value);
     Serial.print(" Mod: ");
     Serial.println(next);
     delay(500);
   }

   
   if (read__ && n<520){
    byte read_value;
    //for (int x = 0; x<60; x++){
      read_value = EEPROM.read(pr_address);

      Serial.print(pr_address);
      Serial.print(" Value: ");
      Serial.println(read_value);
      
  
      pr_address = pr_address + 1;
      n++;
    //}
      //read__ = 0;
   }
  }
