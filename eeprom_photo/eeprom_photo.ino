#include <Wire.h>
#include <EEPROM.h>
#include <PS2X_lib.h>
#include <Servo.h>

#define END_OF_MEMORY (255)
// Begin class with selected address
// available addresses (selected by jumper on board) 
// default is ADDRESS_HIGH

//  ADDRESS_HIGH = 0x76
//  ADDRESS_LOW  = 0x77


int write__ = 0;
int read__ = 0;
int pr_address = 0;


//Controller 
PS2X ps2x;
int error = 0; 
byte type = 0;
byte vibrate = 0;


// Create Variable to store altitude in (m) for calculations;
double base_altitude = 1655.0; // Altitude of SparkFun's HQ in Boulder, CO. in (m)

int address = 0;

const int pResistor = A0; // Photoresistor at Arduino analog pin A0
const int ledPin=9;       // Led pin at Arduino pin 9

//Variables
int photo_value = 0;          // Store value from photoresistor (0-1023)
int photo_value_abs = 0;

void setup() {
    Serial.begin(9600);
    //Retrieve calibration constants for conversion math.
   
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

void loop() {
 ps2x.read_gamepad(false, vibrate);
 photo_value = analogRead(pResistor);
 if(ps2x.ButtonPressed(PSB_GREEN)){
  photo_value_abs = analogRead(pResistor);
 }
 
 if(ps2x.ButtonPressed(PSB_PINK)){
    //photoresistor
    photo_value = analogRead(pResistor);
    delay(500); //Small delay
    Serial.print("The photoresistor value is: ");
    Serial.println(photo_value);
    double photo_10 = ((photo_value-photo_value_abs)/10);
    double next = fmod((photo_value-photo_value_abs), 10);

    if (address >= (EEPROM.length()-1)){
      EEPROM.write(address, END_OF_MEMORY);
    } else {
      EEPROM.write(address, (photo_10));
      address = address + 1;
      EEPROM.write(address, next);
    }
    
    address = address + 1;
    
    Serial.print("Photo resistor value from address= ");
    Serial.println(photo_10);
    Serial.print(" Mod: ");
    Serial.println(next);
    delay(300);
 }

 if (read__){
    byte read_value;
    //for (int x = 0; x<60; x++){
      read_value = EEPROM.read(pr_address);

      Serial.print(pr_address);
      Serial.print(" Value: ");
      Serial.println(read_value);
      
  
      pr_address = pr_address + 1;
    //}
      //read__ = 0;
   }

//   Serial.print("Outside loop value: ");
//   Serial.println(photo_value);
//  
//   if(true){
//     for(int z = 0; z<EEPROM.length(); z++){
//      EEPROM.write(z, 1);
//    }
//   }
  
}
