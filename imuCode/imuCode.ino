//#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>
//  

//void setup(void) 
//{
//  Serial.begin(9600);
//  Serial.println("Orientation Sensor Test"); Serial.println("");
//  
//  /* Initialise the sensor */
//  if(!bno.begin())
//  {
//    /* There was a problem detecting the BNO055 ... check your connections */
//    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//    while(1);
//  }
//  
//  delay(1000);
//    
//  bno.setExtCrystalUse(true);
//}
// 
//void loop(void) 
//{
//  /* Get a new sensor event */ 
//  sensors_event_t event; 
//  bno.getEvent(&event);
//  
//  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(event.orientation.x, 4);
//  Serial.print("\tY: ");
//  Serial.print(event.orientation.y, 4);
//  Serial.print("\tZ: ");
//  Serial.print(event.orientation.z, 4);
//  Serial.println("");
//  
//  delay(100);
//}


#include <PS2X_lib.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define debug 1

Adafruit_BNO055 bno = Adafruit_BNO055(55);

//Define the pins 
#define FAST_INC  10
#define SLOW_INC  5
#define FAST_DEC  20
#define SLOW_DEC   5

#define BACK_RIGHT_PIN  11
#define BACK_LEFT_PIN   8
#define FRONT_RIGHT_PIN 9
#define FRONT_LEFT_PIN  10

//Define the motor values
#define MAX_CCW 175
#define MIN_CCW 95
#define MAX_CW  25
#define MIN_CW  80
#define STOP    90

//Global Variables
Servo back_right; 
Servo back_left; 
Servo front_right;
Servo front_left; 
PS2X ps2x;
int error = 0; 
byte type = 0;
byte vibrate = 0;
bool use_controller = false; 

int speed_back_right = 90;
int speed_back_left = 90;
int speed_front_right = 90;
int speed_front_left = 90;

int start = 1;

int angle = 90;
int position = 90;
int feedbackPin = 5;
int val = 0;
int calVal[] = {191, 1011};  // initial cal values
int calStartPos = 0;
int final = 90;

float yaw = 0;
float pitch = 0;
float roll = 0;

float yaw_adjusted = 0;

Servo servo;

#define NUMREADINGS 10      // Number of readings to take for smoothing


int readings[NUMREADINGS];                // the readings from the analog input
int index = 0;                            // the index of the current reading
int total = 0;                            // the running total
int average = 0; 

//Helper Functions 

int dec_speed(Servo motor, int& speed){
  if(abs(90 - speed) <= FAST_DEC){
    motor.write(speed);
    speed = 90; 
    return speed; 
  }
  else if(speed < 90){ //cw motion
    speed += FAST_DEC;
  }
  else{ //ccw motion
    speed -= FAST_DEC;
  }

  motor.write(speed);
  return speed;
}

int inc_speed(Servo motor, int max, int& speed){ //increase speed fast. Note: cw == 1 for clockwise & 0 for ccw
  if(abs(speed - max) <= FAST_INC){
      motor.write(max);
      speed = max;
      return speed; 
    }
  
  if(max == MAX_CCW){ //want to get to max ccw speed
    //Case 1: speed already > 90
    if(speed >= 90){
      speed += FAST_INC;   
    }
    else{
      //this means the motor is moving in a cw direction, must first stop it and then increase. 
      while(speed >= 90){
        speed = dec_speed(motor, speed);   
      }
      speed += FAST_INC;     
    }
  }
  else{
    //Case 1: speed already < 90
    if(speed <= 90){
      speed -= FAST_INC;   
    }
    else{
      //this means the motor is moving in a ccw direction, must first stop it and then increase. 
      while(speed <= 90){
        speed = dec_speed(motor, speed);      
      }
      speed -= FAST_INC; 
    }
  }
  
  motor.write(speed);
  return speed; 
}

//Submarine functions

void move_forwards(){ //while _____ is pressed, move forwards 
    //back_left needs to go ccw
    inc_speed(back_left, MAX_CCW, speed_back_left); 
    //back_right needs to go cw
    inc_speed(back_right, MAX_CW, speed_back_right); 
}

void move_backwards(){
  inc_speed(back_left, MAX_CW, speed_back_left);
  inc_speed(back_right, MAX_CCW, speed_back_right); 
}

void rise(){
  inc_speed(front_left, MAX_CCW, speed_front_left); 
  inc_speed(front_right, MAX_CW, speed_front_right);   
}

void descend(){
  inc_speed(front_left, MAX_CW, speed_front_left); 
  inc_speed(front_right, MAX_CCW, speed_front_right);  
}


void turn_left(){ //assuming hard left
  Serial.println("Turn Left");
  dec_speed(back_left, speed_back_left); //MAX_CW
  inc_speed(back_right, MAX_CW, speed_back_right); //move forwards 
}

void turn_right(){ //assuming hard right 
  Serial.println("Turn Right");
  inc_speed(back_left, MAX_CCW, speed_back_left);
  dec_speed(back_right, speed_back_right); //MAX_CCW
}

void rise_left(){
  inc_speed(front_left, MAX_CCW, speed_front_left);
  inc_speed(front_right, 90, speed_front_right); //MAX_CCW
}

void rise_right(){
  inc_speed(front_left, 90, speed_front_left); //MAX_CW
  inc_speed(front_right, MAX_CW, speed_front_right); 
}

//Set up and main loop 
void setup() {

  //analogReference(EXTERNAL);
  Serial.begin(9600); //9600 for motors, need 57600 for the controller though
  
  // IMU SETUP
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

  // CONTROLLER SETUP
//  
//  // put your setup code here, to run once:
  Serial.begin(9600);
  back_right.attach(BACK_RIGHT_PIN);
  back_left.attach(BACK_LEFT_PIN);
  front_right.attach(FRONT_RIGHT_PIN);
  front_left.attach(FRONT_LEFT_PIN);

  
  
  error = ps2x.config_gamepad(5,4,3,2, true, true);
//   
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
//
//
     back_right.write(90);
     back_left.write(90);
     front_right.write(100);
     front_left.write(90);
  
  Serial.println("Initialized Shaheen One"); 
} 

void loop() {
  
    // move_forwards();  //analogReference(EXTERNAL);
  
  ///////////////Encoder
//  static int v = 0;
//  position = analogRead(feedbackPin);  
//  position = smooth(position);
//  Serial.print("Recording position: ");
//  if ( 1) {
//    char ch = Serial.read();
//    Serial.print("Recording position:1111111 ");
//    //switch(1) {
//      //case '0'...'9':
//        v = v * 10 + ch - '0';
//        //break;
//        
//      //case 1:   //  record the position of the pot  
//        servo.detach();   
//        Serial.print("Recording position: ");
//        Serial.println(position);      
//        final = position;  
//        v = 0;
//        //break;
//        
//     // case 'b':  //  stop playing
//        //Serial.println("Stop");
//        //servo.detach();    
//        //v = 0;
//        //break;
//        
//      //case 'c':   // play            
//        angle = map(final, calVal[0], calVal[1], 0, 180);
//        Serial.print("playing: ");
//        Serial.println(angle);        
//        servo.attach(9);  
//        servo.write(angle);  
//        v = 0;
//       // break;
//        
//        
//    //  case 'd' :  // calibrate      
////        Serial.println("calibrating");
////        servo.attach(9);
////        servo.write(1);
////        delay(1000);  // wait 1 second for servo to reach the position                
////        calVal[0] = analogRead(feedbackPin);  
////        servo.write(180);
////        delay(1000);
////        calVal[1] = analogRead(feedbackPin);
////        Serial.print("Cal values: ");
////        Serial.print(calVal[0]);
////        Serial.print(",");
////        Serial.println(calVal[1]);
////        v = 0 ;
//      //  break;      
//    //}
//  }



  ////////////////
  
  
  // IMU LOOP
  sensors_event_t event; 
  bno.getEvent(&event);

  yaw = event.orientation.x;
  pitch = event.orientation.y;
  roll = event.orientation.z;
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(yaw, 4);
  Serial.print("\tY: ");
  Serial.print(pitch, 4);
  Serial.print("\tZ: ");
  Serial.print(roll, 4);
  Serial.println("");
  
  //delay(100);
  yaw_adjusted = 360;
  if ((yaw>0 && yaw<15) || (yaw<360&&yaw>345)){
  //if(1){
    Serial.println("FORWARD");
    move_forwards();
    //back_right.write(90);//
    //delay(1000);
  }
  
  if (yaw > 15 && yaw < 90){
    turn_left();
    Serial.println("Turn left");
  } 

  if (yaw<=345 && yaw > 270){
    Serial.println("Turn right");
    turn_right();
    //delay(1000);
  }
//  // CONTROLLER LOOP
//
//  
//  
//   ps2x.read_gamepad(false, vibrate);
//   if(ps2x.ButtonPressed(PSB_START)) use_controller = true; 

  //autonomous code
//   if(!use_controller){
//    if (start == 1){
//      descend();
//      //move_right(); MOVE_RIGHT
//      start = 0;
//    }
//    
//    if(event.orientation.x<0){
//        while(event.orientation.x < 0){    
//          turn_right();
//      }
//    }
//    else if(event.orientation.x>0){
//      while(event.orientation.x > 0){
//        turn_left();
//      }
//
//    if(event.orientation.y<0){
//        while(event.orientation.y < 0){    
//          turn_right();
//      }
//    }
//    else if(event.orientation.y>0){
//      while(event.orientation.y > 0){
//        turn_left();
//      }
//      
//      }
//    }
//  else{
//    //Serial.println("Shouldn't be here");
//    if(ps2x.Button(PSB_R1)){ //take inputs from both the sticks
//#ifdef debug
//       Serial.print(ps2x.Analog(PSS_RX),DEC); 
//       Serial.print(",");
//       Serial.println(ps2x.Analog(PSS_RY), DEC); 
//#endif 
//       if(ps2x.Analog(PSS_RY) == byte(0)){ //move forwards
//          move_forwards();
//       }
//       if(ps2x.Analog(PSS_RY) == byte(255)){
//          move_backwards(); 
//       }
//       if(ps2x.Analog(PSS_RX) == byte(0)){
//          turn_left();  //hard left     
//       }
//       if(ps2x.Analog(PSS_RX) == byte(255)){
//          turn_right();  //hard right
//       }
//    }
//    if(ps2x.Button(PSB_L1)){
//#ifdef debug
//       Serial.print(ps2x.Analog(PSS_LX),DEC); 
//       Serial.print(",");
//       Serial.println(ps2x.Analog(PSS_LY), DEC); 
//#endif
//   
//      if(ps2x.Analog(PSS_LY) == byte(0)){
//       // Serial.println("RISING"); 
//        rise();
//        }   
//      if(ps2x.Analog(PSS_LY) == byte(255)){
//       // Serial.println("DESCENDING");
//        descend(); 
//        }
//      if(ps2x.Analog(PSS_RX) == byte(0)){
//        //increase the left side motor to work faster
//        rise_left(); 
//        }
//      if(ps2x.Analog(PSS_RX) == byte(255)){
//        //increase the front right motor speed
//        rise_right(); 
//        }
//       
//    }
//    if(ps2x.ButtonReleased(PSB_R1)){
//       speed_back_right = 90;
//       back_right.write(90); 
//       speed_back_left = 90;
//       back_left.write(90);  
//    }
//    if(ps2x.ButtonReleased(PSB_L1)){
//       speed_front_right = 90;
//       front_right.write(90); 
//       speed_front_left = 90;
//       front_left.write(90);  
//    }
//  }
// }
}

int smooth(int data) {
    total -= readings[index];               // subtract the last reading
    readings[index] = analogRead(feedbackPin); // read from the sensor
    total += readings[index];               // add the reading to the total
    index = (index + 1);                    // advance to the next index

    if (index >= NUMREADINGS)               // if we're at the end of the array...
    index = 0;                            // ...wrap around to the beginning

    val = total / NUMREADINGS;          // calculate the average
    return val;
}
