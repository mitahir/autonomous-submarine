#include <PS2X_lib.h>
#include <Servo.h>

#define debug 1

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
//    if (speed >= max){
//      motor.write(max);
//      speed = max; 
//      return speed; 
//     }
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
//    if (speed <= max){
//      motor.write(max);
//      speed = max; 
//      return speed; 
//    }
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
  dec_speed(back_left, speed_back_left); //MAX_CW
  inc_speed(back_right, MAX_CW, speed_back_right); //move forwards 
}

void turn_right(){ //assuming hard right 
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
  // put your setup code here, to run once:
  back_right.attach(BACK_RIGHT_PIN);
  back_left.attach(BACK_LEFT_PIN);
  front_right.attach(FRONT_RIGHT_PIN);
  front_left.attach(FRONT_LEFT_PIN);

  Serial.begin(38400); //9600 for motors, need 57600 for the controller though
  
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


     back_right.write(90);
     back_left.write(90);
     front_right.write(90);
     back_left.write(90);
  
  Serial.println("Initialized Shaheen One"); 
} 

void loop() {
   ps2x.read_gamepad(false, vibrate);
   if(ps2x.ButtonPressed(PSB_START)) use_controller = true; 

   if(!use_controller){
  }
  else{
    if(ps2x.Button(PSB_R1)){ //take inputs from both the sticks
#ifdef debug
       Serial.print(ps2x.Analog(PSS_RX),DEC); 
       Serial.print(",");
       Serial.println(ps2x.Analog(PSS_RY), DEC); 
#endif 
       if(ps2x.Analog(PSS_RY) == byte(0)){ //move forwards
          move_forwards();
       }
       if(ps2x.Analog(PSS_RY) == byte(255)){
          move_backwards(); 
       }
       if(ps2x.Analog(PSS_RX) == byte(0)){
          turn_left();  //hard left     
       }
       if(ps2x.Analog(PSS_RX) == byte(255)){
          turn_right();  //hard right
       }
    }
    if(ps2x.Button(PSB_L1)){
#ifdef debug
       Serial.print(ps2x.Analog(PSS_LX),DEC); 
       Serial.print(",");
       Serial.println(ps2x.Analog(PSS_LY), DEC); 
#endif
   
      if(ps2x.Analog(PSS_LY) == byte(0)){
       // Serial.println("RISING"); 
        rise();
        }   
      if(ps2x.Analog(PSS_LY) == byte(255)){
       // Serial.println("DESCENDING");
        descend(); 
        }
      if(ps2x.Analog(PSS_RX) == byte(0)){
        //increase the left side motor to work faster
        rise_left(); 
        }
      if(ps2x.Analog(PSS_RX) == byte(255)){
        //increase the front right motor speed
        rise_right(); 
        }
       
    }
    if(ps2x.ButtonReleased(PSB_R1)){
       speed_back_right = 90;
       back_right.write(90); 
       speed_back_left = 90;
       back_left.write(90);  
    }
    if(ps2x.ButtonReleased(PSB_L1)){
       speed_front_right = 90;
       front_right.write(90); 
       speed_front_left = 90;
       front_left.write(90);  
    }
  }
}
