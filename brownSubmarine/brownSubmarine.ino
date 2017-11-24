#include <PS2X_lib.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_MS5803_I2C.h>


#define debug 1
#define PRESSURE_TESTING

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

//GLOBAL VARIABLE START PROGRAM
bool start_program = false; 

//Global Variables Motors & Controller
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

// Global Variables Photoresistor 
const int photoresistorPin = A0;
const int ledPin = 9;

int photoresistorValue = -1; //initialize to garbage value 
double total_distance = 0;

// Global Variables IMU 
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float yaw = 0;
float pitch = 0;
float roll = 0;
float yaw_adjusted = 360; //set to 360

// Pressure Sensor 
MS5803 sensor(ADDRESS_HIGH);
double pressure_abs;
double base_altitude = 1655.0; // Altitude of SparkFun's HQ in Boulder, CO. in (m)
double init_pressure = 0;

// Pressure Sensor Autonomous Variables
bool CROSSED_OBS1 = false;
bool CROSSED_OBS2 = false; 
bool CROSSED_OBS3 = false; 

double OBS1_MAX = 100;
double OBS1_MIN = -100;
double OBS2_MIN = 100;
double OBS2_MAX = -100;
double OBS3_MIN = 0;
double OBS3_MAX = 0; 
double GROUND = 0; 

double OBS1_DISTANCE = 72;
double OBS2_DISTANCE = 144;
double OBS3_DISTANCE = 342;
double OBS2_WIDTH = 38;

bool elevation_acheived = false; 

// IMU VALUES Autonomous Variables 
const int OBS1_IMU_ANGLE = 300;
const int TOLERANCE = 5;
const int OBS2_IMU_ANGLE = 0;
const int TIME_OBS2 = 100000; //ms
int align_to_zero = 0;  

//Timer functions
unsigned long start_time = millis();
unsigned long elapsed_time = 0;
unsigned long current_time = 0;
int total_time_forward = 0;
int OBS1_TOTAL_TIME = 0;
int total_time = 0;

int OBS1_TIME_MOVE_FORWARD = 6000;
int TIME_TO_CROSS_FIRST_OBSTACLE = 8000;
int PHOTO_RESISTOR_THRESHOLD = 600;
int MOVE_FORWARD_OBS1 = 0;
int CURRENT_MOVE_FORWARD_OBS1 = 0;

int CURRENT_TIME_OBS2 = 0; //global variable

//Helper Functions MOTORS 
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
  //NOTE: THIS FUNCTION IS ONLY FOR MOVING TO MAXIMUM CW AND CCW SPEEDS 
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
  Serial.println("LEFT");
  inc_speed(back_left, MAX_CW, speed_back_left); //MAX_CW
  inc_speed(back_right, MAX_CW, speed_back_right); //move forwards 
}

void turn_right(){ //assuming hard right 
    Serial.println("RIGHT");
  inc_speed(back_left, MAX_CCW, speed_back_left);
  inc_speed(back_right, MAX_CCW, speed_back_right); //MAX_CCW
}

void rise_left(){
  inc_speed(front_left, MAX_CCW, speed_front_left);
  dec_speed(front_right, speed_front_right); //MAX_CCW
}

void rise_right(){
  dec_speed(front_left, speed_front_left); //MAX_CW
  inc_speed(front_right, MAX_CW, speed_front_right); 
}

//turn off motors function
void turn_off(Servo motor1, int &speed1, Servo motor2, int &speed2){
  motor1.write(STOP); 
  motor2.write(STOP);
  speed1 = STOP;
  speed2 = STOP;
}

//Set up and main loop 
void setup() {
  //Start of Setup
  Serial.begin(38400); //9600 for motors, need 57600 for the controller though
  use_controller = false;

  // -------- PIN INITIALIZATIONS ------// 
  // Motors
  back_right.attach(BACK_RIGHT_PIN);
  back_left.attach(BACK_LEFT_PIN);
  front_right.attach(FRONT_RIGHT_PIN);
  front_left.attach(FRONT_LEFT_PIN);

  //-------- PHOTO RESISTOR INITIALIZATION --------//
  pinMode(photoresistorPin, INPUT);// Set pResistor - A0 pin as an input (optional)

  // -------------------------------------------- //

  // --------- CONTROLLER INITIALIZATION -------------//
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
  // ----------------------------------------------- //

  //--------------- SERVO INITIALIZATION ----------- //
  back_right.write(90);
  back_left.write(90);
  front_right.write(90);
  back_left.write(90);
  // -------------------------------------------- //

  //--------- IMU INITIALIZATION --------//
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  // ------------------------------------ //

  //------------ PRESSURE --------------- //
  sensor.reset();
  sensor.begin();
  
  init_pressure = sensor.getPressure(ADC_4096); 
  // ----------------------------------- //
  
  Serial.println("Initialized Shaheen One"); 
} 

void loop() {       

// ----- Controller Code ----- //  
   ps2x.read_gamepad(false, vibrate);
   if(ps2x.ButtonPressed(PSB_START)){
    setup();
    
    //inital pressure value 
    start_program = true; 
  }     
   
  // ----- Main code ------ //
  if(start_program){

    if(ps2x.ButtonPressed(PSB_PINK)){
      back_left.write(STOP);
      back_right.write(STOP);
      front_left.write(STOP);
      front_right.write(STOP);
      speed_front_left = STOP;
      speed_front_right = STOP;
      speed_back_left = STOP;
      speed_back_right = STOP;
      use_controller = true; 
    }
    
    if(!use_controller){ //autonomous code 
     sensors_event_t event; 
     bno.getEvent(&event);    
      
     yaw = event.orientation.x;
     pitch = event.orientation.y;
     roll = event.orientation.z;  
    
      // -------  PRESSURE CODE ------------------ // 
      #ifdef PRESSURE_TESTING
      pressure_abs = sensor.getPressure(ADC_4096);
         
      double pressure_change = pressure_abs - init_pressure; 
//      Serial.print("Pressure change: ");
//      Serial.println(pressure_change);
      double max_allowable = -1; 
      double min_allowable = 100000;
       
      if(!CROSSED_OBS1){
         max_allowable = OBS1_MAX;
         min_allowable = OBS1_MIN;
      }
      else if(!CROSSED_OBS2){
         max_allowable = OBS2_MAX;
         min_allowable = OBS2_MIN;      
      }
      else if(!CROSSED_OBS3){
         max_allowable = OBS3_MAX;
         min_allowable = OBS3_MIN;  
      }
      else{ // everything is done, just land
          max_allowable = GROUND; 
          min_allowable = GROUND; 
      }
      
      if(pressure_change >= max_allowable){ //the submarine is too far down, move up
        turn_off(back_left, speed_back_left, back_right, speed_back_right); 
        elevation_acheived = false; 
        if(pitch > 45){
          turn_off(front_left, speed_front_left, front_right, speed_front_right); 
          delay(300);
        }
        Serial.println("RISE");
        rise(); 
      }
      else if(pressure_change <= min_allowable) { //pressure_change < OBS1_MIN move down, too far up 
        turn_off(back_left, speed_back_left, back_right, speed_back_right); 
        elevation_acheived = false; 
        Serial.println("DESCEND");
        descend();   
      }
      else if(pressure_change>min_allowable && pressure_change < max_allowable){
        turn_off(front_right, speed_front_right, front_left, speed_front_left); 
//        Serial.println("ELEVATION GOOD");
        elevation_acheived = true;   
      }    
      #endif
    // ---------- END OF PRESSURE CODE -------------- //
      //if(true){
      if(elevation_acheived){
        turn_off(front_right, speed_front_right, front_left, speed_front_left); 
        // IMU values 
        //sensors_event_t event; 
        bno.getEvent(&event);    
      
        yaw = event.orientation.x;
        pitch = event.orientation.y;
        roll = event.orientation.z;  
        
        // ----- Photo Resistor ------ //
        photoresistorValue = analogRead(photoresistorPin);
        //Serial.print("Light Sensor value is: ");
        //Serial.println(photoresistorValue);
  
        if(!CROSSED_OBS1){
         if ((yaw <= (OBS1_IMU_ANGLE - TOLERANCE) && yaw>=120) && align_to_zero==0){ //245 < yaw < 360, 0 < yaw < 55
            turn_right();
            Serial.print(" Right");
         }

         if( ((yaw >= (OBS1_IMU_ANGLE + TOLERANCE) && (yaw <= 360))|| (yaw>0 && yaw<120))&& align_to_zero == 0){
            Serial.print(" Left");
            turn_left();
            
         }
         if( align_to_zero == 0 &&(yaw>(OBS1_IMU_ANGLE - TOLERANCE) && yaw<(OBS1_IMU_ANGLE + TOLERANCE))){
             //current_time = millis();
             Serial.print(" Forwards");
             move_forwards();
             delay(200);
             total_time_forward+=200;
             OBS1_TOTAL_TIME+=200;
//             if(OBS1_TOTAL_TIME>=TIME_TO_CROSS_FIRST_OBSTACLE){
//              CROSSED_OBS1 = 1;
//              Serial.println("Exiting OBSC1");
//             }
             //total_time += ((millis())-current_time); 
             Serial.print(" Time: ");
             Serial.println(total_time_forward);
         }

         if(total_time_forward >= OBS1_TIME_MOVE_FORWARD){
             align_to_zero = 1;
             if( yaw>=270 && yaw<=360){
              turn_right();
              Serial.print("Yaw aligning:  ");
              Serial.println( yaw );
             } else {
              align_to_zero = 0;
              total_time_forward = 0;
              turn_off(back_left, speed_back_left, back_right, speed_back_right);
              delay(300);
              MOVE_FORWARD_OBS1 = 1;
             }   
         }

         if (MOVE_FORWARD_OBS1){
           move_forwards();
           delay(200);
           CURRENT_MOVE_FORWARD_OBS1+=200;
           align_to_zero = 1;
           if (CURRENT_MOVE_FORWARD_OBS1 > 2000){
            turn_off(back_left, speed_back_left, back_right, speed_back_right);
             Serial.println("Forwards to OBS2");
             CROSSED_OBS1 = 1;
             Serial.print("OBS1 done MOVE FORWARDS: CURRENT_MOVE_FORWARD_OBS1= ");
            Serial.print(CURRENT_MOVE_FORWARD_OBS1);
             turn_off(back_left, speed_back_left, back_right, speed_back_right); 
           }

           if(yaw<357 && yaw>180){
            turn_right();
           }

           if(yaw>3 && yaw<=180){
            turn_left();
           }

           if((yaw>=357 && yaw<=360) || (yaw>=0 && yaw<=3) ){
            move_forwards();
            Serial.print("OBS1 done MOVE FORWARDS: CURRENT_MOVE_FORWARD_OBS1= ");
            Serial.print(CURRENT_MOVE_FORWARD_OBS1);
 
           }
           
           
         }
         
         if(photoresistorValue >= PHOTO_RESISTOR_THRESHOLD){  
              //while(photoresistorValue >= PHOTO_RESISTOR_THRESHOLD){
//                for(int i=0; i<4; i++){
//                  back_right.write(90);//dec_speed(back_right, speed_back_right);//(back_right, speed_back_right, back_right, speed_back_right);
//                  delay(100);
//                  turn_right();
//                  Serial.print(" Photoresistor turn right.");
//                  Serial.println(photoresistorValue);
//                  delay(200);
//                }
                
              //}
              
              //align_to_zero = 1;
         } 
      }
      else if(!CROSSED_OBS2){
          if(yaw<357 && yaw>180){
              turn_right();
           }
  
           if(yaw>3 && yaw<=180){
              turn_left();
           }
        
           if((yaw>=357 && yaw<=360) || (yaw>=0 && yaw<=3) ){
              CURRENT_TIME_OBS2+=200;
              if(CURRENT_TIME_OBS2<7500){
                 move_forwards();
                 delay(200);
              }
           }
           
//        Serial.println("2222222222222222222222");
//        //programming for Obstacle 2 assuming it is already aligned. 
//        if(photoresistorValue >= PHOTO_RESISTOR_THRESHOLD){
//        //the submarine is too close to the wall, move away 
//          turn_right(); 
//        }
//        else{ //the submarine is not too far away, check the yaws 
//          if (yaw <= (OBS2_IMU_ANGLE - TOLERANCE) && yaw>=120){ //245 < yaw < 360, 0 < yaw < 55
//          //if sub is facing left
//            turn_right();
//            Serial.print("Right");
//         }
//
//         else if( (yaw >= (OBS2_IMU_ANGLE + TOLERANCE) && (yaw <= 360))|| (yaw>0 && yaw<120)){
//          // if sub is facing right   
//            turn_left();
//            Serial.print("Left");
//         }
//
//         else{
//            move_forwards();
//            delay(200);
//            total_time += 200; 
//
//            if(total_time >= TIME_OBS2){
//              CROSSED_OBS2 = true;   
//            }
//         }        
//        }
      }
      else if(!CROSSED_OBS3){
        
        
      }
      else{
        
        
      }
    }
    
    } // end autonomous code 
    else{ //controller code 
      Serial.println("CONTROLLER ACCESS");
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
}
