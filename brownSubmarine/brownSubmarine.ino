#include <PS2X_lib.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_MS5803_I2C.h>


#define debug 1
//#define PRESSURE_TESTING

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

double OBS1_MAX = 0;
double OBS1_MIN = 0;
double OBS2_MIN = 0;
double OBS2_MAX = 0;
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

//Timer functions
unsigned long start_time = millis();
unsigned long elapsed_time = 0;
unsigned long current_time = 0;
int total_time = 0;

int OBS1_TIME_MOVE_FORWARD = 2000;
int PHOTO_RESISTOR_THRESHOLD = 600;

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
  dec_speed(back_left, speed_back_left); //MAX_CW
  inc_speed(back_right, MAX_CW, speed_back_right); //move forwards 
}

void turn_right(){ //assuming hard right 
  inc_speed(back_left, MAX_CCW, speed_back_left);
  dec_speed(back_right, speed_back_right); //MAX_CCW
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
void turn_off(Servo motor1, int speed1, Servo motor2, int speed2){
  dec_speed(motor1, speed1);
  dec_speed(motor2, speed2);   
}

//Set up and main loop 
void setup() {
  //Start of Setup
  Serial.begin(38400); //9600 for motors, need 57600 for the controller though


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
    Serial.print("Yaw: ");
    Serial.println(yaw);
  
  // ----- Main code ------ //
  if(start_program){
    if(ps2x.ButtonPressed(PSB_PINK)) use_controller = true; 
    
    if(!use_controller){ //autonomous code 
      // -------  PRESSURE CODE ------------------ // 
      #ifdef PRESSURE_TESTING
      pressure_abs = sensor.getPressure(ADC_4096);
         
      double pressure_change = pressure_abs - init_pressure; 
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
      
      if(pressure_change > max_allowable){ //the submarine is too far down, move up
        turn_off(back_left, speed_back_left, back_right, speed_back_right); 
        elevation_acheived = false; 
        rise(); 
      }
      else if(pressure_change < min_allowable) { //pressure_change < OBS1_MIN move down, too far up 
        turn_off(back_left, speed_back_left, back_right, speed_back_right); 
        elevation_acheived = false; 
        descend();   
      }
      else{
        turn_off(front_right, speed_front_right, front_left, speed_front_left); 
        elevation_acheived = true;   
      }    
      #endif
    // ---------- END OF PRESSURE CODE -------------- //
      if(true){
      //if(elevation_acheived){
        // IMU values 
        sensors_event_t event; 
        bno.getEvent(&event);    
      
        yaw = event.orientation.x;
        pitch = event.orientation.y;
        roll = event.orientation.z;  
        
        // ----- Photo Resistor ------ //
        photoresistorValue = analogRead(photoresistorPin);
        //Serial.print("Light Sensor value is: ");
        //Serial.println(photoresistorValue);
  
        if(!CROSSED_OBS1){
         if (yaw <= (OBS1_IMU_ANGLE - TOLERANCE) && yaw>=120){ //245 < yaw < 360, 0 < yaw < 55
            turn_right();
            Serial.print(" Right");
         }

         if( (yaw >= (OBS1_IMU_ANGLE + TOLERANCE) && (yaw <= 360))|| (yaw>0 && yaw<120)){
            Serial.print(" Left");
            turn_left();
            
         }
         if((yaw>(OBS1_IMU_ANGLE - TOLERANCE) && yaw<(OBS1_IMU_ANGLE + TOLERANCE)) && total_time<OBS1_TIME_MOVE_FORWARD){
             //current_time = millis();
             Serial.print(" Forwards");
             move_forwards();
             delay(200);
             total_time+=200;
             //total_time += ((millis())-current_time); 
             Serial.print(" Time: ");
             Serial.println(total_time);
         }

         if(total_time >= OBS1_TIME_MOVE_FORWARD){
             total_time = 0;
             while( yaw>=290 && yaw<=360 ){
              turn_right();
              Serial.println("Yaw aligning.");
             }
             if(photoresistorValue >= PHOTO_RESISTOR_THRESHOLD){
              turn_right();
             }
         }
    
        
        
      }
        else if(!CROSSED_OBS2){
          
          
          
        }
        else if(!CROSSED_OBS3){
          
          
        }
        else{
          
          
        }
      }
    
    } // end autonomous code 
    else{ //controller code 
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
