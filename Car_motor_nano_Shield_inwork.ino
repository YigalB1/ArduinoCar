/*************************************************************
Motor Shield 1-Channel DC Motor Demo
http://www.instructables.com/id/Arduino-Motor-Shield-Tutorial/
*************************************************************/
#include <Servo.h> 

#define MOTOR_LEFT_DIR_PIN 12
#define MOTOR_LEFT_BREAK_PIN 9
#define MOTOR_LEFT_SPEED_PIN 3
#define MOTOR_RIGHT_DIR_PIN 13
#define MOTOR_RIGHT_BREAK_PIN 8
#define MOTOR_RIGHT_SPEED_PIN 11

#define SHORT_RANGE 20
#define STOP_RANGE 6
#define SERVO_STEPS_NUM 10
#define SERVO_STEPS_INC 180 / SERVO_STEPS_NUM

#define MOTOR_LEFT_MAX_SPEED 255 // max is 255. should be lower if calibrated needed
#define MOTOR_RIGHT_MAX_SPEED 255 // max is 255. should be lower if calibrated needed

#define NANO 0 // make 0 in case of UNO; 1 in case of Nano
#define UNo  1 

#define DEBUG 1
#define CALIBRATE 1
#define TEST_MODE 1


const int FORWARD   = 0;     // const value can't change
const int STOP      = 1;
const int BACKWARD  = 2;
const int GO        = 3;
const int LEFT      = 4;
const int RIGHT     = 5;
const int trigPin   = 2;  // Ultrasonic (Yellow)
const int echoPin   = 4;  // Ultrasonic (orange)
const int analog_in_pin = 2 ; // potentiometer for claibration

int dist_array[SERVO_STEPS_NUM];
int motor_left_speed = MOTOR_LEFT_MAX_SPEED;
int motor_right_speed = MOTOR_RIGHT_MAX_SPEED;

Servo myservo;  // create servo object to control a servo
                // twelve servo objects can be created on most boards



//******************* SETUP ************************

void setup() {
  
  //Setup Channel A
  pinMode(12, OUTPUT); //Initiates Motor Channel A pin
  pinMode(9, OUTPUT); //Initiates Brake Channel A pin
  //Setup Channel B
  pinMode(13, OUTPUT); //Initiates Motor Channel A pin
  pinMode(8, OUTPUT);  //Initiates Brake Channel A pin
  // UltrSonic sensor
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  
  #if 1==NANO
    myservo.attach(7);  // Use pin 7 in case of NANO
  #else
    myservo.attach(5);  // Use pin 5 in case of UNO
  #endif
  
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
  
  #ifdef CALIBRATE
    Calibrate_wheels();
  #endif
  
  
  // test motors
  #ifdef TEST_MODE
    // testing: making sure car parts are well

    while (true) {  // just for testing
      test_motors(); // human check - if wheels move
      servo_test();
      ultrasonic_test();
      // test ultrasonic" TBD
      delay(10000000); // wait for ever, so I could debug the results
    }
  #endif
  
}


//******************* LOOP ************************
void loop(){
  int dir,dist,tmp;
  
  stop();
  dist = scan();
  
   for (tmp = 0; tmp < SERVO_STEPS_NUM; tmp += 1) {
    // TBD array need to be SERVO_STEPS_NUM+1
    // enter for statment to ifdef
    
    #ifdef DEBUG
    Serial.print(dist_array[tmp]);
    Serial.print(" ");
    #endif 
   }
  
  
  dir=decide(dist);
  
  
  #ifdef DEBUG
    Serial.println(" ");
    Serial.print("dist= ");
    Serial.print(dist);
    Serial.print("  dir= ");
    Serial.println(dir);
    Serial.print("steps inc= ");
    Serial.println(SERVO_STEPS_INC);
  #endif 
  
  
  if (FORWARD==dir)
    go_forward();
  else {
    stop();
    delay(1000);
    go_backward();
  }
  
  /*
  if (BACKWARD==dir)
    go_backward();
  if (LEFT==dir)
    go_left();
  if (RIGHT==dir)
    go_right();
  */
  
  
  delay(3000);


  //Motor B backward @ half speed
//  digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);  //Establishes backward direction of Channel B
//  digitalWrite(MOTOR_RIGHT_BREAK_PIN, LOW);   //Disengage the Brake for Channel B
//  analogWrite(MOTOR_RIGHT_SPEED_PIN, 123);    //Spins the motor on Channel B at half speed
/*
  digitalWrite(13, LOW);  //Establishes backward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 123);    //Spins the motor on Channel B at half speed
*/
  
//  delay(3000);

  
//  digitalWrite(9, HIGH);  //Engage the Brake for Channel A
//  digitalWrite(9, HIGH);  //Engage the Brake for Channel B


//  delay(1000);
  
  
  //Motor A forward @ full speed
//  digitalWrite(12, LOW);  //Establishes backward direction of Channel A
//  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
//  analogWrite(3, 123);    //Spins the motor on Channel A at half speed
  
  //Motor B forward @ full speed
//  digitalWrite(13, HIGH); //Establishes forward direction of Channel B
//  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
//  analogWrite(11, 255);   //Spins the motor on Channel B at full speed
  
  
//  delay(3000);
  
  
//  digitalWrite(MOTOR_LEFT_BREAK_PIN, HIGH);  //Engage the Brake for Channel A
//  digitalWrite(MOTOR_RIGHT_BREAK_PIN, HIGH);  //Engage the Brake for Channel B
  
  
//  delay(1000);
  
}


// ********************** end of LOOP ***************************
void motor_go(int l_side,int l_dir_pin,int l_break_pin,int l_speed)
{

//  l_side:       left or right
//  l_dir_pin:    hardware direction pin
//  l_break_pin:  hardware break pin
//  l_speed:      speed of motor

  #ifdef DEBUG
    Serial.println("---------------------- ");
    Serial.print("in motor_go, side: ");
    Serial.println(l_side);
    Serial.print(" direction pin: ");
    Serial.println(l_dir_pin);
    Serial.print(" break pin: ");
    Serial.println(l_break_pin);
    Serial.print(" Speed: ");
    Serial.println(l_speed);
  #endif

  int left_or_right, brk_pin,spd_pin;
  if (RIGHT==l_side) {
    Serial.print("*** in RIGHT side ");
    left_or_right=MOTOR_RIGHT_DIR_PIN;
    brk_pin=MOTOR_RIGHT_BREAK_PIN;
    spd_pin=MOTOR_RIGHT_SPEED_PIN;
  }
  else {
    Serial.print("*** in LEFT side ");
    left_or_right=MOTOR_LEFT_DIR_PIN;
    brk_pin=MOTOR_LEFT_BREAK_PIN;
    spd_pin=MOTOR_LEFT_SPEED_PIN;
  }

  #ifdef DEBUG
    Serial.print("left_or_right: ");
    Serial.println(left_or_right);
    Serial.print("brk_pin: ");
    Serial.println(brk_pin);
    Serial.print("spd_pin: ");
    Serial.println(spd_pin);
    Serial.println("~~~~~~~~~~~~~~~~~~~ ");
  #endif
  
  digitalWrite(left_or_right, HIGH); //Establishes forward direction of Channel A
  digitalWrite(brk_pin, LOW);   //Disengage the Brake for Channel A
  analogWrite(spd_pin, l_speed);   //Spins the motor on Channel A at full speed

}


void go_forward()
{
  
  #ifdef DEBUG
    Serial.println("in go_forward");
  //  Serial.print(MOTOR_LEFT_DIR_PIN);
  //  Serial.print(" ");
  //  Serial.print(MOTOR_LEFT_BREAK_PIN);
  //  Serial.print(" ");
  //  Serial.println(MOTOR_LEFT_SPEED_PIN);
  #endif 

  //Motor A forward @ full speed

  digitalWrite(MOTOR_LEFT_DIR_PIN, HIGH); //Establishes forward direction of Channel A
  digitalWrite(MOTOR_LEFT_BREAK_PIN, LOW);   //Disengage the Brake for Channel A
  analogWrite(MOTOR_LEFT_SPEED_PIN, motor_left_speed);   //Spins the motor on Channel A at full speed
  
  //Motor B forward @ full speed
  digitalWrite(MOTOR_RIGHT_DIR_PIN, HIGH); //Establishes forward direction of Channel A
  digitalWrite(MOTOR_RIGHT_BREAK_PIN, LOW);   //Disengage the Brake for Channel A
  analogWrite(MOTOR_RIGHT_SPEED_PIN, motor_right_speed);   //Spins the motor on Channel A at full speed
}


void go_backward()
{
  
}


void go_right()
{
  
}


void go_left()
{
    //Motor A forward @ full speed
  digitalWrite(MOTOR_LEFT_DIR_PIN, HIGH); //Establishes forward direction of Channel A
  digitalWrite(MOTOR_LEFT_BREAK_PIN, LOW);   //Disengage the Brake for Channel A
  analogWrite(MOTOR_LEFT_SPEED_PIN, 255);   //Spins the motor on Channel A at full speed
  
  //Motor B forward @ half speed
  digitalWrite(MOTOR_RIGHT_DIR_PIN, HIGH); //Establishes forward direction of Channel A
  digitalWrite(MOTOR_RIGHT_BREAK_PIN, LOW);   //Disengage the Brake for Channel A
  analogWrite(MOTOR_RIGHT_SPEED_PIN, 123);   //Spins the motor on Channel A at half speed
}


void  stop() {
  
  #ifdef DEBUG
    Serial.println("in stop");
  #endif 
  
  
  digitalWrite(MOTOR_LEFT_BREAK_PIN, HIGH);  //Engage the Brake for Channel A
  digitalWrite(MOTOR_RIGHT_BREAK_PIN, HIGH);  //Engage the Brake for Channel B
}


int  scan() {
  int k=0,pos,read_dist,cnt;
  
  
  #ifdef DEBUG
    Serial.println("in scan");
  #endif 
  
  
  for (pos = 0; pos < SERVO_STEPS_NUM; pos += 1) {
  //for (pos = 0; pos <= 180; pos += 10) { // from 0 degrees to 180 degrees in steps of 10 degree
    myservo.write(pos*SERVO_STEPS_INC);              // tell servo to go to position in variable 'pos'
    delay(500);                       // waits 15ms for the servo to reach the position
    
    read_dist = readDistance();
    
    dist_array[pos]=read_dist;
    
// TBD read into array using k variable
// currently - return something
  #ifdef DEBUG
    Serial.print("distance: ");
    Serial.println(read_dist);
  #endif 

  }
  
  myservo.write(90); // bring to center
  return readDistance();    // TBD - tmp untill array is analysed t!!!  
}


int  decide(int l_dist){
  int decision;
  
  #ifdef DEBUG
    Serial.print("in decide.");
  #endif 
  
  if (decide_if_forward())
  	decision = FORWARD;
  else {
  	stop();
  	delay(300);
  	decision = BACKWARD;
  }
    
  return decision;
}


// will decide if the car can go forward. 
// if can't - returns 0 
// if can - return any other
int decide_if_forward() {
  int i;
  int fails = 0;
  int min_dist = 100; // over max possible value - TODO - TBD - add a const or define for that
  for (i = 0 ; i < SERVO_STEPS_NUM ; ++i) {
  	min_dist = min_dist > dist_array[i] ? dist_array[i] : min_dist;
  }
  
  if (STOP_RANGE >= min_dist)
  	return 0; // STOP. don't go forward
  
  return 1; // able to go forward
}
 
// void  go(){}


int readDistance() {
  
  int dist_t,duration_t;
  
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_t = pulseIn(echoPin, HIGH);

  // Calculating the distance
  dist_t = duration_t*0.034/2;
  
  
  #ifdef DEBUG
   //Serial.print("in read distance: ");
   //Serial.print("trigPin: ");
   //Serial.print(trigPin);
   //Serial.print("echoPin: ");
   //Serial.println(echoPin);
   //Serial.print("duration_t= ");
   //Serial.print(duration_t);
   //Serial.print("dist_t= ");
   //Serial.println(dist_t);
  #endif 
  
  return(dist_t);
}


void test_motors(){
    // void because of human validatio
    // maybe add hardware in the future to measure wheels movements
    
    Serial.println("**** Testing Motors");
    Serial.println("----Start LEFT,slow speed");
    motor_go(LEFT,HIGH,LOW, MOTOR_LEFT_MAX_SPEED/2);
    delay (2000);
    Serial.println("----Start LEFT,high speed");
    motor_go(LEFT,HIGH,LOW, MOTOR_LEFT_MAX_SPEED);
    delay (2000);
    stop();
    delay(2000);
    Serial.println("----Start RIGHT,slow speed");
    motor_go(RIGHT,HIGH,LOW, MOTOR_RIGHT_MAX_SPEED/2);
    delay (2000);
    Serial.println("----Start RIGHT,high speed");
    motor_go(RIGHT,HIGH,LOW, MOTOR_RIGHT_MAX_SPEED);
    delay (2000);
    stop();
    
    Serial.println("**** end testing motors");
  
}


void servo_test() {
  
  #ifdef DEBUG
    Serial.println("***** testing servo ");
  #endif
  myservo.write(0);
  delay(2000);
  myservo.write(90);
  delay(2000);
  myservo.write(180);
  delay(2000);
  myservo.write(90);
  delay(2000);
}


void ultrasonic_test() {
  #ifdef DEBUG
    Serial.println("***** Testing UltraSonic sensor");
    int tmp;
    for (int i=0;i<100;i++) {
      tmp=readDistance();
      Serial.println(tmp);
      delay(500);
    }
  #endif
}


void Calibrate_wheels() {
  // Calibrating the wheels, since some engines may run faster than others
  // How: 
  // HW: add potentiometer to allow extranal human input interface, keep connexted to laptop USB (?)
  // SW: 
  // use CALIBRATE to do it once
  // 1 - let the car go for 2 seconds, then changing 
  // 2 - define whicj wheel to claibrate (#define WHEEL). 
  //      slow down the faster one
  // 2 - wait 5 seconds to allow human change potentiometer 
  // 3 - back to (1) untill car goes straight
  // 4 - copy the value from the monitor and save it
  
    int calib_in; // value from calibration potentiometer

    Serial.print("**** Calibrating wheels");
    go_forward();
    delay(2000);
    
    while(true){
      // endless loop to allow calibration
      // once clibrated, unset CALIBRATE and re-start

      calib_in = read_calibrate_input();
      Serial.print("~~~~Calibration value read is: ");
      Serial.println(calib_in);
      set_wheel_speed(LEFT,calib_in);   // TBD - currently left wheel needs to be slower
      go_forward();
    }
  
}


int read_calibrate_input()
{
  // reading value from potrentiometer
  // done only in calibration mode in setup
  int val = analogRead(analog_in_pin);              // input value 0..1023
  int val1= map(val, 0, 1023, 0, 255);  // mapped value to 0..255
  
  Serial.print(val);
  Serial.print(" mapped to 0..255 value: ");
  Serial.println(val1);
  return val1; 
}


void set_wheel_speed(int l_wheel,int l_wheel_speed)
{
  Serial.println("---- in setting wheel speed");
  Serial.print("wheel : ");
  Serial.print(l_wheel);
  Serial.print(" Speed " );
  Serial.println(l_wheel_speed);
  
  if (LEFT==l_wheel)
    motor_left_speed= l_wheel_speed;
  else
    motor_right_speed= l_wheel_speed;
  
}

