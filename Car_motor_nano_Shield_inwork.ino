/*************************************************************
Arduino car
*************************************************************/
#include <Servo.h>

#include "car_motors.h"

/*
#include <string>
using std::string;
*/

#define MOTOR_LEFT_DIR_PIN 12
#define MOTOR_LEFT_BREAK_PIN 9
#define MOTOR_LEFT_SPEED_PIN 3
#define MOTOR_RIGHT_DIR_PIN 13
#define MOTOR_RIGHT_BREAK_PIN 8
#define MOTOR_RIGHT_SPEED_PIN 11
#define servo_forwrd_pin  5      // for forward driving
#define servo_bckwrd_pin  6  // for reverse driving

#define SHORT_RANGE 30
#define STOP_RANGE 20
#define SERVO_STEPS_NUM 10
#define SERVO_STEPS_INC 180 / SERVO_STEPS_NUM

#define SAME_DIRECTION  0
#define CNG_DIRECTION   1

#define MOTOR_LEFT_MAX_SPEED 255 // max is 255. should be lower if calibrated needed
#define MOTOR_RIGHT_MAX_SPEED 255 // max is 255. should be lower if calibrated needed

#define NANO 0 // make 0 in case of UNO; 1 in case of Nano
#define UNo  1

#define DEBUG 1
#define CALIBRATE 0   // for calibration only, one time. 0 for normal
#define TEST_MODE 0   // check if HW is Ok. 0 for normal

#ifdef DEBUG  // makes the code more readable
 #define DEBUG_PRINT(x)  Serial.print (x)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else         // not exists if DEBUG is not defined
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif



const int FORWARD   = 0;     // const value can't change
const int STOP      = 1;
const int BACKWARD  = 2;
const int GO        = 3;
const int LEFT      = 4;
const int RIGHT     = 5;
const int f_trigPin   = 2;  // Front Ultrasonic (Yellow)
const int f_echoPin   = 4;  // Front Ultrasonic (orange)
const int b_trigPin   =10;  // Back Ultrasonic ()
const int b_echoPin   = 7;  // Back Ultrasonic ()

const int analog_in_pin = 2 ; // potentiometer for claibration

int dist_array[SERVO_STEPS_NUM];
int motor_left_speed = MOTOR_LEFT_MAX_SPEED;
int motor_right_speed = MOTOR_RIGHT_MAX_SPEED;
int car_direction;  // Can be forward or backwards. TBD other directions?
bool cng_dir ;      // kee[ [revious direction or change?

//  pre-setp, create servos and motors
Servo F_servo;  // create servo object to control the front servo
Servo B_servo;  // create servo object to control the back servo
Motor motor_left(LEFT, MOTOR_LEFT_DIR_PIN, MOTOR_LEFT_BREAK_PIN, MOTOR_LEFT_SPEED_PIN, MOTOR_LEFT_MAX_SPEED);
Motor motor_right(RIGHT, MOTOR_RIGHT_DIR_PIN, MOTOR_RIGHT_BREAK_PIN, MOTOR_RIGHT_SPEED_PIN, MOTOR_RIGHT_MAX_SPEED);


//******************* SETUP ************************
void setup() {

  //Setup Channel A (Left)
  pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT); //Initiates Motor Channel A pin
  pinMode(MOTOR_LEFT_BREAK_PIN, OUTPUT); //Initiates Brake Channel A pin
  //Setup Channel B (Right)
  pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT); //Initiates Motor Channel A pin
  pinMode(MOTOR_RIGHT_BREAK_PIN , OUTPUT);  //Initiates Brake Channel A pin
  // UltrSonic sensor
  pinMode(f_trigPin, OUTPUT);
  pinMode(f_echoPin, INPUT);
  pinMode(b_trigPin, OUTPUT);
  pinMode(b_echoPin, INPUT);
  // create Front and Back Servos entities
  F_servo.attach(servo_forwrd_pin);
  B_servo.attach(servo_bckwrd_pin);
  car_direction = FORWARD;
  cng_dir = false;

  #if DEBUG
    Serial.begin(9600);
  #endif

  #if CALIBRATE
    Calibrate_wheels();
  #endif

servo_test(); // always test servos. Helps to determine reset

  // test: making sure car parts are well
  #if TEST_MODE
    while (true) {  // just for testing
      test_motors(); // human check - if wheels move
      servo_test();
      ultrasonic_test(FORWARD);
      ultrasonic_test(BACKWARD);
      delay(10000000); // wait for ever, so I could debug the results
    }
  #endif
}


//******************* LOOP ************************
void loop(){
  int new_dir,dist,tmp;

  if (cng_dir) {
    if (FORWARD == car_direction)
      car_direction = BACKWARD;
    else
      car_direction = FORWARD;
    cng_dir = false;
    stop();
    delay(1000);
  }

  if ( FORWARD == car_direction)   // keep going same direction as before
    go_forward();
  else
    go_backward();

  dist = scan(car_direction);  // Scan if no obsticle
  // stop();
  //delay(500); // TBD - remove or shorten
  //dir=decide(dist);
  if ( CNG_DIRECTION == decide(dist) )
    cng_dir = true;

  DEBUG_PRINTLN(" ");
  DEBUG_PRINT("in loop, dist= ");
  DEBUG_PRINT(dist);
  DEBUG_PRINT("  car_direction= ");
  DEBUG_PRINTLN(car_direction);

}
// ********************** end of LOOP ***************************

void go_forward()
{
  DEBUG_PRINTLN(">>> in go_forward");
  motor_left.GoForward(motor_left.Get_Speed());
  motor_right.GoForward(motor_right.Get_Speed());
}


void go_backward()
{
  DEBUG_PRINTLN("<<< in go_backward");
  motor_left.GoBackward(MOTOR_LEFT_MAX_SPEED);
  motor_right.GoBackward(MOTOR_RIGHT_MAX_SPEED);
}


void go_right()
{
  DEBUG_PRINTLN(">>> in go_right");
}


void go_left()
{
  DEBUG_PRINTLN(">>> in go_left");
  motor_left.GoForward(MOTOR_LEFT_MAX_SPEED/2); // slower than right. to turn left
  motor_right.GoForward(MOTOR_RIGHT_MAX_SPEED); // faster than left. to go left
}


void  stop() {
  DEBUG_PRINTLN(">>> in stop");
	motor_left.Stop();
  motor_right.Stop();
}


int  scan(int l_dir) {
  // l_dir can be FORWARD or BACKWARD
  int k=0,pos,read_dist,cnt,i;
  int scan_angle = 30 ; // TND make it a define
  int scan_steps = 15 ; // TND make it a define (exists)
  int max_dist = 120 ; // I was sure such const exists.. TBD #define
  int min ; // temp variable
  Servo current_servo;

  DEBUG_PRINTLN(">>> in scan");

  if ( FORWARD == l_dir )  // use the front or back servo according to direction
      current_servo = F_servo;
  else
      current_servo = B_servo;

  min=max_dist;
  cnt=0;

  for (pos = 0; pos <SERVO_STEPS_NUM ; pos++)
    dist_array[pos] = max_dist ; // peset the array

  for (pos = 90 - scan_angle; pos <= scan_angle + 90 ; pos += scan_steps) {
    current_servo.write(pos); // bring to position
    i = readDistance(l_dir);
    if ( i < STOP_RANGE )
      return i;     // if too close to obsticle, go back immediately
    dist_array[cnt] = i;
    if ( dist_array[cnt] < min )
      min = dist_array[cnt];
    delay(300); // to allow set for the servo
    cnt++;
  };

  F_servo.write(90); // bring to center
  B_servo.write(90); // bring to center

  return min ; // the shortest distance detected
}


int  decide(int l_dist) {
  int decision;

  //#if DEBUG
  //  Serial.print("in decide.");
  //#endif

DEBUG_PRINTLN("in decide function");

  // if (decide_if_forward()) TBD - use one measure now. later full scan.
  if (STOP_RANGE < l_dist)
  	decision = SAME_DIRECTION;
  else {
  	stop();
  	delay(300);
  	decision = CNG_DIRECTION;
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


int readDistance(int l_dir) {
  // l_dir can be FORWARD or BACKWARD
  int trigPin,echoPin;  // local variable to hold the HW pin of ultraosnice sensor
  int dist_t,duration_t;

  // set the trig pin according to the required direction
  if ( FORWARD == l_dir ) {
    trigPin=f_trigPin;
    echoPin=f_echoPin;
  }
  else {
    trigPin=b_trigPin;
    echoPin=b_echoPin;
  }

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
  return(dist_t);
}


void test_motors(){
    // void because of human validatio
    // maybe add hardware in the future to measure wheels movements

    DEBUG_PRINTLN("Testing Motors");
    DEBUG_PRINTLN("----Start LEFT,slow speed");

    //motor_go(LEFT,HIGH,LOW, MOTOR_LEFT_MAX_SPEED/2);
    motor_left.GoForward(MOTOR_LEFT_MAX_SPEED/2);
    delay (2000);
    DEBUG_PRINTLN("----Start LEFT,fast speed");
    //motor_go(LEFT,HIGH,LOW, MOTOR_LEFT_MAX_SPEED);
    motor_left.GoForward(MOTOR_LEFT_MAX_SPEED);
    delay (2000);
    stop();
    delay(2000);
    DEBUG_PRINTLN("----Start RIGHT,slow speed");
    //motor_go(RIGHT,HIGH,LOW, MOTOR_RIGHT_MAX_SPEED/2);
    motor_right.GoForward(MOTOR_RIGHT_MAX_SPEED/2);
    delay (2000);
    DEBUG_PRINTLN("----Start RIGHT,slow speed");
    //motor_go(RIGHT,HIGH,LOW, MOTOR_RIGHT_MAX_SPEED);
    motor_right.GoForward(MOTOR_RIGHT_MAX_SPEED);
    DEBUG_PRINTLN("----Start RIGHT,high speed");
    delay (2000);
    stop();

    DEBUG_PRINTLN("**** end testing motors");
}


void servo_test() {
  DEBUG_PRINTLN("***** testing servo ");
  F_servo.write(0);
  B_servo.write(0);
  delay(2000);
  F_servo.write(90);
  B_servo.write(90);
  // myservo.write(90);
  delay(2000);
  F_servo.write(180);
  B_servo.write(180);
  //myservo.write(180);
  delay(2000);
  F_servo.write(90);
  B_servo.write(90);
  //myservo.write(90);
  delay(2000);
}


void ultrasonic_test(int l_dir) {
  // l_dir can be FORWARD or BACKWARD
  #if DEBUG
    if (FORWARD == l_dir)
      DEBUG_PRINTLN("***** FRONT Testing distance sensor");
    else
      DEBUG_PRINTLN("***** BACK  Testing distance sensor");

    int tmp;
    for (int i = 0; i < 20; i++) {
      tmp = readDistance(l_dir);
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
    DEBUG_PRINTLN("**** Calibrating wheels");
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
  int val1 = map(val, 0, 1023, 0, 255);  // mapped value to 0..255

  //Serial.print(val);
  //Serial.print(" mapped to 0..255 value: ");
  //Serial.println(val1);
  return val1;
}


void set_wheel_speed(int l_wheel,int l_wheel_speed)
{
  //Serial.println("---- in setting wheel speed");
  //Serial.print("wheel : ");
  //Serial.print(l_wheel);
  //Serial.print(" Speed " );
  //Serial.println(l_wheel_speed);

  if (LEFT == l_wheel)
    motor_left_speed = l_wheel_speed;
  else
    motor_right_speed = l_wheel_speed;
}
