/*************************************************************
Motor Shield 1-Channel DC Motor Demo
http://www.instructables.com/id/Arduino-Motor-Shield-Tutorial/
*************************************************************/
#include <Servo.h> 


#define MOTOR_A_DIR_PIN 12
#define MOTOR_B_DIR_PIN 13
#define MOTOR_A_BREAK_PIN 9
#define MOTOR_B_BREAK_PIN 8
#define MOTOR_A_SPEED_PIN 3
#define MOTOR_B_SPEED_PIN 11
#define SHORT_RANGE 20
#define STOP_RANGE 6
#define SERVO_STEPS_NUM 10
#define SERVO_STEPS_INC 180 / SERVO_STEPS_NUM
#define NANO 0 // make 0 in case of UNO

#define DEBUG 1

const int FORWARD   = 0;     // const value can't change
const int STOP      = 1;
const int BACKWARD  = 2;
const int GO        = 3;
const int LEFT      = 4;
const int RIGHT     = 5;
const int trigPin   = 2;  // Ultrasonic (Yellow)
const int echoPin   = 4;  // Ultrasonic (orange)

int dist_array[SERVO_STEPS_NUM];



Servo myservo;  // create servo object to control a servo
                // twelve servo objects can be created on most boards

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
  
}

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
  
  
  if (BACKWARD==dir)
    go_backward();
  if (LEFT==dir)
    go_left();
  if (RIGHT==dir)
    go_right();
    
    
  

  delay(3000);



  //Motor B backward @ half speed
//  digitalWrite(MOTOR_B_DIR_PIN, LOW);  //Establishes backward direction of Channel B
//  digitalWrite(MOTOR_B_BREAK_PIN, LOW);   //Disengage the Brake for Channel B
//  analogWrite(MOTOR_B_SPEED_PIN, 123);    //Spins the motor on Channel B at half speed
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
  
  
//  digitalWrite(MOTOR_A_BREAK_PIN, HIGH);  //Engage the Brake for Channel A
//  digitalWrite(MOTOR_B_BREAK_PIN, HIGH);  //Engage the Brake for Channel B
  
  
//  delay(1000);
  
}

// ********************** end of LOOP ***************************

void go_forward()
{
  
  #ifdef DEBUG
    Serial.println("in go_forward");
  //  Serial.print(MOTOR_A_DIR_PIN);
  //  Serial.print(" ");
  //  Serial.print(MOTOR_A_BREAK_PIN);
  //  Serial.print(" ");
  //  Serial.println(MOTOR_A_SPEED_PIN);
  #endif 
  



    //Motor A forward @ full speed

  digitalWrite(MOTOR_A_DIR_PIN, HIGH); //Establishes forward direction of Channel A
  digitalWrite(MOTOR_A_BREAK_PIN, LOW);   //Disengage the Brake for Channel A
  analogWrite(MOTOR_A_SPEED_PIN, 255);   //Spins the motor on Channel A at full speed
  

  
  
  //Motor B forward @ full speed
  digitalWrite(MOTOR_B_DIR_PIN, HIGH); //Establishes forward direction of Channel A
  digitalWrite(MOTOR_B_BREAK_PIN, LOW);   //Disengage the Brake for Channel A
  analogWrite(MOTOR_B_SPEED_PIN, 255);   //Spins the motor on Channel A at full speed




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
  digitalWrite(MOTOR_A_DIR_PIN, HIGH); //Establishes forward direction of Channel A
  digitalWrite(MOTOR_A_BREAK_PIN, LOW);   //Disengage the Brake for Channel A
  analogWrite(MOTOR_A_SPEED_PIN, 255);   //Spins the motor on Channel A at full speed
  
  //Motor B forward @ half speed
  digitalWrite(MOTOR_B_DIR_PIN, HIGH); //Establishes forward direction of Channel A
  digitalWrite(MOTOR_B_BREAK_PIN, LOW);   //Disengage the Brake for Channel A
  analogWrite(MOTOR_B_SPEED_PIN, 123);   //Spins the motor on Channel A at half speed
}





void  stop() {
  
  #ifdef DEBUG
    Serial.println("in stop");
  #endif 
  
  
  digitalWrite(MOTOR_A_BREAK_PIN, HIGH);  //Engage the Brake for Channel A
  digitalWrite(MOTOR_B_BREAK_PIN, HIGH);  //Engage the Brake for Channel B
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
  return 30;    // TBD - to REMOVE!!!  
}






int  decide(int l_dist){
  int decision;
  
  #ifdef DEBUG
    Serial.print("in decide.");
    Serial.print(" l_dist=");
    Serial.println(l_dist);
  #endif 
  
  
  
  
  if (l_dist<STOP_RANGE)
    decision = STOP;
  else  if (l_dist<SHORT_RANGE)
    decision = LEFT;
  else
    decision = FORWARD;
    
  return decision;
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
  
  
  //#ifdef DEBUG
   //Serial.print("in read distance: ");
   //Serial.print("trigPin: ");
   //Serial.print(trigPin);
   //Serial.print("echoPin: ");
   //Serial.println(echoPin);
    // Serial.print("duration_t= ");
    // Serial.print(duration_t);
    //Serial.print("dist_t= ");
    // Serial.println(dist_t);
  //#endif 
  
  return(dist_t);
}