class Motor {
public:
  Motor(int m_name, int dir_p, int brk_p, int spd_p, int max_spd);
  ~Motor() {  }

  void GoForward(int l_speed);
  void GoBackward(int l_speed);
  void Stop();
  int Get_Speed();
private:
	const int name;
  const int dir_pin;
  const int break_pin;
  const int speed_pin;
  const int max_speed;

  int speed;

  // object can't be copied
  Motor(const Motor& m);
  Motor& operator=(const Motor& m);
};

Motor::Motor(int m_name, int dir_p, int brk_p, int spd_p, int max_spd):
	name(m_name),
	dir_pin(dir_p),
	break_pin(brk_p),
	speed_pin(spd_p),
	max_speed(max_spd)
{
	speed = max_spd;
}


void Motor::GoForward(int l_speed)
{
//  l_speed:      speed of motor

	#if DEBUG
    Serial.println("---------------------- ");
    Serial.print("in motor side: ");
    Serial.println(name);
    Serial.print(" direction pin: ");
    Serial.println(HIGH);
    Serial.print(" break pin: ");
    Serial.println(LOW);
    Serial.print(" Speed: ");
    Serial.println(l_speed);

    Serial.print("brk_pin: ");
    Serial.println(break_pin);
    Serial.print("spd_pin: ");
    Serial.println(speed_pin);
    Serial.println("~~~~~~~~~~~~~~~~~~~ ");
  #endif

  digitalWrite(dir_pin, HIGH); //set direction forward
  digitalWrite(break_pin, LOW);   //Disengage the Brake
  analogWrite(speed_pin, l_speed);   //Spins the motor l_speed speed
}

void Motor::GoBackward(int l_speed)
{
	//  l_speed:      speed of motor

	#if DEBUG
    Serial.println("---------------------- ");
    Serial.print("in motor side: ");
    Serial.println(name);
    Serial.print(" direction pin: ");
    Serial.println(LOW);
    Serial.print(" break pin: ");
    Serial.println(LOW);
    Serial.print(" Speed: ");
    Serial.println(l_speed);

    Serial.print("brk_pin: ");
    Serial.println(break_pin);
    Serial.print("spd_pin: ");
    Serial.println(speed_pin);
    Serial.println("~~~~~~~~~~~~~~~~~~~ ");
  #endif

	digitalWrite(dir_pin, LOW); //set direction backwards
	digitalWrite(break_pin, LOW);   //Disengage the Brake
  analogWrite(speed_pin, l_speed);   //Spins the motor l_speed speed
}


void Motor::Stop()
{
	digitalWrite(break_pin, HIGH);  //Engage the Brake
}


int Motor::Get_Speed()
{
	return speed;
}
