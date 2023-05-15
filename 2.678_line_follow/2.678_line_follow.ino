// Pololu #713 motor driver pin assignments
const int PWMA=11; // Pololu drive A
const int AIN2=10;
const int AIN1 =9;
const int STDBY=8;
const int BIN1 =7; // Pololu drive B
const int BIN2 =6;
const int PWMB =5;

const int left_w = 190;
const int left_b = 921;
const int mid_w = 84;
const int mid_b = 845;
const int right_w = 130;
const int right_b = 886;

const int normal_time = 0;
const int thin_time = 0;
const int qr_time = 0;
const int tight_wiggle_time = 0;
const int broken_time = 0;
const int right_angle_loop_time = 0;
const int parallel_time = 0;
const int infinite_loop_time = 0;

void setup()
{
  pinMode(PWMA , OUTPUT);
  pinMode(AIN1 , OUTPUT);
  pinMode(AIN2 , OUTPUT);
  pinMode(BIN1 , OUTPUT);
  pinMode(BIN2 , OUTPUT);
  pinMode(PWMB , OUTPUT);
  pinMode(STDBY , OUTPUT);
  digitalWrite(STDBY , HIGH);

  Serial.begin(9600);
}

void loop()
{
  int sensorR = analogRead(A0);
  int sensorM = analogRead(A1);
  int sensorL = analogRead(A2);

  Serial.print(sensorL);
  Serial.print(",");

  Serial.print(sensorM);
  Serial.print(",");

  Serial.println(sensorR);

  line_follower_normal(sensorL, sensorM, sensorR);
  delay(30);
}

void motorWrite(int spd, int pin_IN1 , int pin_IN2 , int pin_PWM)
{
  if (spd < 0)
  {
    digitalWrite(pin_IN1 , HIGH); // go one way
    digitalWrite(pin_IN2 , LOW);
  }
  else
  {
    digitalWrite(pin_IN1 , LOW); // go the other way
    digitalWrite(pin_IN2 , HIGH);
  }
  analogWrite(pin_PWM , abs(spd));
}
// your drive() function goes here!
void drive(int speedL, int speedR)
{
  // drive left wheel
  motorWrite(speedR, AIN1, AIN2, PWMA);
  motorWrite(-speedL, BIN1, BIN2, PWMB);
}

enum Direction
{
  LEFT,
  RIGHT,
  MID
};

const int TURN_FORWARD_SPEED_INIT = 80;
const int TURN_BACKWARD_SPEED_INIT = 60;
const float TURN_DECAY_RATE = 1.2;
const int N_TURNS_THRES = 8;
const int BLACK_THRES = 450;

void line_follower_normal(int sensorL, int sensorM, int sensorR)
{
  bool left_white = sensorL < BLACK_THRES;
  bool right_white = sensorR < BLACK_THRES;
  bool mid_white = sensorM < BLACK_THRES;
  int drive_speed = 200;
  int turn_forward_speed = TURN_FORWARD_SPEED_INIT;
  int turn_backward_speed = TURN_BACKWARD_SPEED_INIT;
  Direction last_dir = MID;
  Direction dir = MID;
  int n_turns = 0;

  if (!mid_white)
  {
    if (left_white && right_white)
    {
      dir = MID;
    }
    else if (left_white)
    {
      dir = RIGHT;
    }
    else if (right_white)
    {
      dir = LEFT;
    }
    else
    {
      dir = MID;
    }
  }
  else
  {
    if (!right_white)
    {
      dir = RIGHT;
    }
    else if (!left_white)
    {
      dir = LEFT;
    }
    else
    {
    }
  }

  if (last_dir != dir)
  {
    ++n_turns;
  }
  if (dir == MID)
  {
    turn_forward_speed = TURN_FORWARD_SPEED_INIT;
    turn_backward_speed = TURN_BACKWARD_SPEED_INIT;
    n_turns == 0;
  }
  if (n_turns > N_TURNS_THRES)
  {
    turn_forward_speed = turn_forward_speed / TURN_DECAY_RATE;
    turn_backward_speed = turn_backward_speed / TURN_DECAY_RATE;
  }
  last_dir = dir;

  switch (dir)
  {
    case LEFT:
      drive(-turn_backward_speed, turn_forward_speed);
      break;
    case RIGHT:
      drive(turn_forward_speed, -turn_backward_speed);
      break;
    case MID:
      drive(drive_speed, drive_speed);
      break;
  }
}
