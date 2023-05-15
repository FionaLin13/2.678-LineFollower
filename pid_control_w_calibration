// Pololu #713 motor driver pin assignments
const int PWMA=11; // Pololu drive A
const int AIN2=10;
const int AIN1 =9;
const int STDBY=8;
const int BIN1 =7; // Pololu drive B
const int BIN2 =6;
const int PWMB =5;

const int normal_time = 0;
const int thin_time = 0;
const int qr_time = 0;
const int tight_wiggle_time = 0;
const int broken_time = 0;
const int right_angle_loop_time = 0;
const int parallel_time = 0;
const int infinite_loop_time = 0;
QTRSensors qtr;

const uint8_t SensorCount = 3;
uint16_t sensorValues[SensorCount];

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

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  // int sensorR = analogRead(A0);
  // int sensorM = analogRead(A1);
  // int sensorL = analogRead(A2);

  // Serial.print(sensorL);
  // Serial.print(",");

  // Serial.print(sensorM);
  // Serial.print(",");

  // Serial.println(sensorR);

  // line_follower_normal(sensorL, sensorM, sensorR);
  // delay(30);

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  delay(250);
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
  motorWrite(speedR, AIN1, AIN2, PWMA);
  motorWrite(-speedL, BIN1, BIN2, PWMB);
}
