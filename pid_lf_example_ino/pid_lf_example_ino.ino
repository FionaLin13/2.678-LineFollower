/*
 * File name: PID_LF_example
 * 
 * Hardware requirements: an Arduino Pro Mini
 *                        a QTR-8RC Reflectance Sensor Array
 *                        a DRV8835 Dual Motor Driver Carrier 
 *                        
 * Description: The basic PID control system implemented with 
 *              the line follower with the specified hardware. 
 *              The robot can follow a black line on a white surface 
 *              (or vice versa). 
 * Related Document: See the written documentation or the LF video from
 *                   Bot Reboot.
 *                   
 * Author: Bot Reboot
 */

#include <QTRSensors.h> //Make sure to install the library

/*************************************************************************
* Sensor Array object initialisation 
*************************************************************************/
QTRSensors qtr;
const uint8_t SensorCount = 3;
uint16_t sensorValues[SensorCount];

/*************************************************************************
* PID control system variables 
*************************************************************************/
float Kp = 0.08; //related to the proportional control term; //0.08 works until u-turn
              //change the value by trial-and-error (ex: 0.07).
float Ki = 0; //related to the integral control term; 
              //change the value by trial-and-error (ex: 0.0008).
float Kd = 0; //related to the derivative control term; 
              //change the value by trial-and-error (ex: 0.6).
int P;
int I;
int D;

/*************************************************************************
* Global variables
*************************************************************************/
int lastError;
boolean onoff = false;
int motorspeeda;
int motorspeedb;

unsigned long timer;
int time_uturn = 33000; //millisec
int time_breaks = 35000;
int have_run = 0;

/*************************************************************************
* Motor speed variables (choose between 0 - no speed, and 255 - maximum speed)
*************************************************************************/
const uint8_t maxspeeda = 250;
const uint8_t maxspeedb = 250;
uint8_t basespeeda = 120;
uint8_t basespeedb = 120;

/*************************************************************************
* MOTOR pins declaration
*************************************************************************/
const int PWMA=11; // Pololu drive A
const int AIN2=10;
const int AIN1 =9;
const int STDBY=8;
const int BIN1 =7; // Pololu drive B
const int BIN2 =6;
const int PWMB =5;


boolean Ok = false;
/*************************************************************************
* Function Name: setup
**************************************************************************
* Summary:
* This is the setup function for the Arduino board. It first sets up the 
* pins for the sensor array and the motor driver. Then the user needs to 
* slide the sensors across the line for 10 seconds as they need to be 
* calibrated. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void setup() {
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2}, SensorCount);
  
  pinMode(PWMA , OUTPUT);
  pinMode(AIN1 , OUTPUT);
  pinMode(AIN2 , OUTPUT);
  pinMode(BIN1 , OUTPUT);
  pinMode(BIN2 , OUTPUT);
  pinMode(PWMB , OUTPUT);
  pinMode(STDBY , OUTPUT);
  digitalWrite(STDBY , HIGH);
  
  
  pinMode(LED_BUILTIN, OUTPUT);

  while (Ok == false) { // the main function won't start until the robot is calibrated
      calibration(); //calibrate the robot for 10 seconds
      Ok = true;
      }
  drive(0, 0); //stop the motors

  Serial.begin(9600);

  timer = millis();

}

/*************************************************************************
* Function Name: calibration
**************************************************************************
* Summary:
* This is the calibration function for the QTR-8RC Reflectance Sensor Array. 
* The function calls the method 'qtr.calibrate()' offered by the imported 
* library. For approx. 10 seconds, each of the 8 sensors will calibrate with
* readings from the track. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void calibration() {
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

/*************************************************************************
* Function Name: PID_control
**************************************************************************
* Summary: 
* This is the function of the PID control system. The distinguishing 
* feature of the PID controller is the ability to use the three control 
* terms of proportional, integral and derivative influence on the controller 
* output to apply accurate and optimal control. This correction is applied to
* the speed of the motors, which should be in range of the interval [0, max_speed],
* max_speed <= 255. 
* 
* Parameters:
* none
* 
* Returns:
*  none
*************************************************************************/
void PID_control(int position_calc) {
//  uint16_t position = qtr.readLineBlack(sensorValues); //read the current position
//  int error = 1000 - position; //3500 is the ideal position (the centre)

  int error = position_calc;

  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;
  int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                       //needed to be applied to the speed
  Serial.println(motorspeed);
  int motorspeeda = basespeeda - motorspeed;
  int motorspeedb = basespeedb + motorspeed;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = 0;
  }
  if (motorspeedb < 0) {
    motorspeedb = 0;
  } 
  drive(motorspeeda, motorspeedb);
}

/*************************************************************************
* Function Name: loop
**************************************************************************
* Summary:
* This is the main function of this application. When the start button is
* pressed, the robot will toggle between following the track and stopping.
* When following the track, the function calls the PID control method. 
* 
* Parameters:
*  none
* 
* Returns:
*  none
*************************************************************************/
void loop() {

  int sensorL = sensorValues[2];
  int sensorM = sensorValues[1];
  int sensorR = sensorValues[0];

//  Serial.print(sensorL);
//  Serial.print(",");
//
//  Serial.print(sensorM);
//  Serial.print(",");
//
//  Serial.println(sensorR);

  
  int position_calc = sensorL - sensorR;
  int last_pos_calc = 0;
  
  if (Ok == true) {
    if (sensorL < 700 && sensorR <=700 && sensorM < 700 && timer < time_breaks) { //all white condition for u turn
      drive(100,-100);
      basespeeda = 90;
      basespeedb = 90;
      have_run = 1;
      Serial.println("still in white");
    }
    else if (sensorL < 700 && sensorR <=700 && sensorM < 700) {
      basespeeda = 90;
      basespeedb = 90;
      boolean left_turned = false;
      boolean right_turned = false;
      drive(-100, 100);
      delay(3500);
      left_turned = true;
      drive(100, -100);
      delay(7000);
      right_turned = true;
      if (left_turned && right_turned) {
        drive(100,100);
        delay(5000\
        );
      }
    }
    else{
      PID_control(position_calc);
      last_pos_calc = position_calc;
      Serial.println(last_pos_calc);
      basespeeda = 120;
      basespeedb = 120;
    }
     
  }
  else {
    drive(0,0); //stop the motors
  }
  qtr.read(sensorValues);

  // print the sensor values as numbers from 0 to 1023, where 0 means maximum
  // reflectance and 1023 means minimum reflectance
//  for (uint8_t i = 0; i < SensorCount; i++)
//  {
//    Serial.print(sensorValues[i]);
//    Serial.print('\t');
//  }
//  Serial.println();


}

/*************************************************************************
* Function Name: forward_brake
**************************************************************************
* Summary:
* This is the control interface function of the motor driver. As shown in
* the Pololu's documentation of the DRV8835 motor driver, when the MODE is 
* equal to 1 (the pin is set to output HIGH), the robot will go forward at
* the given speed specified by the parameters. The phase pins control the
* direction of the spin, and the enbl pins control the speed of the motor.
* 
* A warning though, depending on the wiring, you might need to change the 
* aphase and bphase from LOW to HIGH, in order for the robot to spin forward. 
* 
* Parameters:
*  int posa: int value from 0 to 255; controls the speed of the motor A.
*  int posb: int value from 0 to 255; controls the speed of the motor B.
* 
* Returns:
*  none
*************************************************************************/

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
