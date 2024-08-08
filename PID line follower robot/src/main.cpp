/* Include essential libraries for hardware interaction: QTRSensors for line sensors, 
 Wire for I2C communication, LiquidCrystal_I2C for LCD control, and MPU6050 for gyroscope/accelerometer data.*/
#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BusIO_Register.h>

// Initialize QTRSensors object and define an array to store sensor readings for 8 line sensors.
QTRSensors qtr ; 
const uint8_t SensorCount  = 8;
uint16_t sensorValues[SensorCount];

// Initialize the library with the I2C address and LCD size
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define constants for motor speed limits and base speeds for both motors.
const int maxspeeda = 150 ; 
const int maxspeedb = 150 ; 
const int minspeeda = 0 ; 
const int minspeedb = 0 ; 
const int basespeeda = 150 ; 
const int basespeedb = 150 ; 

// Declare variables for PID control parameters and their values, including integral, proportional, and derivative components.
int I ; 
int P ;
int D ; 
float Ki ; 
float Kp ;
float Kd ; 
float Pvalue; 
float Ivalue; 
float Dvalue; 
float multiP = 1 ; 
float multiI = 1 ; 
float multiD = 1 ; 

// Define pin assignments for motor control: phase and enable pins for two motors.
int aphase =1; 
int aenbl =2;
int bphase =3; 
int benbl =4;

// Define pin assignments for buttons used to adjust and reset PID values(for the LCD Display).
const int buttonPinIncrement = 40; 
const int buttonPinDecrement = 41; 
const int buttonPinReset = 42;     
const int buttonPinNext = 43;    

// Initialize arrays to hold the labels for PID parameters and their current values.
String NounValues[8] = {"Kp : ", "Kd : ", "Ki : ", "multiP : ", "multiD : ", "multiI : ", "basespeeda : ", "basespeedb : "};
float Values[8] = {Kp, Kd, Ki, multiP, multiD, multiI, basespeeda, basespeedb};

const uint16_t threshold = 500; 
int val, cnt = 0;
int LastError = 0;
bool Test = true ; 
// put function definitions here:

// Calibrate the QTR sensors by taking 400 calibration samples.
void Calibration(){
  for(int i=0 ; i<400 ; i++) 
    qtr.calibrate();
}

// Set the motors to move forward with specified speeds and brake by setting phase pins low.
void forward_brake(int posa ,int posb)
{
  digitalWrite(aphase , 0) ; 
  digitalWrite(bphase , 0) ; 
  analogWrite(aenbl , posa) ;
  analogWrite(benbl , posb) ;
}

// Set the motors to turn right with specified speeds and brake by setting phase pins accordingly.
void right_brake(int posa ,int posb)
{
  digitalWrite(aphase , 0) ; 
  digitalWrite(bphase , 1) ; 
  analogWrite(aenbl , posa) ;
  analogWrite(benbl , posb) ;
}

// Set the motors to turn left with specified speeds and brake by setting phase pins accordingly.
void left_brake(int posa ,int posb)
{
  digitalWrite(aphase , 1) ; 
  digitalWrite(bphase , 0) ; 
  analogWrite(aenbl , posa) ;
  analogWrite(benbl , posb) ;
}

// Control motor speeds and direction based on input values: forward, right, or left, adjusting for negative speed values.
void speedcontrol(int mota , int motb){
  if (mota >= 0 && motb >= 0){
    forward_brake(mota, motb);
  }
  if (mota < 0 && motb >= 0){
    mota = 0 - mota;
    right_brake(mota, motb);
  }
  if (mota >= 0 && motb < 0){
    motb = 0 - motb;
    left_brake(mota, motb);
  }
}

// Perform PID control to adjust motor speeds based on the error, updating P, I, and D values, and ensuring speeds are within limits.
void PID_CONTROl(int Error)
{
  P=Error ; 
  D=Error-LastError; 
  I = I + Error ; 
  LastError = Error ; 

  Pvalue = (Kp/pow(10,multiP))*P;
  Ivalue = (Ki/pow(10,multiI))*I;
  Dvalue = (Kd/pow(10,multiD))*D;

  int motorSpeedChange = Pvalue + Ivalue + Dvalue;
  int motorSpeedA = basespeeda + motorSpeedChange;
  int motorSpeedB = basespeedb - motorSpeedChange;

  if (motorSpeedA > maxspeeda){
    motorSpeedA = maxspeeda ; 
  }
  if (motorSpeedB > maxspeedb){
    motorSpeedB = maxspeedb ; 
  }
  if (motorSpeedA < minspeeda){
    motorSpeedA=minspeeda ; 
  }
  if (motorSpeedB < minspeedb){
    motorSpeedB=minspeedb ; 
  }

  speedcontrol(motorSpeedA, motorSpeedB);
}

// Read the line sensor values to determine the robot's position and adjust motor control based on sensor readings and PID calculations.
void robot_control(){
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int Error = 3500 - positionLine;

  int cnt=0 ; 
  float sum=0 ; 
  for(int i=0 ; i<8 ; i++){
    if(sensorValues[i] >= threshold){
      cnt++ ;
      sum+=1 ; 
    }
  }

  if (cnt >= 3){
    int motorspeedA=0 ; 
    int motorspeedB=0 ; 
    int val = sum/cnt ; 

    if(val < 3.5){
      right_brake(100,100) ; 
    }
    if(val > 3.5){
      left_brake(100,100) ; 
    }
    if(val == 3.5){
      cnt = cnt/2;
      uint16_t mini = 1000;
      uint8_t minpos = 0 ; 
      for (int i = 4 - cnt; i <= 3 + cnt; i++){
        if (mini > sensorValues[i]){
          mini = sensorValues[i];
          minpos = i ; 
        } 
      }
      if(minpos < 3.5){
        right_brake(100, 100);
      }
      if(minpos > 3.5){
        left_brake(100, 100);
      }
    }
    else 
      PID_CONTROl(Error);
  }
}

/* Display and update PID parameter values on the LCD based on button inputs for incrementing,
 decrementing, resetting, and navigating through settings.*/
void setValues(){
  bool buttonIncrementState = digitalRead(buttonPinIncrement);
  bool buttonDecrementState = digitalRead(buttonPinDecrement);
  bool buttonResetState = digitalRead(buttonPinReset);
  bool buttonNextState = digitalRead(buttonPinNext);
  for(int i=0 ; i<7 ; i++)
  {
    while(Test == true){
      lcd.init();
      lcd.backlight();
      lcd.setCursor(0, 0);
      lcd.print(NounValues[i]);
      // Increase value if increment button is pressed
      if (buttonIncrementState == HIGH) {
        Values[i]++;
        delay(200); // Debounce delay
      }
      // Decrease value if decrement button is pressed
      if (buttonDecrementState == HIGH) {
        Values[i]--;
        delay(200); // Debounce delay
      }
      // Reset value if reset button is pressed
      if (buttonResetState == HIGH) {
        Values[i] = 0;
        delay(200); // Debounce delay
      }
      if (buttonNextState == HIGH)
        {
          Test = false ;
        }
      lcd.setCursor(0, 1); // Move cursor to the second row
      lcd.print(NounValues[i]);
      lcd.print(Values[i]);
      lcd.print("   "); // Clear any extra digits
      delay(100); // Update delay
    
    }
  }
}

void setup() {
  Serial.begin(9600); 
  qtr.setTypeRC(); 
  qtr.setSensorPins((const  uint8_t[]){10, 11, 12, 14, 15, 16, 18, 19}, SensorCount);
  // Set pin modes for motor control and button inputs: motor control pins as OUTPUT and button pins as INPUT.
  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  pinMode(buttonPinIncrement, INPUT);
  pinMode(buttonPinDecrement, INPUT);
  pinMode(buttonPinReset, INPUT);
  pinMode(buttonPinNext, INPUT);

  setValues(); 
  delay(500);
  Calibration();
  forward_brake(0, 0);
}

void loop() {
  robot_control();
}
