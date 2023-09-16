#include <PID_v1.h>

//initialize arduino pins
const int IN1 = 4;
const int IN2 = 5;
const int CHA = 2;
const int CHB = 3;
const int ENA = 6;

//to store user input
String readString;
int User_Input = 0;

//feedback and control variables
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
int PPR = 1620;
int angle = 360;
int REV = 0; //Required Encoder Value (Setpoint)
int lastMSB = 0;
int lastLSB = 0;
double kp = 5, ki = 1, kd = 0.01;
double input = 0, output = 0, setpoint = 0;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {

  //setup pin-mode
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.begin(9600);

  pinMode(CHA, INPUT_PULLUP);
  pinMode(CHB, INPUT_PULLUP);

  //turn on pull-up resistors
  digitalWrite(CHA, HIGH);
  digitalWrite(CHB, HIGH);

  //update encoder whenever change observed
  //interrupt 0 - pin2; interrupt 1 - pin3
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);

  TCCR1B = TCCR1B & 0b11111000 | 1; //set PWM to 31 KHz to avoid noise
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-125, 125); //Max Limits for angular movement, more than this will change speed
  
}

void loop() {

  while (Serial.available()) { //check if serial data is available
    delay(3);
    char c = Serial.read(); //store input data
    readString += c; //accumalate each character
  }

  if (readString.length() > 0) { //if variable contains info
    Serial.println(readString.toInt()); //print input in integer form
    User_Input = readString.toInt(); //store input data in int form
  }

  REV = map(User_Input, 0, 360, 0, 1620); //mapping degree to pulse
  Serial.print("REV = "); //print REV value
  Serial.println(REV);

  setpoint = REV; //assign setpoint as REV
  input = encoderValue; //process value or feedback value
  Serial.print("FV = ");
  Serial.println(encoderValue); //print FV

  myPID.Compute();
  pwmOut(output);

}

void pwmOut(int out) {
  if (out > 0) { //if REV > encoderValue
    analogWrite(ENA, out); //enabling motor enable pin
    forward();
  }
  else {
    analogWrite(ENA, abs(out));
    reverse();
  }
  readString="";
}

void updateEncoder(){
  int MSB = digitalRead(CHA); //Most Significant Bit
  int LSB = digitalRead(CHB); //Least Significant Bit

  int encoded = (MSB << 1) | LSB; //converting 2 pin value to a single number

  int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;

  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  lastEncoded = encoded; //store value for next time
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void reverse() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
}

void close() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}