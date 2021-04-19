//
//  Maxon EC Motor Position Controller with Hall-Effect Sensor
//  
//  Created by ibaydin on 19.04.2021.
//  Copyright © 2021 i.Batuhan Aydın. All rights reserved.
//  www.ibaydin.com
//


#define HallSensorU_pin  19 //PD2
#define HallSensorV_pin  20 //PD1
#define HallSensorW_pin  21 //PD0

#define CW  1
#define CCW -1

#define en 12    //PE3
#define sped 7  //PH4

int motor_steps = 72;
int reduction = 156;

double Kp = 0.32;
double Kd = 6.5;

double step_to_deg = (motor_steps*reduction)/360;

int direct = 1;
int pulseCount = 0;

double desired_angle = 0.0;

double angle = 0.0;

int desired_step = 0.0;

double error = 0;
double prev_error = 0;

bool HSU_Val = digitalRead(HallSensorU_pin);
bool HSV_Val = digitalRead(HallSensorV_pin);
bool HSW_Val = digitalRead(HallSensorW_pin);

char buffer[20];

String data;

int debug_mode  = 0;
int status1 = 0;

void setup() {
  pinMode(HallSensorU_pin, INPUT);
  pinMode(HallSensorV_pin, INPUT);
  pinMode(HallSensorW_pin, INPUT);


  pinMode(en, OUTPUT);
  pinMode(sped, OUTPUT);

  digitalWrite(en, LOW);
  
  attachInterrupt(digitalPinToInterrupt(HallSensorU_pin), HallSensorU, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HallSensorV_pin), HallSensorV, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HallSensorW_pin), HallSensorW, CHANGE);

  Serial.begin(115200);

}

void loop() {
  angle = pulseCount / step_to_deg;
  control();

  if (debug_mode == 1)
  {
    Serial.print("Step : ");
    Serial.print(pulseCount);
    Serial.print("   |   desired step : ");
    Serial.print(desired_step);
    Serial.print("   |   Angle : ");
    Serial.print(angle, 5);
    Serial.print("   |   Kp : ");
    Serial.print(Kp, 5);
    Serial.print("   |   Kd : ");
    Serial.print(Kd, 5);
    Serial.print("   |   Status : ");
    Serial.println(status1);
  }

  else
    Serial.println(angle);
}


void control() 
{
  desired_step = (int)(desired_angle * step_to_deg);
  
  error = desired_step - pulseCount;

  double pid_out = error * Kp+ Kd * (error-prev_error);
  prev_error = error;
  
  if (pid_out > 0){
    if (pid_out > 100)
      pid_out = 100;
    else if (pid_out < 10)
      pid_out = 10;
  }

  else {
    if (pid_out < -100)
      pid_out = -100;
    else if (pid_out > -10)
      pid_out = -10;
  }
  
  motor_start(pid_out);
}

void motor_start(double spd){
  if (spd != 0 && status1 == 1) {
    digitalWrite(en, HIGH);

    double out = map(spd, -100, 100, 255, 0);
    analogWrite(sped, out);
  }
  else{
    digitalWrite(en, LOW);
  }

}

void HallSensorW() {
  HSW_Val = digitalRead(HallSensorW_pin);         
  HSV_Val = digitalRead(HallSensorV_pin);           
  direct = (HSW_Val == HSV_Val) ? CW : CCW;   
  pulseCount = pulseCount + (1 * direct); 

}

void HallSensorV() {
  HSV_Val = digitalRead(HallSensorV_pin);
  HSU_Val = digitalRead(HallSensorU_pin);
  direct = (HSV_Val == HSU_Val) ? CW : CCW;
  pulseCount = pulseCount + (1 * direct);
}

void HallSensorU() {
  HSU_Val = digitalRead(HallSensorU_pin);
  HSW_Val = digitalRead(HallSensorW_pin);
  direct = (HSU_Val == HSW_Val) ? CW : CCW;
  pulseCount = pulseCount + (1 * direct);
}

void serialEvent() {
  if (Serial.available() > 0){
    data = Serial.readString();
    data.toCharArray(buffer, data.length());
    Serial.print("Data : ");
    Serial.println(buffer);
    desired_angle = parseNumber('a', 'a', desired_angle);
    Kp = parseNumber('P', 'p', Kp);
    Kd = parseNumber('d', 'D', Kd);
    debug_mode = parseNumber('m', 'M', debug_mode);
    status1 = parseNumber('s', 'S', status1);
  }
}

float parseNumber(char code, char code2, double val){
  char *ptr = buffer; // start at the beginning of buffer
  for (int i = 0; i < sizeof(buffer); i++)
  {
    if (*ptr == code || *ptr == code2) // if you find code on your walk,
      return atof(ptr + 1); // convert the digits that follow into a float and return it
    ptr++;
  }
  return val;
}
