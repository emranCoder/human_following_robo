#include <SoftwareSerial.h>
#include <NewPing.h>
#include <AFMotor.h>
SoftwareSerial BT_Serial(2, 3);  // RX, TX
#define servo 10
#define R_S A0      //ir sensor Right
#define L_S A1      //ir sensor Left
#define echo A2     //Echo pin
#define trigger A3  //Trigger pin
int distance_L, distance_F = 40, distance_R;
long distance;
int set = 30;
int bt_ir_data;  // variable to receive data from the serial port 
int Speed = 120;
int mode = 0;
#define MAX_DISTANCE 100
NewPing sonar(trigger, echo, MAX_DISTANCE);

AF_DCMotor Motor1(1, MOTOR12_1KHZ);
AF_DCMotor Motor4(4, MOTOR34_1KHZ);

void setup() {               // put your setup code here, to run once
  pinMode(R_S, INPUT);       // declare if sensor as input
  pinMode(L_S, INPUT);       // declare ir sensor as input
  pinMode(echo, INPUT);      // declare ultrasonic sensor Echo pin as input
  pinMode(trigger, OUTPUT);  // declare ultrasonic sensor Trigger pin as Output

  Serial.begin(9600);  // start serial communication at 9600bps
  BT_Serial.begin(9600);
  for (int angle = 70; angle <= 140; angle += 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 140; angle >= 0; angle -= 5) {
    servoPulse(servo, angle);
  }
  for (int angle = 0; angle <= 70; angle += 5) {
    servoPulse(servo, angle);
  }
  delay(500);
}
void loop() {
  if (BT_Serial.available() > 0) {  //if some date is sent, reads it and saves in state
    bt_ir_data = BT_Serial.read();
    Serial.println(bt_ir_data);
    if (bt_ir_data > 20) { Speed = bt_ir_data; }
  }

  else if (bt_ir_data == 9) {
    mode = 1;
    Speed = 130;
  }  //Auto Line Follower Command
  else if (bt_ir_data == 10) {
    mode = 2;
    Speed = 255;
  }  //Auto Obstacle Avoiding Command
  else if (bt_ir_data == 8) {
    mode = 0;
    Speed = 120;
  }  //Manual  Command
  if (mode == 0) {
    //===============================================================================
    //                          Key Control Command
    //===============================================================================
    if (bt_ir_data == 1) { forword(); }  // if the bt_data is '1' the DC motor will go forward
    else if (bt_ir_data == 2) {
      backword();
    }                                          // if the bt_data is '2' the motor will Reverse
    else if (bt_ir_data == 3) { turnLeft(); }  // if the bt_data is '3' the motor will turn left
    else if (bt_ir_data == 4) {
      turnRight();
    }                                      // if the bt_data is '4' the motor will turn right
    else if (bt_ir_data == 5) { Stop(); }  // if the bt_data '5' the motor will Stop
    //===============================================================================
    //                          Voice Control Command
    //===============================================================================
    else if (bt_ir_data == 6) {
      turnLeft();
      delay(400);
      speed = 100
      bt_ir_data = 5;
    } else if (bt_ir_data == 7) {
      turnRight();
      delay(400);
      speed = 100
      bt_ir_data = 5;
    }
  }
  if (mode == 1) {
    //===============================================================================
    //                          Human Following Control
    //===============================================================================

    delay(50);
    unsigned int distance = sonar.ping_cm();

    int Right_Value = digitalRead(R_S);
    int Left_Value = digitalRead(L_S);


    if ((Right_Value == 1) && (distance >= 10 && distance <= 30) && (Left_Value == 1)) {
      forword();
    } else if ((Right_Value == 0) && (Left_Value == 1)) {
      turnLeft();
    } else if ((Right_Value == 1) && (Left_Value == 0)) {
      turnRight();
    } else if ((Right_Value == 1) && (Left_Value == 1)) {
      Stop();
    } else if (distance > 1 && distance < 10) {
      Stop();
    }
  }
  if (mode == 2) {

    //===============================================================================
    //                          Obstacle Avoiding Control
    //===============================================================================
    if (bt_ir_data == 8) {
      mode = 0;
      Speed = 120;
      distance_F = 30;
    } else {
      distance_F = Ultrasonic_read();
      Serial.print("S=");
      Serial.println(distance_F);
      if (bt_ir_data == 8) {
        mode = 0;
        Speed = 120;
        distance_F = 30;
      }
      if (distance_F > set) {
        forword();
      } else {
        Check_side();
      }
    }
  }
  delay(10);
}

void servoPulse(int pin, int angle) {
  int pwm = (angle * 11) + 500;  // Convert angle to microseconds
  digitalWrite(pin, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(pin, LOW);
  delay(50);  // Refresh cycle of servo
}
//**********************Ultrasonic_read****************************
long Ultrasonic_read() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  distance = pulseIn(echo, HIGH);
  return distance / 29 / 2;
}
void compareDistance() {
  if (distance_L > distance_R) {
    turnLeft();
    delay(350);
  } else if (distance_R > distance_L) {
    turnRight();
    delay(350);
  } else {
    backword();
    delay(300);
    turnRight();
    delay(600);
  }
}
void Check_side() {
  Stop();
  delay(100);
  for (int angle = 70; angle <= 140; angle += 5) {
    servoPulse(servo, angle);
  }
  delay(300);
  distance_L = Ultrasonic_read();
  delay(100);
  for (int angle = 140; angle >= 0; angle -= 5) {
    servoPulse(servo, angle);
  }
  delay(500);
  distance_R = Ultrasonic_read();
  delay(100);
  for (int angle = 0; angle <= 70; angle += 5) {
    servoPulse(servo, angle);
  }
  delay(300);
  compareDistance();
}
void forword() {  //forword
  Motor1.setSpeed(Speed);
  Motor1.run(BACKWARD);
  Motor4.setSpeed(Speed);
  Motor4.run(FORWARD);
}
void backword() {  //backword
  Motor1.setSpeed(Speed);
  Motor1.run(FORWARD);
  Motor4.setSpeed(Speed);
  Motor4.run(BACKWARD);
}
void turnRight() {  //turnRight
  Motor1.setSpeed(100);
  Motor1.run(FORWARD);
  Motor4.setSpeed(200);
  Motor4.run(FORWARD);
}
void turnLeft() {  //turnLeft
  Motor1.setSpeed(200);
  Motor1.run(BACKWARD);
  Motor4.setSpeed(100);
  Motor4.run(BACKWARD);
}
void Stop() {  //stop
  Motor1.setSpeed(0);
  Motor1.run(RELEASE);
  Motor4.setSpeed(0);
  Motor4.run(RELEASE);
}
