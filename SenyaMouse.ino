#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
const int16_t accel_offset[3] = { 346, -677, 699 };
const int16_t gyro_offset[3] = { 15, -119, 4 };
uint8_t fifoBuffer[45];
const int PWMR1{ 11 };
const int PWMR2{ 6 };
const int PWML1{ 3 };
const int PWML2{ 5 };
const int Min{ -50 };
const int Max{ 200 };
const int D = { 4 };
const int F = { 7 };
const int ledBlue = { 10 };
const int ledYellow = { 9 };
const int button = { 2 };
const int pinDL = { A2 };
const int pinFL = { A3 };
const int pinDR = { A1 };
const int pinFR = { A0 };
const int THf = { 7 };
const int speed = { 50 };  //60
const float RVIN = 1.98;
const float RGND = 0.96;
const float BATTERY_DIVIDER_RATIO = RGND / (RGND + RVIN);
const float ADC_MAX = 1023.0;
const float ADC_REF_VOLTS = 5.00;
const float K_VOLTAGE = ADC_REF_VOLTS / ADC_MAX / BATTERY_DIVIDER_RATIO;
enum Walls {
  LEFT = 0b0010,
  RIGHT = 0b0100,
  BOTH = 0b0110,
  UTURN = 0b1111
};
int ho = 0;
int gt = 0;
int del = 0;
const float K = 0.045;  //0.67
float DLcm = 0;
float FLcm = 0;
float DRcm = 0;
float FRcm = 0;
int DL = 0;
int FL = 0;
int DR = 0;
int FR = 0;
int mode = 0;
unsigned long time = 0;
float Fx1FL = 0;
float Fx2DL = 0;
float Fx3DR = 0;
float Fx4FR = 0;

void turn180() {
  digitalWrite(ledBlue, 1);
  digitalWrite(ledYellow, 1);
  readSensorsCM();
  driveVoltage(0, 0);
  delay(50);
  driveVoltage(-1.4, 1.4);
  while (FLcm < 15 && FRcm < 15) {
    readSensorsCM();
  }
  driveVoltage(-1.4, 1.4);
  delay(100);
  digitalWrite(ledBlue, 1);
  digitalWrite(ledYellow, 1);
  driveVoltage(0, 0);
  delay(50);
  // ho = getDegrees();
  // gt = (ho + 180) % 360;
  // if (ho > 180) {
  //   while (getDegrees() > gt) {
  //     driveVoltage(-1, 1);
  //   }
  // } else if (ho < 180) {
  //   while (getDegrees() < gt) {
  //     driveVoltage(1, -1);
  //   }
  // }
}

void firstSenors() {
  readSensorsCM();
  Fx1FL = FL;
  Fx2DL = DL;
  Fx3DR = DR;
  Fx4FR = FR;
}

void sensorAlpha() {
  readSensorsCM();
  float x = 0;
  float d = 0.03;
  x = FL;
  Fx1FL = x * d + (1 - d) * Fx1FL;
  x = DL;
  Fx2DL = x * d + (1 - d) * Fx2DL;
  x = DR;
  Fx3DR = x * d + (1 - d) * Fx3DR;
  x = FR;
  Fx4FR = x * d + (1 - d) * Fx4FR;
  FL = Fx1FL;
  DL = Fx2DL;
  DR = Fx3DR;
  FR = Fx4FR;
}

float getDegrees() {
  static float degr = 0;
  static Quaternion q;
  static VectorFloat gravity;
  static float ypr[3]{};
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    degr = degrees(ypr[0]);
    if (degr < 0) {
      degr = 360 + degr;
    }
  }
  return degr;
}

bool initGyro() {
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed"));
    return false;
  } else {
    Serial.println(F("MPU6050 connection - OK"));
  }


  mpu.dmpInitialize();

  mpu.setXGyroOffset(gyro_offset[0]);
  mpu.setYGyroOffset(gyro_offset[1]);
  mpu.setZGyroOffset(gyro_offset[2]);
  mpu.setXAccelOffset(accel_offset[0]);
  mpu.setYAccelOffset(accel_offset[1]);
  mpu.setZAccelOffset(accel_offset[2]);

  mpu.setDMPEnabled(true);
  return true;
}

inline void driveVoltage(float leftV, float rightV) {
  rightV = rightV * 1.05;
  //leftV = constrain(leftV, -8.0, +8.0);
  float v = getVoltage();
  int pwmLeft = leftV * 255 / v;
  //rightV = constrain(rightV, -8.0, +8.0);
  int pwmRight = rightV * 255 / v;
  drive(pwmLeft, pwmRight);
}

inline float getVoltage() {
  return analogRead(A7) * K_VOLTAGE;
}

void turnLeft() {
  drive(50, 50);
  delay(250);
  digitalWrite(ledYellow, 1);
  digitalWrite(ledBlue, 0);
  while (DLcm > 10) {
    drive(30, 70);
    readSensorsCM();
  }
  digitalWrite(ledYellow, 0);
}

// void turn90() {
//   float now = getDegrees();

//   if (target - rew > 180) {
//     rew = rew + 360;
//   }

//     float rew = getDegrees();

//     while
//   }

void turnRight() {
  pryamo(650);
  digitalWrite(ledYellow, 0);
  digitalWrite(ledBlue, 1);
  ho = getDegrees();
  gt = (ho + 90) % 360;
  driveVoltage(1, -1);
  while (abs(gt - getDegrees()) > 13) {
  }
  driveVoltage(0, 0);
  delay(100);
  pryamo(30);
  digitalWrite(ledBlue, 0);
}

void drive(int left, int right) {
  if (left > 160 || right > 160 || left < -160 || right < -160) {
    readSensorsCM();
    float r = getVoltage() + 0.2;
    byte event = getEventSensors();
    String strBin = getBinStr(event);
    Serial.print(FLcm);
    Serial.print("    ");
    Serial.print(DLcm);
    Serial.print("    ");
    Serial.print(DRcm);
    Serial.print("    ");
    Serial.println(FRcm);
    Serial.print("    ");
    Serial.println(DLcm + FLcm - DRcm - FRcm);
    Serial.println(strBin);
    Serial.println(r);
  }
  left = constrain(left, Min, Max);
  right = constrain(right, Min, Max);
  if (left > 0) {
    digitalWrite(PWML1, 1);
    analogWrite(PWML2, 255 - left);
  } else {
    analogWrite(PWML1, 255 + left);
    digitalWrite(PWML2, 1);
  }
  if (right > 0) {
    digitalWrite(PWMR1, 1);
    analogWrite(PWMR2, 255 - right);
  } else {
    analogWrite(PWMR1, 255 + right);
    digitalWrite(PWMR2, 1);
  }
}

String getBinStr(byte n) {
  String str = "0000";
  if (n >= 8) {
    str[0] = '1';
    n = n - 8;
  }
  if (n >= 4) {
    str[1] = '1';
    n = n - 4;
  }
  if (n >= 2) {
    str[2] = '1';
    n = n - 2;
  }
  if (n == 1) {
    str[3] = '1';
  }
  return str;
}

byte getEventSensors() {
  byte DW = 0;
  readSensorsCM();
  if (FRcm < 5) {
    DW++;
  }
  if (DRcm < 23) {
    DW = DW + 2;
  }
  if (DLcm < 18) {
    DW = DW + 4;
  }
  if (FLcm < 5) {
    DW = DW + 8;
  }
  return DW;
}

void raseLeft() {
  while (1) {
    byte event = getEventSensors();
    // readSensorsCM();
    // if (FLcm < THf && DLcm < THf && DRcm < THf && FRcm < THf) {
    //   drive(0, 0);
    //   drive(-40, -40);
    //   delay(690);
    // }
    //else
    if (FLcm < 8 && FRcm < 8 && DLcm < 8 && DRcm < 8) {
      turn180();
    }
    // else if (FLcm > 39 && FRcm > 39) {
    //   int delta = DLcm + FLcm - FRcm - DRcm;
    //   drive(speed + 40 - K * delta, speed + 40 + K * delta);
    // }
    else if (DLcm > 35) {
      turnLeft();
    }
    if (event == BOTH) {
      int delta = DLcm - DRcm;
      drive(speed - K * delta, speed + K * delta);
    }
    //  else if (DRcm > 35) {
    //   turnRight();
    // }
    /*else {
      int delta = DLcm - DRcm;
      drive(speed - K * delta, speed + K * delta);
    }
  */
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's') {
        drive(0, 0);
        return;
      }
    }
  }  // while
}

void raseRight() {
  long count = 0;
  time = millis();
  while (1) {
    // float v = getVoltage() + 0.2;
    sensorAlpha();
    // readSensorsCM();
    byte event = getEventSensors();
    // if (FLcm < THf && DLcm < THf && DRcm < THf && FRcm < THf) {
    //   drive(0, 0);
    //   drive(-40, -40);
    //   delay(690);
    // }
    //else
    //count++;
    // if (event != BOTH) {
    //   drive(0, 0);
    //   Serial.print(FLcm);
    //   Serial.print("    ");
    //   Serial.print(DLcm);
    //   Serial.print("    ");
    //   Serial.print(DRcm);
    //   Serial.print("    ");
    //   Serial.println(FRcm);
    //   Serial.print("    ");
    //   Serial.println(DLcm + FLcm - DRcm - FRcm);
    //   while (1) {
    //   }
    // }


    // else if (FLcm > 39 && FRcm > 39) {
    //   int delta = DLcm + FLcm - FRcm - DRcm;
    //   drive(speed + 40 - K * delta, speed + 40 + K * delta);
    // }
    // else if (DLcm > 35) {
    //   turnLeft();
    // }
    if (event == BOTH) {
      int delta = DLcm - DRcm;
      driveVoltage(1.5 - K * delta, 1.5 + K * delta);
    } else if (event == UTURN) {
      turn180();
    } else if (event == RIGHT) {
      turnRight();
    } else if (event == LEFT) {
      int delta = 10 - DRcm;
      driveVoltage(1 - K * delta, 1 + K * delta);
    }
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's') {
        long u = millis() - time;
        drive(0, 0);
        Serial.print(count);
        Serial.print("    ");
        Serial.print(u);
        Serial.print("    ");
        Serial.print((1000L * count) / u);
        return;
      }
    }
  }  // while
}

void rase() {
  while (1) {
    readSensorsCM();
    // if (FLcm < THf && DLcm < THf && DRcm < THf && FRcm < THf) {
    //   drive(0, 0);
    //   drive(-40, -40);
    //   delay(690);
    // }
    //else
    if (FLcm < 8 && FRcm < 8 && DLcm < 8 && DRcm < 8) {
      turn180();
    }
    // else if (FLcm > 39 && FRcm > 39) {
    //   int delta = DLcm + FLcm - FRcm - DRcm;
    //   drive(speed + 40 - K * delta, speed + 40 + K * delta);
    // }
    else if (DLcm > 35) {
      turnLeft();
    } else if (DRcm > 35) {
      turnRight();
    } else {
      int delta = DLcm - DRcm;
      drive(speed - K * delta, speed + K * delta);
    }

    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's') {
        drive(0, 0);
        return;
      }
    }


  }  // while
}

void printSensors() {
  readSensors();
  Serial.print(FL);
  Serial.print("    ");
  Serial.print(DL);
  Serial.print("    ");
  Serial.print(DR);
  Serial.print("    ");
  Serial.println(FR);
  Serial.print("    ");
  Serial.println(DL + FL - DR - FR);
}

void testDrive() {
  drive(-60, -60);
  delay(1000);
  while (1) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 's') {
        drive(0, 0);

        return;
      }
    }
  }
}

void readSensorsCM() {
  readSensors();
  FLcm = getCM(FL, 3112.3, 1.601);
  DLcm = getCM(DL, 2778.2, 1.963);
  DRcm = getCM(DR, 1859.9, 1.926);
  FRcm = getCM(FR, 4113.1, 1.548);
}

void printSensorsCM() {
  readSensorsCM();
  float r = getVoltage() + 0.2;
  byte event = getEventSensors();
  String strBin = getBinStr(event);
  Serial.print(FLcm);
  Serial.print("    ");
  Serial.print(DLcm);
  Serial.print("    ");
  Serial.print(DRcm);
  Serial.print("    ");
  Serial.println(FRcm);
  Serial.print("    ");
  Serial.println(DLcm + FLcm - DRcm - FRcm);
  Serial.println(strBin);
  Serial.println(r);
}

void readSensors() {
  digitalWrite(D, 1);
  delay(1);
  DL = analogRead(pinDL);
  DR = analogRead(pinDR);
  digitalWrite(D, 0);

  digitalWrite(F, 1);
  delay(1);

  FL = analogRead(pinFL);
  FR = analogRead(pinFR);
  // FL = FL / 67.0 * 100;
  // DL = DL / 22.0 * 100;
  // DR = DR / 17.0 * 100;
  digitalWrite(F, 0);
}

float getCM(int x, float K, float p) {
  if (x == 0) {
    return 50;
  } else {
    return exp(log(K / float(x)) / p);
  }
}

void pryamo(int t) {
  readSensorsCM();
  long mil = millis();
  float target = getDegrees();
  while ((millis() - mil) < t && (FLcm > 6 && FRcm > 6)) {
    readSensorsCM();
    float rew = getDegrees();
    if (target - rew > 180) {
      rew = rew + 360;
    }
    float error = target - rew;
    driveVoltage(1.2 + error * 0.03, 1.2 - error * 0.03);
    // Serial.print(target);
    // Serial.print(' ');
    // Serial.print(' ');
    // Serial.print(' ');
    // Serial.print(rew);
    // Serial.print(' ');
    // Serial.print(' ');
    // Serial.print(' ');
    // Serial.println(error);
    // delay(800);
  }
  //driveVoltage(1.2 - error * 0.03, 1.2 + error * 0.03);
}

void setup() {
  Serial.begin(9600);
  float ypr[3];
  firstSenors();
  // Set Timer2 PWM mode phase correct(pin 11, pin 3) :
  // 16 000 000 / (32 * 510) = 980.39Hz
  // CS22 CS21 CS20  011 - prescale 32
  TCCR2B |= (1 << CS21 | 1 << CS20);
  TCCR2B &= ~(1 << CS22);

  pinMode(D, OUTPUT);
  pinMode(F, OUTPUT);
  pinMode(ledBlue, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(PWML1, OUTPUT);
  pinMode(PWML2, OUTPUT);
  pinMode(PWMR1, OUTPUT);
  pinMode(PWML2, OUTPUT);
  pinMode(button, INPUT_PULLUP);

  if (initGyro() == true) {
    for (int i = 1; i < 4; i++) {
      digitalWrite(ledBlue, 1);
      digitalWrite(ledYellow, 1);
      delay(200);
      digitalWrite(ledBlue, 0);
      digitalWrite(ledYellow, 0);
      delay(200);
      Serial.println(getDegrees());
    }
  } else if (initGyro() == false) {
    for (int i = 1; i < 3; i++) {
      digitalWrite(ledBlue, 1);
      digitalWrite(ledYellow, 1);
      delay(200);
      digitalWrite(ledBlue, 0);
      digitalWrite(ledYellow, 0);
      delay(200);
    }
  }
  while (time < 6000) {
    time = millis();
    if (digitalRead(button) == 0) {
      delay(350);
      mode++;
      if (mode > 3) {
        mode = 3;
      }
      if (mode == 1) {
        digitalWrite(ledBlue, 0);
        digitalWrite(ledYellow, 1);
      } else if (mode == 2) {
        digitalWrite(ledBlue, 1);
        digitalWrite(ledYellow, 0);
      } else if (mode == 3) {
        digitalWrite(ledBlue, 1);
        digitalWrite(ledYellow, 1);
      }
    }
  }
  // while (digitalRead(button) == 1) {
  // }
}

void loop() {
  if (mode == 1) {
  } else if (mode == 2) {
    raseRight();
  } else {
    driveVoltage(3,3);  
  }
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      while (1) {
        delay(100);
        printSensorsCM();
        if (Serial.available()) {
          char c = Serial.read();
          if (c == 's') {
            return;
          }
        }
      }
    } else if (c == 't') {
      testDrive();
    } else if (digitalRead(button) == 0) {
      rase();
    }
  }
  // delay(100);
}