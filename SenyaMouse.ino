#include <Arduino.h>
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// MPU6050 mpu;
// const int16_t accel_offset[3] = { 346, -677, 699 };
// const int16_t gyro_offset[3] = { 15, -119, 4 };
uint8_t fifoBuffer[45];
const int PWMR1{ 11 };
const int PWMR2{ 6 };
const int PWML1{ 3 };
const int PWML2{ 5 };
const int Min{ -60 };
const int Max{ 120 };
const int D = { 4 };
const int F = { 7 };
const int PinGyro = { 12 };
const int ledBlue = { 10 };
const int ledYellow = { 9 };
const int button = { 2 };
const int pinDL = { A2 };
const int pinFL = { A3 };
const int pinDR = { A1 };
const int pinFR = { A0 };
const int THf = { 7 };
const float RVIN = 1.98;
const float RGND = 0.96;
const float BATTERY_DIVIDER_RATIO = RGND / (RGND + RVIN);
const float ADC_MAX = 1023.0;
const float ADC_REF_VOLTS = 5.00;
const float K_VOLTAGE = ADC_REF_VOLTS / ADC_MAX / BATTERY_DIVIDER_RATIO;
enum Walls {
  LEFTK = 0b110,
  RIGHTK = 0b011,
  NOTHING = 0b000,
  LEFT = 0b100,
  RIGHT = 0b001,
  BOTH = 0b101,
  UTURN = 0b111,
  FRONT = 0b010,
};
float spdLeft = 0;
float spdRight = 0;
float delta = 0;
float lastDelta = 0;
int del = 0;

float DLmm = 0;
float FLmm = 0;
float DRmm = 0;
float FRmm = 0;
int DL = 0;
int FL = 0;
int DR = 0;
int FR = 0;
int mode = 0;
unsigned long time = 0;
float znachenie = 0;
float Fx1FL = 0;
float Fx2DL = 0;
float Fx3DR = 0;
float Fx4FR = 0;

//------------надо менять-----------------------------------

const float SpeedBoth = 1.7;
const float K = 0.004;
const float KD = 0.4;
const float K180a = 0.08;
const int tim = 200;

// declaration functions
void readSensorsMM();

//-----------------180----------------------------------------------

void turn180() {
  digitalWrite(ledBlue, 1);
  digitalWrite(ledYellow, 1);
  driveVoltage(-1.5, 1.5);
  do {
    readSensorsMM();
  } while (FLmm < 200 && FRmm < 200);
  driveVoltage(-1.5, 1.5);
  delay(50);
}

//------------------0ые значения-----------------

void firstSenors() {
  readSensorsMM();
  Fx1FL = FL;
  Fx2DL = DL;
  Fx3DR = DR;
  Fx4FR = FR;
}

//------------------плавные значения----------------------

void sensorAlpha() {
  readSensorsMM();
  constexpr float x = 0;
  constexpr float d = 0.09;
  Fx1FL = FLmm * d + (1 - d) * Fx1FL;
  Fx2DL = DLmm * d + (1 - d) * Fx2DL;
  Fx3DR = DRmm * d + (1 - d) * Fx3DR;
  Fx4FR = FRmm * d + (1 - d) * Fx4FR;
  FLmm = Fx1FL;
  DLmm = Fx2DL;
  DRmm = Fx3DR;
  FRmm = Fx4FR;
}

//---------------------драйв по акуму-------------------------

inline void driveVoltage(float leftV, float rightV) {
  rightV = rightV * 1.02;
  //leftV = constrain(leftV, -8.0, +8.0);
  float v = getVoltage();
  int pwmLeft = leftV * 255 / v;
  //rightV = constrain(rightV, -8.0, +8.0);
  int pwmRight = rightV * 255 / v;
  drive(pwmLeft, pwmRight);
}

//---------------вольты----------------------------

inline float getVoltage() {
  return analogRead(A7) * K_VOLTAGE;
}

//-----------------драйв-------------------------

void drive(int left, int right) {
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

//-------------------перевод в 2ую-------------------------

String getBinStr(byte n) {
  String str = "000";
  if (n >= 4) {
    str[0] = '1';
    n = n - 4;
  }
  if (n >= 2) {
    str[1] = '1';
    n = n - 2;
  }
  if (n == 1) {
    str[2] = '1';
  }
  return str;
}

//-------------------значения 1010-------------------------

byte getEventSensors() {
  byte DW = 0;
  readSensorsMM();
  if (DRmm < 225) {
    DW++;
  }
  if (FLmm < 230 && FRmm < 230) {
    DW = DW + 2;
  }
  if (DLmm < 225) {
    DW = DW + 4;
  }
  return DW;
}

//------------------по правой руке-------------------------

void raseRight() {
  long count = 0;
  time = millis();
  while (1) {
    // float v = getVoltage() + 0.2;
    //sensorAlpha();
    readSensorsMM();
    // readSensorsCM();
    uint8_t event = getEventSensors();
    if (event == BOTH) {
      delta = DLmm - DRmm;
      lastDelta = delta;
      znachenie = K * delta + KD * (delta - lastDelta);
      driveVoltage(SpeedBoth - znachenie, SpeedBoth + znachenie);
    } else if (event == UTURN) {
      readSensorsMM();
      while (FRmm > 60 && FLmm > 60) {
        readSensorsMM();
        delta = DLmm - DRmm;
        lastDelta = delta;
        znachenie = K * delta + KD * (delta - lastDelta);
        driveVoltage(SpeedBoth - znachenie, SpeedBoth + znachenie);
      }
      turn180();
      driveVoltage(0, 0);
    } else if (event == RIGHTK) {
      driveVoltage(1.5, 1.5);
      delay(70);
      while (DLmm > 110) {
        driveVoltage(1.3, 2.3);
        readSensorsMM();
      }
      driveVoltage(0, 0);
    } else if (event == LEFTK) {
      driveVoltage(1.5, 1.5);
      delay(70);
      readSensorsMM();
      while (DRmm > 80) {
        readSensorsMM();
        driveVoltage(2.3, 1);
        readSensorsMM();
      }
      driveVoltage(0, 0);
    } else if (event == LEFT) {
      readSensorsMM();
      if (DRmm < 225) {
        return;
      }
      driveVoltage(1.5, 1.5);
      delay(200);
      while (DRmm > 100) {
        readSensorsMM();
        driveVoltage(2.3, 1);
      }
      driveVoltage(0, 0);
    } else if (event == RIGHT) {
      readSensorsMM();
      delta = 110 - DRmm;
      lastDelta = delta;
      driveVoltage(1.5 - K * delta, 1.5 + K * delta);
    } else if (event == NOTHING) {
      readSensorsMM();
      while (DRmm > 140) {
        driveVoltage(3.1, 1);
        readSensorsMM();
      }
      driveVoltage(0, 0);
    } else {
      delta = DLmm - DRmm;
      lastDelta = delta;
      float znachenie = 0.003 * delta + 0.04 * (delta - lastDelta);
      driveVoltage(SpeedBoth - znachenie, SpeedBoth + znachenie);
    }
  }  // while
}

//--------------------принт-----------------------

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

//-----------------------просто тест---------------------

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

//-----------------перевод в мм-------------------------

void readSensorsMM() {
  readSensors();
  FLmm = getMM(FL, 2089.7, -0.705);
  DLmm = getMM(DL, 725.92, -0.571);
  DRmm = getMM(DR, 656.39, -0.602);
  FRmm = getMM(FR, 2708.3, -0.703);
}

//----------------------принт мм------------------------

void printSensorsMM() {
  readSensorsMM();
  float r = getVoltage() + 0.2;
  byte event = getEventSensors();
  String strBin = getBinStr(event);
  Serial.print(FLmm);
  Serial.print("    ");
  Serial.print(DLmm);
  Serial.print("    ");
  Serial.print(DRmm);
  Serial.print("    ");
  Serial.println(FRmm);
  Serial.print("    ");
  Serial.println(DLmm - DRmm);
  Serial.println(strBin);
  Serial.println(r);
  Serial.println(analogRead(A7));
}

//-------------------------сырые значения-----------------------

void readSensors() {
  digitalWrite(D, 1);
  delayMicroseconds(400);
  DL = analogRead(pinDL);
  DR = analogRead(pinDR);
  digitalWrite(D, 0);

  digitalWrite(F, 1);
  delayMicroseconds(400);

  FL = analogRead(pinFL);
  FR = analogRead(pinFR);
  digitalWrite(F, 0);
}

//---------------------формула мм---------------------------

inline float getMM(int x, float k, float p) {
  if (x < 5) {
    return 500;
  }
  float ji = pow(x, p);
  return ji * k;
}

//----------------------сетап-------------------------

void setup() {
  Serial.begin(9600);
  float ypr[3];
  firstSenors();
  // set up freq PWM
  TCCR2B |= (1 << CS21 | 1 << CS20);
  TCCR2B &= ~(1 << CS22);

  // speed up ADC
  ADCSRA |= (1 << ADPS2);
  ADCSRA &= ~(1 << ADPS1);
  ADCSRA |= (1 << ADPS0);

  pinMode(PinGyro, INPUT);
  pinMode(D, OUTPUT);
  pinMode(F, OUTPUT);
  pinMode(ledBlue, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(PWML1, OUTPUT);
  pinMode(PWML2, OUTPUT);
  pinMode(PWMR1, OUTPUT);
  pinMode(PWML2, OUTPUT);
  pinMode(button, INPUT_PULLUP);

  digitalWrite(ledBlue, 1);
  digitalWrite(ledYellow, 1);
  delay(100);
  digitalWrite(ledBlue, 0);
  digitalWrite(ledYellow, 0);

  while (time < 3000) {
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
}

//-------------------------луп------------------------

void loop() {
  if (mode == 1) {
  } else if (mode == 2) {
    raseRight();
  } else {
    driveVoltage(3, 3);
  }
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      while (1) {
        delay(100);
        printSensorsMM();
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
    }
  }
  // delay(100);
}
