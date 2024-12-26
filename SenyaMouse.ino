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
const float K = { 0.63 };  //0.72
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

void turn180() {
  drive(0, 0);
  delay(100);
  drive(-40, 40);
  delay(280);
}

float getVoltage() {
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

void turnRight() {
  drive(50, 50);
  delay(280);
  digitalWrite(ledYellow, 0);
  digitalWrite(ledBlue, 1);
  while (DRcm > 10) {
    readSensorsCM();
    drive(65, 35);
  }
  digitalWrite(ledBlue, 0);
}

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
  if (FRcm < 6) {
    DW++;
  }
  if (DRcm < 20) {
    DW = DW + 2;
  }
  if (DLcm < 20) {
    DW = DW + 4;
  }
  if (FLcm < 6) {
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
  const float KR = 1.8;
  while (1) {
    readSensorsCM();
    byte event = getEventSensors();
    // if (FLcm < THf && DLcm < THf && DRcm < THf && FRcm < THf) {
    //   drive(0, 0);
    //   drive(-40, -40);
    //   delay(690);
    // }
    //else
    if (event == UTURN) {
      turn180();
    }
    // else if (FLcm > 39 && FRcm > 39) {
    //   int delta = DLcm + FLcm - FRcm - DRcm;
    //   drive(speed + 40 - K * delta, speed + 40 + K * delta);
    // }
    // else if (DLcm > 35) {
    //   turnLeft();
    // }
    if (event == BOTH) {
      int delta = DLcm - (DRcm - 4);
      drive(speed - K * delta, speed + K * delta);
    } else if (event == RIGHT) {
      turnRight();
    } else if (event == LEFT) {
      int delta = 14 - DRcm;
      drive(speed - KR * delta, speed + KR * delta);
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
  float r = getVoltage();
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

void setup() {
  Serial.begin(9600);
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

  for (int i = 1; i < 4; i++) {
    digitalWrite(ledBlue, 1);
    digitalWrite(ledYellow, 1);
    delay(200);
    digitalWrite(ledBlue, 0);
    digitalWrite(ledYellow, 0);
    delay(200);
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
  // if (mode == 1) {
  //   raseLeft();
  // } else if (mode == 2) {
  //   raseRight();
  // } else {
  //   rase();
  // }
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
  delay(100);
}
