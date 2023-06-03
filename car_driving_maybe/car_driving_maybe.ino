#include <Arduino.h>
#include <IRremote.hpp>
#include <Servo.h>

//#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.

Servo servo;
int pos = 40;

class Motor {
  public:
    int aPin;
    int dirPin;
    double output; 
  
    Motor(int analogPin, int directionPin) {
      aPin = analogPin;
      dirPin = directionPin;
    }
  
    void drive(double percentOutput) {
      setOutput(percentOutput);
      drive();
    }

    void drive() {
      
      if (output < 0) {
        digitalWrite(dirPin, LOW);
      } else {
        digitalWrite(dirPin, HIGH);
      }
      
      analogWrite(aPin,(map(abs(output * 1000), 0, 1000, 0, 255)));
    }

    void setOutput(double percentOutput) {
      percentOutput = max(percentOutput, -1.0);
      percentOutput = min(percentOutput, 1.0);
      output = percentOutput;
    }

    void manualDrive(double percentOutput) {
      percentOutput = max(percentOutput, -1.0);
      percentOutput = min(percentOutput, 1.0);
      if (percentOutput < 0) {
        digitalWrite(dirPin, LOW);
      } else {
        digitalWrite(dirPin, HIGH);
      }
      analogWrite(aPin,(map(abs(percentOutput * 1000), 0, 1000, 0, 255))); 
    }
  
};

Motor motors[] = {Motor (6, 2), Motor(5, 4)};

void setup() {
  // put your setup code here, to run once:
  
  IrReceiver.begin(A2, true);
  Serial.begin(115200);
  
  motors[0].setOutput(0);
  motors[1].setOutput(0);
  pinMode(12, OUTPUT);
  pinMode(13, INPUT);
  servo.attach(A0);
}

void printStates() {
  for (int i = 0; i < 2; i++) {
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(" percentOutput ");
    Serial.print(motors[i].output);
    Serial.print(", ");
  }
  Serial.println();
}

int s = 12;
long loopCount = 0;
long lastMillis = 0;
long currMillis = 0;

int upperLimit = 650;
int lowerLimit = 20;
double getTurnPower() {
  double r = analogRead(A4);
  lowerLimit = min(lowerLimit, r);
  upperLimit = max(upperLimit, r);
  r = max(min(r, upperLimit), lowerLimit) - lowerLimit - upperLimit / 2;
  return r * 2 / upperLimit;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (IrReceiver.decode()) {


    IrReceiver.printIRResultShort(&Serial);
    IrReceiver.printIRSendUsage(&Serial);
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
      // We have an unknown protocol here, print more info
      IrReceiver.printIRResultRawFormatted(&Serial, true);
    }
    Serial.println();

    /*
       !!!Important!!! Enable receiving of the next value,
       since receiving has stopped after the end of the current received data packet.
    */
    IrReceiver.resume(); // Enable receiving of the next value
    
    if (IrReceiver.decodedIRData.command == 0x43) {
      motors[0].drive(motors[0].output + 0.1);
    } else if (IrReceiver.decodedIRData.command == 0x9) {
      motors[0].drive(motors[0].output - 0.1);
    } else if (IrReceiver.decodedIRData.command == 0x44) {
      motors[1].drive(motors[1].output + 0.1);
    } else if (IrReceiver.decodedIRData.command == 0x7) {
      motors[1].drive(motors[1].output - 0.1);
    } else if (IrReceiver.decodedIRData.command == 0x45) {
      motors[0].drive(0);
      motors[1].drive(0);
    } else if (IrReceiver.decodedIRData.command == 0x46) {
      motors[1].drive(motors[1].output + 0.1);
      motors[0].drive(motors[0].output + 0.1);
    } else if (IrReceiver.decodedIRData.command == 0x15) {
      motors[1].drive(motors[1].output - 0.1);
      motors[0].drive(motors[0].output - 0.1);
    }
  }
  if (analogRead(A4) > 20) {
    double p = getTurnPower();
    
    motors[1].manualDrive(-0.3 + 2 * pow(p, 3));
    motors[0].manualDrive(-0.3 - 2 * pow(p, 3));
    
    Serial.println(analogRead(A4));
  } else {
    motors[1].drive();
    motors[0].drive();
  }

}

double readSensor() {
  long duration;
  int distance;

  digitalWrite(12, LOW);
  delayMicroseconds(2);

  digitalWrite(12, HIGH);
  delayMicroseconds(10);
  digitalWrite(12, LOW);
  
  duration = pulseIn(13, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}
