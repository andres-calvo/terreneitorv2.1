#include <Arduino.h>
#include <ESP32Servo.h>
#include <HCSR04.h>
#include "MPU6050.h"

int servoPin = 14;
int trigPin = 32;
int echoPin = 33;
int sdaPin = 21;
int sclPin = 22;
int16_t ax, ay, az;

Servo servo;
UltraSonicDistanceSensor distanceSensor(trigPin, echoPin);
MPU6050 mpu;

unsigned long previousMillisServo = 0; // Almacena el último tiempo registrado
unsigned long previousMillisMPU = 0;   // Almacena el último tiempo registrado
const unsigned long intervalServo = 100;
const unsigned long intervalMPU = 100;
unsigned int servoPos = 0;
int incremento = 15;

// put function declarations here:
void updateServoPos();
void readAccel();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50); // standard 50 hz servo
  servo.attach(servoPin, 1000, 2000);
  mpu.initialize();
}

void loop()
{
  // put your main code here, to run repeatedly:
  updateServoPos();
  readAccel();
}

void readAccel()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisMPU >= intervalMPU)
  {
    previousMillisMPU = currentMillis; // Actualiza el tiempo anterior
    mpu.getAcceleration(&ax, &ay, &az);
    Serial.print("a/g:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.println(az);
  }
}

void updateServoPos()
{
  unsigned long currentMillisServo = millis();
  if (currentMillisServo - previousMillisServo >= intervalServo)
  {
    previousMillisServo = currentMillisServo; // Actualiza el tiempo anterior
    // Mover el servo en incrementos de 15 grados
    servoPos += incremento;

    // Si el ángulo supera 180, cambiar la dirección
    if (servoPos >= 180)
    {
      servoPos = 180;
      incremento = -15; // Cambiar la dirección del movimiento
    }
    // Si el ángulo es menor a 0, cambiar la dirección
    if (servoPos <= 0)
    {
      servoPos = 0;
      incremento = 15; // Cambiar la dirección del movimiento
    }
    Serial.print("Servo position: ");
    Serial.print(servoPos);
    Serial.print(" Distance (cm): ");
    Serial.println(distanceSensor.measureDistanceCm());

    servo.write(servoPos);
  }
}