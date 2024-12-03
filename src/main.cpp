#include <Arduino.h>
#include <ESP32Servo.h>

int servoPin = 14;

Servo servo;

unsigned long previousMillisServo = 0; // Almacena el último tiempo registrado
const unsigned long intervalServo = 100;
unsigned int servoPos = 0;
int incremento = 15;

// put function declarations here:
void updateServoPos();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50); // standard 50 hz servo
  servo.attach(servoPin, 1000, 2000);
}

void loop()
{
  // put your main code here, to run repeatedly:
  updateServoPos();
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
    Serial.println(servoPos);
    servo.write(servoPos);
  }
}