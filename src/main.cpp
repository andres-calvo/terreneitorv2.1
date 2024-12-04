#include <Arduino.h>
#include <ESP32Servo.h>
#include <HCSR04.h>
#include "MPU6050.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>

#define DHTPIN 18     // Pin de datos para el DHT11
#define DHTTYPE DHT11 // Tipo de sensor, puedes cambiar por DHT22

float humedad;
float temperaturaC;
float temperaturaF;
float sensacionC;
float sensacionF;

int servoPin = 14;
int trigPin = 32;
int echoPin = 33;
int sdaPin = 21;
int sclPin = 22;
int16_t ax, ay, az;

Servo servo;
UltraSonicDistanceSensor distanceSensor(trigPin, echoPin);
MPU6050 mpu;
DHT sensorDHT(DHTPIN, DHTTYPE); // Crear el objeto de tipo DHT

unsigned long previousMillisServo = 0; // Almacena el último tiempo registrado
unsigned long previousMillisMPU = 0;   // Almacena el último tiempo registrado
unsigned long previousMillisDHT = 0;   // Almacena el último tiempo registrado
const unsigned long intervalServo = 100;
const unsigned long intervalMPU = 100;
const unsigned long intervalDHT = 100;
unsigned int servoPos = 0;
int incremento = 15;

// put function declarations here:
void updateServoPos();
void readAccel();
void temperatureSensor();

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();
  sensorDHT.begin();

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
  temperatureSensor();
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

void temperatureSensor()
{

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillisDHT >= intervalDHT)
  {
    previousMillisDHT = currentMillis; // Actualiza el tiempo anterior

    // Lectura de sensores
    float humedad = sensorDHT.readHumidity();
    float temperaturaC = sensorDHT.readTemperature();
    float temperaturaF = sensorDHT.readTemperature(true);

    // Verifica si las lecturas son válidas
    if (isnan(humedad) || isnan(temperaturaC) || isnan(temperaturaF))
    {
      Serial.println("Error en la lectura del sensor");
      return; // Reinicia el loop si hay un error
    }

    // Calcula la sensación térmica
    float sensacionC = sensorDHT.computeHeatIndex(temperaturaC, humedad, false);
    float sensacionF = sensorDHT.computeHeatIndex(temperaturaF, humedad);

    // Imprime los resultados
    Serial.print("Temperatura: ");
    Serial.print(temperaturaC);
    Serial.println(" ºC");
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