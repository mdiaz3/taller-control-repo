#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Servo obj_servo;

const int pinServo = 9;
unsigned long tiempo1;
unsigned long tiempo2;
const int T = 20000; // periodo en us
int t_delay = 0;
float dif = 0;

void setup() {
  // inicializar servo
  obj_servo.attach(pinServo);

}

void loop() {
  

  obj_servo.writeMicroseconds(1350);
  delay(3000);

  tiempo1 = micros();
  obj_servo.writeMicroseconds(368.69); // 30 grados
  tiempo2 = micros();

  dif = tiempo2-tiempo1; // tiempo en us
  Serial.print(dif)

  delay(3000);


}



int angulo_a_us(float angulo) {
  return 1350 + (-angulo * 350.0 / 10.7); // cero grados son 1350us, 350us mueve 10.5 grados (medido experimentalmente)
}