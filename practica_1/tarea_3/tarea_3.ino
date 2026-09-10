#include <Servo.h>
// Para la posición 0° el pulso es de 0.6ms, para 90° es de 1.5ms y para 180° 2.4ms
Servo obj_servo;

const int pinServo = 9;
const int pinPote = A1;
int valorPote = 0;
float anguloPote = 0;
unsigned long tiempo1 = 0;
unsigned long tiempo2 = 0;
unsigned long dif = 0;
const int t_preciso = 20000;
int t_delay = 0;

int angulo_a_us(float angulo) {
  return 500 + (angulo * 1000.0 / 150.0);
}

void setup() {
  obj_servo.attach(pinServo);
}

void loop() {
  tiempo1 = micros();
  valorPote = analogRead(pinPote);
  anguloPote = (300.0*valorPote)/1023.0;
  Serial.print(anguloPote);
  Serial.print("\n");
  tiempo2 = micros();
  dif = tiempo2 - tiempo1;
  t_delay = t_preciso - dif;
  delay(10);
  delayMicroseconds(t_delay-10000);

  obj_servo.writeMicroseconds(angulo_a_us(anguloPote));

  
}

