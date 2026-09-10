#include <NewPing.h>

const int pinTrig = 2;
const int pinEcho = 4;

const float vs = 29.287;
float tiempo = 0;
float tiempo1 = 0;
float tiempo2 = 0;
float dif = 0;
NewPing sensor (pinTrig, pinEcho);
float valorDist = 0;
float frec = 0;
float frecLectura = 0;
const int t_preciso = 20000;
int t_delay = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  tiempo1 = micros();

  tiempo =sensor.ping();
  valorDist = tiempo/vs/2;

  Serial.print(valorDist);
  Serial.print("|");

  tiempo2 = micros();
  dif = (tiempo2 - tiempo1);
  frec = (1000000.0/dif);
  Serial.print(frec);
  Serial.print("\n");

  tiempo2 = micros();
  dif = (tiempo2 - tiempo1);
  t_delay = t_preciso - dif;
  float tiempo3 = micros();
  delay(10);
  delayMicroseconds(t_delay-10000);

  tiempo2 = micros();
  dif = (tiempo2 - tiempo1);
  frecLectura = (1000000.0/dif);
  Serial.print(frecLectura);
  Serial.print("\n");

}

