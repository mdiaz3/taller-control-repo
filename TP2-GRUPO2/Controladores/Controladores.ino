#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>

Servo obj_servo;

const int pinTrig = 6;
const int pinEcho = 7;
const float vs = 29.287;
NewPing sensor(pinTrig, pinEcho);

const int pinServo = 9;
unsigned long tiempo1;
unsigned long tiempo2;
const int T = 20000;
int t_delay = 0;
float dif = 0;
float wx_prom = 0;

float Kp1=-0.9;//controlador P
float Kp2 = -1.2;//controlador PI
float Ki2 = -0.09;//controlador PI

float Kp3 = -1.2;//controlador PD
float Kd3 = -0.007;//controlador PD

void setup(void) {
  Serial.begin(115200);
  obj_servo.attach(pinServo);
}

void loop() {
  tiempo1 = micros();
  float referencia = 15;//cambiar esto a 15 cuando no simulamos el escalon
  float dist_carro = leer_distancia();
  float pos = dist_carro-15;
  float error = referencia - dist_carro;

  //float u = control_P(error, Kp1); // u en grados
  //float u = control_PI(error, Kp2, Ki2);
  float u = control_PD(error, Kp3, Kd3);

  mover_servo(angulo_a_us(u), obj_servo);

  float datos[2] = {u,pos/100};
  matlab_send(datos, 2);

  tiempo2 = micros();
  dif = tiempo2 - tiempo1;
  t_delay = T - dif;
  if(t_delay >= 16383){
    delay(10);
    delayMicroseconds(t_delay-10000);
  } else if(t_delay > 0) {
    delayMicroseconds(t_delay);
  }
}

float leer_distancia() {
  unsigned int tiempo_echo = sensor.ping();
  float distancia = tiempo_echo / (2.0 * vs);
  return distancia;
}

float control_P(float error, float Kp){
    float u = Kp * error;
    return u;
}

float control_PI(float error, float Kp, float Ki) {
    static float error_anterior = 0.0;
    static float u_anterior = 0.0;
    const float Ts = 0.02;

    float u = u_anterior + (Kp + Ki * Ts / 2.0) * error
            + (-Kp + Ki * Ts / 2.0) * error_anterior;

    error_anterior = error;
    u_anterior = u;

    return u;
}
float control_PD(float error, float Kp, float Kd) {
    static float error_anterior = 0.0;
    const float Ts = 0.02;

    float u = Kp * error + (2.0 * Kd / Ts) * (error - error_anterior);
    error_anterior = error;
    return u;
}
int angulo_a_us(float angulo) {
  return 1350 + (-angulo * 350.0 / 11); // cero grados son 1350us, 350us mueve 10.5 grados (medido experimentalmente)
}

void mover_servo(float u, Servo &obj_servo){
    if (u < 900) u = 900;
    else if (u > 1800) u = 1800;
    obj_servo.writeMicroseconds(u);
}

void matlab_send(float datos[], int n) {
  Serial.write("abcd");
  for (int i = 0; i < n; i++) {
    byte *b = (byte *)&datos[i];
    Serial.write(b, 4);
  };
}

