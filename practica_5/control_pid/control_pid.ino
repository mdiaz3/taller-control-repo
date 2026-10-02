// calibracion del giroscopio, lectura de la imu, 
// estimacion de angulos, filtro complementario,
// envio de datos a simulink, movimiento de la barra con el servo

#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>

Servo obj_servo;

const int pinTrig = 6;
const int pinEcho = 7;
const float vs = 29.287; // inversa de la velocidad del sonido us/cm
NewPing sensor(pinTrig, pinEcho);

const int pinServo = 9;
unsigned long tiempo1;
unsigned long tiempo2;
const int T = 20000; // periodo en us
int t_delay = 0;
float dif = 0;
float wx_prom = 0; // velocidad angular promedio en x


void setup(void) {
  Serial.begin(115200);

  // inicializar servo
  obj_servo.attach(pinServo);

}

void loop() {
  
  tiempo1 = micros();
  
  // referencia de angulo de la barra
  float referencia = 15;
  float K0 = 32.6;
  static float T0 = 1.5; // periodo de oscilacion en s


  // medir distancia.
  float dist_carro = leer_distancia();
  float error = referencia - dist_carro;

  float u = control_PID(error, 0.45*K0, 0.3*K0/T0, 0.000001*K0/T0, T*1E-6);
  //float u = control_PI(error, 0.5*K0, 0.4*K0/T0, T*1E-6);
  //float u = control_P(error, 0.5*K0);

  mover_servo(1350.0 + u, obj_servo);

  float datos[3] = {u/100, dist_carro, error};
  matlab_send(datos, 3);

  // controlar la frecuencia de muestreo
  tiempo2 = micros();
  dif = tiempo2 - tiempo1;
  t_delay = T - dif;
  if(t_delay >= 16383){
    delay(10);
    delayMicroseconds(t_delay-10000);
  }
  else{
    delayMicroseconds(t_delay);
  }

}


float leer_distancia() {
  //Función para leer a qué distancia en cm se encuentra un objeto del sensor
  unsigned int tiempo_echo = sensor.ping();
  float distancia = tiempo_echo / (2.0 * vs);
  return distancia;
}

float estimar_angulo_rad(sensors_event_t a, sensors_event_t g){
  // filtro complementario

  const float R = 0.93;
  static float alpha_g_inicial = 0;
  static float alpha_g_comp = alpha_g_inicial + (g.gyro.x - wx_prom) * T*1E-6; // estimacion del angulo basada en el giroscopio para el filtro complementario
  
  float alpha_a_comp = atan2(a.acceleration.y, a.acceleration.z); // estimacion del angulo basada en el acelerometro para el filtro complementario
  float alpha_f = alpha_g_comp*R + alpha_a_comp*(1-R); // estimacion del filtro complementario
  alpha_g_comp = alpha_f + (g.gyro.x - wx_prom) * T*1E-6;

  return alpha_f;
}

float control_P(float error, float Kp){

    float u = Kp * error;

    return u;
}

float control_PI(float error, float Kp, float Ki, float Ts){
  static float error_anterior = 0, I = 0;

  float I_k = I + (Ts / 2.0) * (error + error_anterior);
  float u   = Kp * error + Ki * I_k;

  I = I_k;
  error_anterior = error;
  return u;
}

float control_PID(float error, float Kp, float Ki, float Kd, float Ts){
  static float error_anterior = 0, I = 0, D = 0;

  float I_k = I + (Ts / 2.0) * (error + error_anterior);
  float D_k = (2.0 / Ts) * (error - error_anterior) - D;
  float u   = Kp*error + Ki*I_k + Kd*D_k;

  I = I_k;
  D = D_k;
  error_anterior = error;
  return u;
}

int angulo_a_us(float angulo) {
  return 1350 + (-angulo * 350.0 / 11); // cero grados son 1350us, 350us mueve 10.5 grados (medido experimentalmente)
}

void mover_servo(float u, Servo &obj_servo){

    if (u < 900){
        u = 900;
    }

    else if (u > 1800){
        u = 1800;
    }
    obj_servo.writeMicroseconds(u);
}

void matlab_send(float datos[], int n) {
  Serial.write("abcd");

  for (int i = 0; i < n; i++) {
    byte *b = (byte *)&datos[i];
    Serial.write(b, 4);
  };
}


