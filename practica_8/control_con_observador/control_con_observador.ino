#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
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
float wx_prom = 0; //sesgo en la velocidad angular

Adafruit_MPU6050 mpu;

// constantes del observador
  // A_d = I + A*T
  const float a11 = 1.0,    a12 = 0.02;
  const float a21 = -2.572, a22 = 0.7996; 
  // B_d = B*T
  const float b1 = 0.0, b2 = 2.55;
  // L_d (vector columna 2x1, sale de place)
  const float l1 = 0.2965, l2 = 2.7897;

// matriz de ganancias y compensador
const float K1 = -0.3825, K2 = -0.045;
const float F = 1.3911;

float x1_hat = 0.0;   // ángulo estimado
float x2_hat = 0.0;   // velocidad angular estimada  

void setup(void) {
  Serial.begin(115200);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  delay(100);

  // inicializar servo
  obj_servo.attach(pinServo);

}

void loop() {
  
  tiempo1 = micros();
  bool step = true;
  float v = step ? 10.0 : 0.0; // grados del escalon

  // referencia del angulo de la barra en grados
  float referencia = 0+v;

  // obtener mediciones y estimar el angulo
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 
  float alpha_f = estimar_angulo_rad(a, g) * 180.0 / PI;
  float wx = g.gyro.x;
  
  // controlador + perturbacion
  float u = controlador_PD(referencia); 
  mover_servo(angulo_a_us(u), obj_servo);

  // observador
  actualizar_observador(alpha_f,u);
  
  // enviar datos a matlab {accion de control, angulo medido, angulo estimado, velocidad medida, velocidad estimada}
  float datos[5] = {u, alpha_f, x1_hat, wx*180/PI, x2_hat};
  matlab_send(datos, 5);


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

float controlador_PD(float r){
  float u = K1*x1_hat + K2*x2_hat + F*r; // accion de control en grados
  return u;

}

void actualizar_observador(float y, float u) {
  
  // y_hat = C_d * x_hat  (C_d = [1 0], o sea, el primer estado)
  float y_hat = x1_hat;

  // error de salida
  float e = y - y_hat;

  // x_hat[k+1] = A_d*x_hat[k] + B_d*u + L_d*(y - y_hat)
  float x1_nuevo = a11*x1_hat + a12*x2_hat + b1*u + l1*e;
  float x2_nuevo = a21*x1_hat + a22*x2_hat + b2*u + l2*e;

  x1_hat = x1_nuevo; //angulo estimado en grados
  x2_hat = x2_nuevo; //velocidad angular estimada en grados/s
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

float angulo_a_us(float angulo) {
  return 1350 + (-angulo * 350.0 / 11); // cero grados son 1350us, 350us mueve 10.5 grados (medido experimentalmente)
}

float us_a_angulo(float us) {
  return 11*(1350.0-us)/350.0;
}

void mover_servo(float u, Servo &obj_servo){

    if (u < 400){
        u = 400;
    }

    else if (u > 2500){
        u = 2500;
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

float angulo_perturbacion(void){
  // mueve al servo en una secuencia periodica de angulos, devuelve el angulo
  const float secuencia_angulos[] = {10.0, 0.0, 0.0, 0.0, -10.0, 0.0, 0.0, 0.0};
  const unsigned long periodo_us = 4E6;

  static unsigned long ult_mov_us = 0;
  static int pos_actual = 0;
  static float angulo_actual = secuencia_angulos[pos_actual];
  unsigned long ahora = micros();

  if(ahora - ult_mov_us >= periodo_us){
    pos_actual = (pos_actual+1) % 8;
    angulo_actual = secuencia_angulos[pos_actual];
    ult_mov_us = ahora;
  }

  return angulo_actual;

}

