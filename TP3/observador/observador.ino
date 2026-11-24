
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
float wx_prom = 0.0; // velocidad angular promedio en x [rad/s]

Adafruit_MPU6050 mpu;


// constantes del observador
const float a11 = 1, a12 = 0.0193581, a13 = -6.3436e-05, a14 = -4.0532e-07;
const float a21 = 0, a22 = 0.936505, a23 = -0.00624902, a24 = -5.93747e-05;
const float a31 = 0, a32 = 0, a33 = 0.976017, a34 = 0.0179685;
const float a41 = 0, a42 = 0, a43 = -2.31075, a44 = 0.795972;

const float b1 = -2.62032e-07;
const float b2 = -5.16783e-05;
const float b3 = 0.0237778;
const float b4 = 2.29099;

const float l11 = 0.231917, l12 = -0.00316062;
const float l21 = 0.351402, l22 = -0.0202643;
const float l31 = -0.000687908, l32 = 0.101075;
const float l41 = 0.00122019, l42 = -2.23952;

// inicializar el estado estimado: pos [m], dpos [m/s], tita [grad], dtita [grad/s]
float x1_hat = 0, x2_hat = 0, x3_hat = 0, x4_hat = 0;

float pos_anterior = 0.0;
float pos_actual = 0.0;

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

  obj_servo.writeMicroseconds(angulo_a_us(0));
  delay(1500);
  float suma = 0;
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    suma += g.gyro.x;
    delay(5);
  }
  wx_prom = suma / 500.0;   // rad/s, la misma unidad que usa estimar_angulo_rad
  Serial.print(wx_prom);
}

void loop() {
  
  tiempo1 = micros();    
  
  // obtener mediciones
  actualizar_posicion();
  float dpos = calcular_velocidad();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 
  float dtita = g.gyro.x-wx_prom;
  float tita = estimar_angulo_rad(a, g, dtita) * 180.0 / PI;
  
  
  // mover el servo con una senial periodica
  float u = angulo_perturbacion();
  obj_servo.writeMicroseconds(angulo_a_us(u));

  actualizar_observador(u, pos_actual, tita);

  // enviar a matlab 
  float datos[9] = {u, pos_actual, x1_hat, dpos, x2_hat, tita, x3_hat, dtita*180/PI, x4_hat};
  matlab_send(datos, 9);

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

float actualizar_posicion() {
  //Función para obtener la posicion del carrito (m) usando el sensor de distancia
  unsigned int tiempo_echo = sensor.ping();
  float distancia = tiempo_echo / (2.0 * vs);
  pos_anterior = pos_actual;
  pos_actual = (distancia - 15)/100.0; // en m
}

float calcular_velocidad(){

  float v = (pos_actual - pos_anterior)/(T*1E-6); // velocidad en m/s

  return v;
}

void actualizar_observador(float u, float y1, float y2) {
  // actualiza x_hat[k] -> x_hat[k+1]
  float e1 = y1 - x1_hat;   // pos - pos estimada
  float e2 = y2 - x3_hat;   // tita - tita estimada

  float x1_new = a11*x1_hat + a12*x2_hat + a13*x3_hat + a14*x4_hat + b1*u + l11*e1 + l12*e2;
  float x2_new = a21*x1_hat + a22*x2_hat + a23*x3_hat + a24*x4_hat + b2*u + l21*e1 + l22*e2;
  float x3_new = a31*x1_hat + a32*x2_hat + a33*x3_hat + a34*x4_hat + b3*u + l31*e1 + l32*e2;
  float x4_new = a41*x1_hat + a42*x2_hat + a43*x3_hat + a44*x4_hat + b4*u + l41*e1 + l42*e2;

  x1_hat = x1_new; x2_hat = x2_new; x3_hat = x3_new; x4_hat = x4_new;
}

float estimar_angulo_rad(sensors_event_t a, sensors_event_t g, float dtita){
  // filtro complementario

  const float R = 0.93;
  static float alpha_g_inicial = 0;
  static float alpha_g_comp = alpha_g_inicial + dtita * T*1E-6; // estimacion del angulo basada en el giroscopio para el filtro complementario
  
  float alpha_a_comp = atan2(a.acceleration.y, a.acceleration.z); // estimacion del angulo basada en el acelerometro para el filtro complementario
  float alpha_f = alpha_g_comp*R + alpha_a_comp*(1-R); // estimacion del filtro complementario
  alpha_g_comp = alpha_f + dtita * T*1E-6;

  return alpha_f;
}


void matlab_send(float datos[], int n) {
  Serial.write("abcd");

  for (int i = 0; i < n; i++) {
    byte *b = (byte *)&datos[i];
    Serial.write(b, 4);
  };
}

int angulo_a_us(float angulo) {
  return 1350 + (-angulo * 350.0 / 11); // cero grados son 1350us, 350us mueve 10.5 grados (medido experimentalmente)
}


float angulo_perturbacion(void){
  // mueve al servo en una secuencia periodica de angulos, devuelve el angulo
  const float secuencia_angulos[] = { 2.5, 0.0, -2.5, 0.0};
  const unsigned long periodo_us = 1E6;

  static unsigned long ult_mov_us = 0;
  static int pos_actual = 0;
  static float angulo_actual = secuencia_angulos[pos_actual];
  unsigned long ahora = micros();

  if(ahora - ult_mov_us >= periodo_us){
    pos_actual = (pos_actual+1) % 4;
    angulo_actual = secuencia_angulos[pos_actual];
    ult_mov_us = ahora;
  }

  return angulo_actual;

}