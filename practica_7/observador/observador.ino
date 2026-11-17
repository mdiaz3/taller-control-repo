
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
float wx_prom = 0; // velocidad angular promedio en x

Adafruit_MPU6050 mpu;

// Control PI
const float Kp = 0.6;
const float Ki = 6;

// constantes del observador
  // A_d = I + A*T
const float a11 = 1.0,    a12 = 0.02;
const float a21 = -2.572, a22 = 0.7996; 
  // B_d = B*T
const float b1 = 0.0, b2 = 2.55;
  // L_d (vector columna 2x1, sale de place)
const float l1 = 2.1282, l2 = 49.2712;  // x=2 > l1 = 0.5863, l2 = 8.2942;   

float x1_hat = 0.0;   // ángulo estimado
float x2_hat = 0.0;   // velocidad angular estimada  

float error_anterior = 0;
float u_anterior = 0;

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
  
  // obtener mediciones y estimar el angulo
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 
  float alpha_f = estimar_angulo_rad(a, g) * 180.0 / 3.1415;
  float wx = g.gyro.x;

  // mover el servo con una senial periodica
  float u = angulo_perturbacion();
  obj_servo.writeMicroseconds(angulo_a_us(u));

  actualizar_observador(alpha_f,u);

  // enviar a matlab > {senial de control, angulo medido por filtro complementario, angulo estimado por observador}
  float datos[5] = {u, alpha_f, x1_hat, wx*180/3.1415, x2_hat};
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

float control_PI(float referencia, float angulo){

  float error = referencia - angulo;

  float u = u_anterior
            + Kp * (error - error_anterior)
            + (Ki * T * 1E-6 / 2.0) * (error + error_anterior);

  error_anterior = error;
  u_anterior = u;

  return u;
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
  const float secuencia_angulos[] = {10.0, 0.0, -10.0, 0.0};
  const unsigned long periodo_us = 5E6;

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