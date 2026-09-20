// calibracion del giroscopio, lectura de la imu, 
// estimacion de angulos, filtro complementario,
// envio de datos a simulink, movimiento de la barra con el servo

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

  // Calibrar giroscopio
  float wx_acc = 0; // velocidad angular acumulada en x
  sensors_event_t a, g, temp;
  for (int i = 0; i < 100; i++){
    mpu.getEvent(&a, &g, &temp);
    wx_acc = wx_acc + g.gyro.x;
    delay(20);
  };
  wx_prom = wx_acc / 100.0;
  Serial.print("Velocidad angular promedio en x: ");
  Serial.print(wx_prom);
  Serial.print("\n");
  // inicializar servo
  obj_servo.attach(pinServo);

}

void loop() {
  
  tiempo1 = micros();

  // obtener mediciones
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 

  // estimacion de los angulos
  static float alpha_g; // estimacion del angulo basada en el giroscopio
  static float alpha_a; // estimacion del angulo basada en el acelerometro
  static float alpha_g_anterior = 0;
  alpha_g = alpha_g_anterior + (g.gyro.x - wx_prom)* T*1E-6; 
  alpha_g_anterior = alpha_g;
  alpha_a = atan2(a.acceleration.y, a.acceleration.z);

  // filtro complementario
  const float R = 0.93;
  static float alpha_g_comp = alpha_g; // estimacion del angulo basada en el giroscopio para el filtro complementario
  static float alpha_a_comp = alpha_a; // estimacion del angulo basada en el acelerometro para el filtro complementario
  float alpha_f; // estimacion del filtro complementario
  alpha_f = alpha_g_comp*R + alpha_a_comp*(1-R);
  alpha_g_comp = alpha_f + (g.gyro.x - wx_prom) * T*1E-6; 
  alpha_a_comp = atan2(a.acceleration.y, a.acceleration.z);

  // controlar la frecuencia de muestreo
  tiempo2 = micros();
  dif = tiempo2 - tiempo1;
  t_delay = T - dif;
  if(t_delay >= 16383){
    delay(10);
    delayMicroseconds(t_delay-10000);
  };
  
  mover_servo(obj_servo);

  // enviar a matlab para ajustar el R
  float datos[3] = {alpha_f*180/3.1415, alpha_g_comp*180/3.1415, alpha_a_comp*180/3.1415};
  matlab_send(datos, 3);

}

void mover_servo(Servo &obj_servo){

  // mueve al servo en una secuencia periodica de angulos
  const float secuencia_angulos[] = {15.0, 0.0, -15.0, 0.0};
  const unsigned long periodo_us = 2E6;

  static unsigned long ult_mov_us = 0;
  static int pos_actual = 0;
  unsigned long ahora = micros();
  
  if(ahora - ult_mov_us >= periodo_us){
    float us = angulo_a_us(secuencia_angulos[pos_actual]);
    obj_servo.writeMicroseconds(us);
    pos_actual = (pos_actual+1) % 4;
    ult_mov_us = ahora;
  };

}

void matlab_send(float datos[], int n) {
  Serial.write("abcd");

  for (int i = 0; i < n; i++) {
    byte *b = (byte *)&datos[i];
    Serial.write(b, 4);
  };
}

int angulo_a_us(float angulo) {
  return 1350 + (angulo * 350.0 / 10.5); // cero grados son 1350us, 350us mueve 10.5 grados (medido experimentalmente)
}

