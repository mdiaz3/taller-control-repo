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
  for (int i = 0; i==10; i++){
    mpu.getEvent(&a, &g, &temp);
    wx_acc = wx_acc + g.gyro.x;
    delay(20);
  };
  wx_prom = wx_acc / 100.0;

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
  const float alpha_g_inicial = 0;
  alpha_g = alpha_g_inicial + g.gyro.x * T - wx_prom; 
  alpha_a = atan2(a.acceleration.y, a.acceleration.z);

  // filtro complementario
  const float R = 0.9;
  static float alpha_g_comp = alpha_g; // estimacion del angulo basada en el giroscopio para el filtro complementario
  static float alpha_a_comp = alpha_a; // estimacion del angulo basada en el acelerometro para el filtro complementario
  float alpha_f; // estimacion del filtro complementario
  alpha_f = alpha_g_comp*R + alpha_a_comp*(1-R);
  alpha_g_comp = alpha_f + g.gyro.x * T - wx_prom; 

  // enviar a matlab
  float datos[2] = {alpha_g, alpha_a};
  matlab_send_2(datos, 2);
  //float datos[3] = {alpha_g_comp, alpha_a_comp, alpha_f};
  //matlab_send_2(datos, 3);

  // controlar la frecuencia de muestreo
  tiempo2 = micros();
  dif = tiempo2 - tiempo1;
  t_delay = T - dif;
  if(t_delay >= 16383){
    delay(10);
    delayMicroseconds(t_delay-10000);
  };
  
  // mover servo cada ~ 200 * 20ms = 4s
  static int counter = 0;
  obj_servo.writeMicroseconds(angulo_a_us(0));
  if(counter >= 200){
    obj_servo.writeMicroseconds(angulo_a_us(60));
    counter = 0;
  };
  counter++;

}

void matlab_send(float dato1, float dato2, float dato3){
  Serial.write("abcd");
  byte * b = (byte *) &dato1;
  Serial.write(b,4);
  b = (byte *) &dato2;
  Serial.write(b,4);
  b = (byte *) &dato3;
  Serial.write(b,4);
  //etc con mas datos tipo float. Tambien podría pasarse como parámetro a esta funcion un array de floats.
}

void matlab_send_2(float datos[], int n) {
  Serial.write("abcd");

  for (int i = 0; i < n; i++) {
    byte *b = (byte *)&datos[i];
    Serial.write(b, 4);
  };
}

int angulo_a_us(float angulo) {
  return 500 + (angulo * 1000.0 / 60.0);
}

