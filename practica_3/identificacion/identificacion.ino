// calibracion del giroscopio, lectura de la imu, 
// estimacion de angulos, filtro complementario,
// envio de datos a simulink, movimiento de la barra con el servo

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

#define SERVO_HORIZONTAL 1400
const int pinServo = 9;

Servo obj_servo;
Adafruit_MPU6050 mpu;

unsigned long tiempo1;
unsigned long tiempo2;
const int T = 20000; // periodo en us
int t_delay = 0;
float dif = 0;


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
  //wx_prom = wx_acc / 100.0;

  // inicializar servo
  obj_servo.attach(pinServo);

}

void loop() {
  
  tiempo1 = micros();

  // obtener mediciones
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 

  // filtro complementario
  const float R = 0.93;
  static float alpha_g_inicial = 0;
  static float alpha_g_comp = alpha_g_inicial + g.gyro.x * 0.02; // estimacion del angulo basada en el giroscopio para el filtro complementario
  static float alpha_a_comp = atan2(a.acceleration.y, a.acceleration.z); // estimacion del angulo basada en el acelerometro para el filtro complementario
  float alpha_f; // estimacion del filtro complementario
  alpha_f = alpha_g_comp*R + alpha_a_comp*(1-R);
  alpha_g_comp = alpha_f + g.gyro.x * 0.02;// - wx_prom; 
  alpha_a_comp = atan2(a.acceleration.y, a.acceleration.z);


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
  static int counter2 = 0; 
  static int counter3 = 0;
  static int counter4 = 0;

  static float angulo = 0;
  if(counter >= 200){
    obj_servo.writeMicroseconds(1400);
    counter = 0;
    angulo = 0;
  };
  if(counter2 >= 400){
     obj_servo.writeMicroseconds(1600);
     counter2 = 0;
     angulo = 10;
  };
  if(counter3 >= 600){
    obj_servo.writeMicroseconds(1400);
    counter3 = 0;
    angulo = 0;
  };
  if(counter4 >= 800){
     obj_servo.writeMicroseconds(1200);
     counter4 = 0;
     angulo = -10;
  };
  counter++;
  counter2++;
  counter3++;
  counter4++;

  float datos[3] = {alpha_f, angulo,1};
  matlab_send_2(datos, 3);
  
}


void matlab_send_2(float datos[], int n) {
  Serial.write("abcd");

  for (int i = 0; i < n; i++) {
    byte *b = (byte *)&datos[i];
    Serial.write(b, 4);
  };
}


void inicio_servo(){
  obj_servo.writeMicroseconds(1400);
}

