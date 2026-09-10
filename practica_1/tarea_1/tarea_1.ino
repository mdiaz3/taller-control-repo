const int pinPote = A1;
int valorPote = 0;
unsigned long tiempo1 = 0;
unsigned long tiempo2 = 0;
unsigned long t1 = 0;
unsigned long t2 = 0;
unsigned long dif = 0;
float frecPote = 0;
float anguloPote = 0;
const int t_preciso = 20000;
int t_delay = 0;

void setup() {
  Serial.begin(115200);
  t1 = micros();
}

void loop() {

  // Sensor analogico
  // 1. Frecuencia de lectura: 8.6kHz
  //tiempo1 = micros();

  //valorPote = analogRead(pinPote);

  //tiempo2 = micros();
  //dif = tiempo2 - tiempo1;
  //frecPote = (1000000/dif);

  //Serial.print(frecPote);
  //Serial.print("\n");

  // 2. Angulo de la perilla
  //anguloPote = (300.0*valorPote)/1023.0;

  //Serial.print(anguloPote);
  //Serial.print("\n");

  // 3. Leer a 50Hz 
  tiempo1 = micros();

  valorPote = analogRead(pinPote);
  anguloPote = (300.0*valorPote)/1023.0;

  Serial.print(anguloPote);
  Serial.print("|");
  Serial.print(valorPote);
  Serial.print("\n");
  tiempo2 = micros();
  dif = tiempo2 - tiempo1;
  
  t_delay = t_preciso - dif;
  delayMicroseconds(t_delay);

  // tiempo2 = micros();
  // dif = tiempo2 - tiempo1;
  // frecPote = (1000000/dif);
  // Serial.print(frecPote);
  // Serial.print("\n");

}

