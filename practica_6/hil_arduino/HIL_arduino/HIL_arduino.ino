

typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  // Ajustar condiciones iniciales de trabajo
  static float u0=0.5, h_ref=0.4, h=0.45, u;
  static float Ts=1;
  FLOATUNION_t aux;
  static float sampling_period_ms = 1000*Ts;
  //=========================
  // Definir parametros y variables del control

  //=========================

  if (Serial.available() >= 4) {
    aux.number = getFloat();
    h = aux.number;
  }
  //=========================
  //CONTROL
  //error
  float e;
  e = h_ref-h;

  static float e_ant=0.0;
  static float u_ant=0.0;

  u = u_ant - 3.0036*(e-0.9976*e_ant);

  e_ant = e;
  u_ant = u;
    
  matlab_send((u+u0),h_ref,u0);
  delay(sampling_period_ms);
}

void matlab_send(float u, float h, float u0){
  Serial.write("abcd");
  byte * b = (byte *) &u;
  Serial.write(b,4);
  b = (byte *) &h;
  Serial.write(b,4);
  b = (byte *) &u0;
  Serial.write(b,4);
}

float getFloat(){
    int cont = 0;
    FLOATUNION_t f;
    while (cont < 4 ){
        f.bytes[cont] = Serial.read() ;
        cont = cont +1;
    }
    return f.number;
}
