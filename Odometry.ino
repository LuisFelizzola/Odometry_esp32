/*


CODIGO PARA LEER LA VELOCIDAD Y ESTIMAR LA POSICION DEL ROBOT
*/

#define motor1A 2
#define motor1B 4
#define en1 14
#define motor2A 16
#define motor2B 17
#define en2 18
#define motor3A 19
#define motor3B 21
#define en3 22
#define motor4A 23
#define motor4B 25
#define en4 26
#define encoderA1 27
#define encoderB1 32
#define encoderA2 33 
#define encoderB2 34
#define encoderA3 35
#define encoderB3 36 
#define encoderA4 37 
#define encoderB4 38

const int freq=3000;
const int pwmChannel=0,pwmChannel2=1,pwmChannel3=2,pwmChannel4=3;
const int resolution=8;
long prevT1=0,prevT2=0,prevT3=0,prevT4=0; //tiempo previo
volatile int pos_i1=0,pos_i2=0,pos_i3=0,pos_i4=0; //posicion de la rueda 1 a la 4
int prevPos1=0, prevPos2,prevPos3=0,prevPos4=0;

TaskHandle_t Tarea0,Tarea1,Tarea2,Tarea3;

//variables para el filtro pasa-bajas con frecuencia de corte de 25Hz

float vFilt1=0,vFilt2=0,vFilt3=0,vFilt4=0;
float vPrev1=0,vPrev2=0,vPrev3=0,vPrev4=0;


void setup() {

  xTaskCreatePinnedToCore(readEncoder1,"Tarea_0",1000,NULL,1,&Tarea0,1);
  xTaskCreatePinnedToCore(readEncoder2,"Tarea_1",1000,NULL,1,&Tarea1,1);
  xTaskCreatePinnedToCore(readEncoder3,"Tarea_2",1000,NULL,1,&Tarea2,1);
  xTaskCreatePinnedToCore(readEncoder4,"Tarea_3",1000,NULL,1,&Tarea3,1);


  setPins(); // configuro cuáles son mis entradas y salidas

  Serial.begin(115200);

}
//LEO EL ENCODER DEL MOTOR 1
void readEncoder1(void *parameter){

  while(1==1){
    int b= digitalRead(encoderB1);
    pos_i1=incrementarPos(b,pos_i1);
    delayMicroseconds(111); //cada 111 us
  }
}
//LEO EL ENCODER DEL MOTOR 2
void readEncoder2(void *parameter){
  while(1==1){
    int b= digitalRead(encoderB2);
    pos_i2=incrementarPos(b,pos_i2);
    delayMicroseconds(110); //cada 110 us
  }
}
//LEO EL ENCODER DEL MOTOR 3
void readEncoder3(void *parameter){
  while(1==1){
     int b= digitalRead(encoderB3);
    pos_i3=incrementarPos(b,pos_i3);
    delayMicroseconds(112); //cada 112 us   
  }
}

//LEO EL ENCODER DEL MOTOR 4
void readEncoder4(void *parameter){
  while(1==1){
     int b= digitalRead(encoderB4);
    pos_i4=incrementarPos(b,pos_i4);
    delayMicroseconds(113); //cada 113 us   
  }
}

//funcion para incrementar la posicion de cada rueda cuando el flanco del encoder es activado
int incrementarPos(int b,int posI){
  int i=0;
  if (b>0){ //si el motor se mueve counter clockwise
    i=1;
  }else{
    i=-1;
  }
  return posI+i; //retorno el valor de la posición inicial más el incremento
}

//función para configurar mis entradas y salidas
void setPins(){
  pinMode(motor1A,OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(en1,OUTPUT);

  pinMode(motor2A,OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(en2,OUTPUT);

  pinMode(motor2A,OUTPUT);
  pinMode(motor2B, OUTPUT);
  pinMode(en2,OUTPUT);

  pinMode(motor3A,OUTPUT);
  pinMode(motor3B, OUTPUT);
  pinMode(en3,OUTPUT);

  pinMode(motor4A,OUTPUT);
  pinMode(motor4B, OUTPUT);
  pinMode(en4,OUTPUT);


  pinMode(encoderA1,INPUT);
  pinMode(encoderB1,INPUT);

  pinMode(encoderA2,INPUT);
  pinMode(encoderB2,INPUT);

  pinMode(encoderA3,INPUT);
  pinMode(encoderB3,INPUT);

  pinMode(encoderA4,INPUT);
  pinMode(encoderB4,INPUT);

  //configuracion del las funcionalidades del PWM
  ledcSetup(pwmChannel,freq,resolution);
  ledcSetup(pwmChannel2,freq,resolution);
  ledcSetup(pwmChannel3,freq,resolution);
  ledcSetup(pwmChannel4,freq,resolution);

  //Relacionar los pines a ese canal
  ledcAttachPin(en1,pwmChannel);
  ledcAttachPin(en2,pwmChannel2);
  ledcAttachPin(en3,pwmChannel3);
  ledcAttachPin(en4,pwmChannel4);
}
void loop() {
  //move forward
  setMotor(motor1A,motor1B,en1,200,1,0,pwmChannel);
  setMotor(motor2A,motor2B,en2,200,1,0,pwmChannel2);
  setMotor(motor3A,motor3B,en3,200,1,0,pwmChannel3);
  setMotor(motor4A,motor4B,en4,200,1,0,pwmChannel4);
  long currentTime=micros();
  //guardar posicion de los motores, en un bloque átomico para guardar las posiciones,desactivo las interrupciones (por si alguna ocurre).
  int pos1=0,pos2=0,pos3=0,pos4=0;
  noInterrupts();
  pos1=pos_i1;
  pos2=pos_i2;
  pos3=pos_i3;
  pos4=pos_i4;
  interrupts(); //las vuelvo a activar, una vez llene los datos de la posición
  //calculate the veloocity in RPM and save it
  //I know the encoder resolution is: 1 rev/2ct. ct/s-*rev/2ct *60s/min
  float velocity1=calculateVel(currentTime,prevPos1,pos1,prevT1,vFilt1,vPrev1)*(60/2);
  prevT1=currentTime;
  prevPos1=pos1;
  float velocity2=calculateVel(currentTime,prevPos2,pos2,prevT2,vFilt2,vPrev2)*(60/2);
  prevT2=currentTime;
  prevPos2=pos2;
  float velocity3=calculateVel(currentTime,prevPos3,pos3,prevT3,vFilt3,vPrev3)*(60/2);
  prevT3=currentTime;
  prevPos3=pos3;
  float velocity4=calculateVel(currentTime,prevPos4,pos4,prevT4,vFilt4,vPrev4)*(60/2);
  prevT4=currentTime;
  prevPos4=pos4;

  Serial.print(velocity1);
  Serial.println();
  Serial.print(velocity2);
  Serial.println();
  Serial.print(velocity3);
  Serial.println();
  Serial.print(velocity4);
  Serial.println();
  delay(200);
}
//funcion para encender un motor
void setMotor(int inA,int inB, int enA, int dutycicle, int state1,int state2,int pwmCh){
    digitalWrite(inA,state1);
    digitalWrite(inB,state2);
    ledcWrite(pwmCh,dutycicle);
}
//calculo la velocidad teniendo en cuenta que v= dx/dt. Además aplico el filtro pasa bajas con frecuencia de corte de 25hz
float calculateVel(long ct,int prevPos, int pos, long prevT,float &vFlt, float &vPrev){
  float vel=0;
  float deltaT=((float)(ct-prevT))/1.0e6; //paso el dx a s
  vel= (pos-prevPos)/deltaT;

  //aplico el filtro pasa-bajas
  vFlt=0.854*vFlt+0.0728*vel+0.0728*vPrev;
  vPrev=vel;  
  return vFlt;
}
