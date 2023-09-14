#include <Robot.h> //importo la clase robot y rueda
#include <Wheel.h>
#include <Odometry.h> //importo las funciones de la odometria
/*
PINS I CANNOT USE
5,16,17,35
PINES DEL MOTOR1: 2,4,14 . ENCODER MOTOR 1:A-27, B-32.
PINES DEL MOTOR 2: 18,19,21 ENCODER MOTOR 2: A-33, B-34.
CODIGO PARA LEER LA VELOCIDAD Y ESTIMAR LA POSICION DEL ROBOT
*/

//RUEDAS DELANTERAS
#define motor1A 2
#define motor1B 4
#define en1 14
#define motor2A 18
#define motor2B 19
#define en2 21
//RUEDAS TRASERAS
#define motor3A 22
#define motor3B 23
#define en3 25
#define motor4A 13
#define motor4B 12
#define en4 15

#define encoderA1 33
#define encoderB1 32
#define encoderA2 27 
#define encoderB2 34

#define encoderA3 35
#define encoderB3 5
#define encoderA4 36
#define encoderB4 39

const int freq=3000;
const int pwmChannel=0,pwmChannel2=1,pwmChannel3=2,pwmChannel4=3;
const int resolution=8;
long prevT=0; //tiempo previo
volatile int pos_i1=0,pos_i2=0,pos_i3=0,pos_i4=0; //posicion de la rueda 1 a la 4
int prevPos1=0, prevPos2,prevPos3=0,prevPos4=0;
float r=0.04; //radio de la llanta
float Lx=0.116, Ly=0.055; //MITAD DE LA DISTANCIA ENTRE LAS LLANTAS TRASERAS Y LAS LLANTAS DELANTERAS (m)
//variables para el filtro pasa-bajas con frecuencia de corte de 25Hz
float vFilt1=0,vFilt2=0,vFilt3=0,vFilt4=0;
float vPrev1=0,vPrev2=0,vPrev3=0,vPrev4=0;
int dtc=250;
struct odometry odom;
//posicion Inicial del robot:
float xi=0, yi=0,tetai=0;
char mv=' '; //variable para validar que tipo de movimiento esta Mercury (pruebas)
TaskHandle_t Tarea0,Tarea1,Tarea2,Tarea3;
Wheel wh1(motor1A, motor1B, en1,r, pwmChannel);
Wheel wh2(motor2A, motor2B, en2,r, pwmChannel2);
Wheel wh3(motor3A, motor3B, en3,r, pwmChannel3);
Wheel wh4(motor4A, motor4B, en4,r, pwmChannel4);
//DEFINO LOS PARAMETROS DEL OBJETO DE MI CLASE ROBOT (CREADA PARA EL ROBOT MECANUM )
Robot Mercury(&wh1,&wh2,&wh3,&wh4,xi,yi,tetai);
void setup() {

  xTaskCreatePinnedToCore(printing,"Tarea_0",1000,NULL,1,&Tarea0,0);
  attachInterrupt(encoderA1, readEncoder1, RISING);
  attachInterrupt(encoderA2, readEncoder2, RISING);
  attachInterrupt(encoderA3, readEncoder3, RISING);
  //attachInterrupt(encoderA4, readEncoder4, RISING);
  setPins(); // configuro cuáles son mis entradas y salidas

  Serial.begin(115200);

}
//LEO EL ENCODER DEL MOTOR 1
void readEncoder1(){//void *parameter
    int b= digitalRead(encoderB1);
    pos_i1=incrementarPos(b,pos_i1);
}
//LEO EL ENCODER DEL MOTOR 2
void readEncoder2(){//void *parameter
    int b= digitalRead(encoderB2);
    pos_i2=incrementarPos(b,pos_i2);
}
void printing(void *parameter){
  while(1==1){
    char a=Serial.read();
    if(a=='F'){ //forward (same velocyty for all of them)
    Mercury.moveForward(dtc); //metodo de la clase Robot, solo necesita como parametro la velocidad
    mv='F';
    }else if (a=='B'){ //backward (same velocyty for all of them)
    Mercury.moveBackward(dtc);
    mv='B';
    }else if (a=='R'){ //right (u1=u3, u2=u4). u1=-u2
    Mercury.moveRight(dtc);
    mv='R';
    }else if (a=='L'){ //left
    Mercury.moveLeft(dtc);
    mv ='L';
    }else if(a=='I'){
    Mercury.rotation(dtc);
    mv ='I';
    }else if(a=='S'){ //stop
    Mercury.stop();
    mv ='S';
    }else if(a=='D'){
      dtc=50;
    }
    delay(200);
  }
}
//LEO EL ENCODER DEL MOTOR 3

void readEncoder3(){
  int b= digitalRead(encoderB3);
  pos_i3=incrementarPos(b,pos_i3);
}

//LEO EL ENCODER DEL MOTOR 4
void readEncoder4(){
  int b= digitalRead(encoderB4);
  pos_i4=incrementarPos(b,pos_i4);
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
  ledcAttachPin(en3, pwmChannel3);
  ledcAttachPin(en4, pwmChannel3);

}
void loop() {
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
  //I know the encoder resolution is: 1 rev/2ct. ct/s*rev/2ct *60s/min
  //RESOLUTION OF THE ENCODER es de ct/s*0.2 = RPM. ct/s*0.2/60 = rev/s 
  /*  CODIGO PARA PROBAR LA CALIDAD DE LOS 4 ENCODERS, SE ESCOGE EL MEJOR DE LOS 4
  float w1=calculateW(currentTime,prevPos1,pos1,prevT,vFilt1,vPrev1)*0.2; //RPM
  float w2=calculateW(currentTime,prevPos2,pos2,prevT,vFilt2,vPrev2); //ct /s 
  float w3=calculateW(currentTime,prevPos3,pos3,prevT,vFilt3,vPrev3)* 
  float w4=calculateW(currentTime,prevPos4,pos4,prevT,vFilt4,vPrev4)* //ct/s  
  */
  //el encoder con mejor rendimiento
  float w1 =calculateW(currentTime,prevPos1,pos1,prevT,vFilt1,vPrev1)*0.2/60; //convierto la velocidad de ct/s a rv/s. teniendo en cuenta la resolucion del encoder
  float w2,w3,w4;
  prevPos1=pos1;
  /*
  prevPos2=pos2;
  prevPos3=pos3;
  prevPos4=pos4;
  */
  switch (mv){ //al ser el movimiento de Mercury reestringido, asi debe ser mis calculos para cumplir con la reestriccion en las formulas
    
    case 'F': //forward or bakward
    case 'B':
         w2 =w1;
         w3= w1;
         w4= w1;
        break;
    case 'R': //sideways
    case 'L':
         w2=-w1;
         w3=w2;
         w4=w1;
        break;
    case 'I': //rotation
         w2=-w1;
         w3=w1;
         w4=w2;
        break;
      
    case 'S': //stop
        w1=0;
        w2=0;
        w3=0;
        w4=0;
        pos1=0;
        break;
  }
  Mercury.wheel_1->setVelocity(w1);
  Mercury.wheel_2->setVelocity(w2);
  Mercury.wheel_3->setVelocity(w3);
  Mercury.wheel_4->setVelocity(w4);
//VELOCIDAD DE LA RUEDA EN RADIANES:
  float w1r=Mercury.wheel_1->Velocityradians(); //para que pueda guardar todo el calculo posterior en una variable float (numero muy grande)
  float w2r=Mercury.wheel_2->Velocityradians(); 
  float w3r=Mercury.wheel_3->Velocityradians();
  float w4r=Mercury.wheel_4->Velocityradians();   
  forwardKinematics(odom, w1r, w2r, w3r, w4r,  r,  Lx,  Ly,  prevT, currentTime); //estimo la posicion
  prevT=currentTime;

  Mercury.setPosition(odom.x, odom.y, odom.teta); //configuro la posicion de Mercury
  Serial.print(odom.x);
  Serial.print(" ");
  Serial.print(odom.y);
  Serial.print(" ");
  Serial.print(odom.teta);
  Serial.print(" ");
  //Serial.print(" ");
  Serial.print(odom.vx);
  Serial.print(" ");
  Serial.print(odom.vy);
  Serial.println();
  delay(1000);
}



