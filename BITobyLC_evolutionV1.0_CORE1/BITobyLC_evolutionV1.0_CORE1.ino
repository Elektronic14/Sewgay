
//Software control equilibrio BiToby
//CORE 1 // 16MHz
//DeltaTime:

//AUTOR DANIEL HIDALGO DE GODOS

/*INFORMACION IMPORTANTE:
-Al realizar giros, uno de los motores parece girar más.
-Parece poderse sintonizar mejor los PIDs(no hay ganas?).


  Pin Función Motores
  8    Inversion 1 1             azul
  9    PWM motor 1 Derecha       verde
  10   PWM motor 2 Izquierda     blano
  11   Inversion 2 1             rojo 
  12   Inversion 2 2             gris
  13   Inversion 1 2             amarillo
  
  Pines encoder
  3    Blanco Motor 1
  7    Amarillo Motor1
  2    Blanco Motor 2
  5    Amarillo Motor 2

 
 //LISTA DE FUNCIONES
 
 -void GyroSetup()                                   IN:                       //   OUT:
 -void MotoresPWM(int VMot1,int VMot2)               IN:+-256 bits             //   OUT:PWM signal
 -void PIDMotores(float consigna1,float consigna2)   IN:VelocidadRueda:+-2500  //   OUT:+-256 bits
 -void PIDAngulo(int consigna)                       IN:Angulo:+-40            //   OUT:VelocidadRueda:+-2500
 -void calculoAngulo()                               IN:                       //   OUT:Angulo+-90
 -void PeriodoRueda1()                               IN:                       //   OUT:periodo encoder1
 -void PeriodoRueda2()                               IN:                       //   OUT:periodo encoder2
 -void controlVelocidad(int velocidadCons)           IN:Velocidad usuario      //   OUT:inclinacion necesaria
 -void getRemoteValues()                             IN:                       //   OUT:valores del puerto serie, Rx
 */

//////////////////////////////////////////////////////////
/////////////////Librerias///////////////////////////
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
 
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
int16_t gx, gy, gz;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//Variables del los encoders de los motores

unsigned long time1=0;
int velocidadAnterior1[]={0,0,0};
int sentido1=1;
unsigned long periodoRueda1=5000000;
unsigned long time2=0;
int velocidadAnterior2[]={0,0,0};
int sentido2=1;
unsigned long periodoRueda2=5000000;


//Variables del PID de los motores
float errorMotor1=0;
float errorMotor2=0;
float Km=15;
float Deltatime1PIDmotor=1;
float velocidadRueada=0;
float OutputMotor1=0;
float OutputMotor2=0;
int OffSet=0;
int ValorOff=0;

int i=0;;


//Variables del PIDAngulo
float offSetInicial=-0.2;
float errorAngulo=0;
float Ka=0.23;
float Ia=0;
float Ia1=3;
float Ia2=5;
float time1PIDAngulo=1;
float lastInputAngulo;
float IterAngulo=0;
float consignaVelocidad=0;
float anguloFiltrado=0;
bool recto=0;


//Variables del control de velocidad

float errorVelocidad=0;
float Kv=1;
float Iv=0;
float lastInputVel=0;
float IterVel=0;
float consignaInclinacion=0;
float OffsetMedia=0;
float ajusteVelLineal=0.28;
float velocidadLineal=0;//"La del c.d.g."


//variables control remoto
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int consignaRemota1=0;
int consignaRemota2=0;
int consignaRemota3=0;
int consignaRemotaAnterior1=0;
int consignaRemotaAnterior2=0;
unsigned long time1Lapse=0;
int watchDog=0;
void setup(){
  Serial.begin(115200); //Iniciamos comunicacion serie
   // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  GyroSetup();
  attachInterrupt(1, PeriodoRueda1, RISING );
  attachInterrupt(0, PeriodoRueda2, RISING );   
  Serial.println("Start Serial");
  Serial.println("Pin Setup..");
  pinMode(3,INPUT); 
  pinMode(7,INPUT);//EncoderB 90ºDesfase 
  for(i=8;i<=13;i++){
    pinMode(i,OUTPUT); 
  }
  
  pinMode(14,INPUT);
  Serial.println("Done");
  Serial.println("GyroSetup...");
  Serial.println("DONE");
  Km=10;
  OffSet=40;
  Ka=110;
  Ia=2.25;
  Kv=1;
}
//////////////////Empieza loop///////////////////////
void loop(){
  calculoAngulo();//Almacena en una variable el angulo de inclinacion
  VelocidadRueda1();//Almacena en una variable la velocidad de las ruedas
  VelocidadRueda2();
  getRemoteValues();
  controlVelocidad(consignaRemota1*0.70);//Intenta alcanzar la velocidad de consigna mediante el angulo de inclinacion
  PIDAngulo(consignaInclinacion);//Intenta alcanzar el angulo de consigna mediante la velocidad de las ruedas
  if(abs(anguloFiltrado)<45){//De pie, solo intenta equilibrar cuando está de pie  
    PIDMotores(consignaVelocidad+float(consignaRemota2/float(4)),consignaVelocidad-0.9*float(consignaRemota2/float(4)));//Intenta alcanzar la velocidad de rueda de consigna  
    MotoresPWM(OutputMotor2,OutputMotor1);//Genera un PWM para el puente en H
  }else{//Caida
    IterAngulo=0;//Resetea los integradores cuando hay una caida
    IterVel=0;
    MotoresPWM(0,0);//para los motores
  } 
 
}
//////////////////Termina loop///////////////////////


void MotoresPWM(int VMot1,int VMot2){// VMot:-255 a 255 //accionador de los motores
//Motor 1
  if(VMot1<0){
    digitalWrite(8,LOW);
    digitalWrite(13,HIGH);
  }else{
    digitalWrite(8,HIGH);
    digitalWrite(13,LOW);
  }
  VMot1=abs(VMot1);
  if(VMot1>10){
    analogWrite(9,VMot1);
  }else{
    digitalWrite(9,LOW);
  }
//Motor 2  
  if(VMot2<0){
    digitalWrite(11,HIGH);
    digitalWrite(12,LOW);
  }else{
    digitalWrite(11,LOW);
    digitalWrite(12,HIGH);
  } 
  VMot2=abs(VMot2);
  if(VMot2>10){
    analogWrite(10,VMot2);
  }else{
    digitalWrite(10,LOW);
  }
}

void PIDMotores(float consigna1,float consigna2){
    
    errorMotor1=float((consigna1-velocidadAnterior1[0]*sentido1)/10);
    errorMotor2=float((consigna2-velocidadAnterior2[0]*sentido2)/10);
    
    OutputMotor1= Km*errorMotor1;
    OutputMotor2= Km*errorMotor2;
    
    if(OutputMotor1>255){
      OutputMotor1=255;
    }
    if(OutputMotor1<-255){
      OutputMotor1=-255;
    }
    if(OutputMotor2>255){
      OutputMotor2=255;
    }
    if(OutputMotor2<-255){
      OutputMotor2=-255;
    }
    
    if(consigna1==0&&VelocidadRueda1()<300){//Esto es para el frenado
      OutputMotor1=0;
    } 
    if(consigna2==0&&VelocidadRueda2()<300){//Esto es para el frenado
      OutputMotor2=0;
    }     
}


void PIDAngulo(int consigna){
  
    time1PIDAngulo= 1; 
    if(consigna>14){
      consigna=14;
    }
    if(consigna<-14){
      consigna=-14;
    }
    errorAngulo=-consigna+(anguloFiltrado+offSetInicial);
    
    if(errorAngulo>10){
      errorAngulo=10;
    }
    if(errorAngulo<-10){
      errorAngulo=-10;
    }
      
    IterAngulo+=float(((Ia*errorAngulo))*(time1PIDAngulo));
    if(IterAngulo>2000){//Limitador de integrador
      IterAngulo=2000;
    }else if(IterAngulo<-2000){
      IterAngulo=-2000;
    }
    
    if(abs(anguloFiltrado)>45){
      IterAngulo=0;
    }
    
    consignaVelocidad= Ka*errorAngulo+IterAngulo;
   
    
  
}
void calculoAngulo(){
    if (!dmpReady) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
           
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        
    }
    mpu.getRotation(&gx, &gy, &gz);
    velocidadLineal=sentido1*(velocidadAnterior1[0]+velocidadAnterior2[0])/2;
    anguloFiltrado=ypr[2] * 180/M_PI;
    
  
}
void PeriodoRueda1(){ 
  periodoRueda1=(micros()-time1);
  time1=micros();
  if(digitalRead(7)){
    sentido1=-1;
  }else{
    sentido1=1;
  }
}
void PeriodoRueda2(){ 
  periodoRueda2=(micros()-time2);
  time2=micros();
  if(digitalRead(5)){
    sentido2=1;
  }else{
    sentido2=-1;
  }
}

int VelocidadRueda1(){
  velocidadAnterior1[2]=velocidadAnterior1[1];
  velocidadAnterior1[1]=velocidadAnterior1[0];
  if(micros()-time1>periodoRueda1){
    if(micros()-time1>150000&&velocidadAnterior1[0]<150){
      velocidadAnterior1[0]=0;
    }else{
      velocidadAnterior1[0]=2*velocidadAnterior1[0]-velocidadAnterior1[2]-5; 
      if( velocidadAnterior1[0]<0){
        velocidadAnterior1[0]=0;
      }      
    }
  }else{ 
    velocidadAnterior1[0]= float(1000000/float(periodoRueda1));
  } 
  return velocidadAnterior1[0];    
}
int VelocidadRueda2(){
  velocidadAnterior2[2]=velocidadAnterior2[1];
  velocidadAnterior2[1]=velocidadAnterior2[0];
  if(micros()-time2>periodoRueda2){
    if(micros()-time2>150000&&velocidadAnterior2[0]<150){
      velocidadAnterior2[0]=0;
    }else{
      velocidadAnterior2[0]=2*velocidadAnterior2[0]-velocidadAnterior2[2]-5; 
      if( velocidadAnterior2[0]<0){
        velocidadAnterior2[0]=0;
      }      
    }
  }else{ 
    velocidadAnterior2[0]= float(1000000/float(periodoRueda2));
  } 
  return velocidadAnterior2[0];    
}
void controlVelocidad(float velocidadCons){//La salida es el ángulo de inclinación
   errorVelocidad= velocidadCons-velocidadLineal;
   IterVel+=float(0.05*errorVelocidad/1000);
   
    consignaInclinacion= (0.45*errorVelocidad/100+IterVel*0.1);    
   if(consignaInclinacion>10){
      consignaInclinacion=10;
   }
   if(consignaInclinacion<-10){
     consignaInclinacion=-10;
   }
}
void GyroSetup(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-37);
    mpu.setYGyroOffset(26);
    mpu.setZGyroOffset(-19);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
       
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
}

void serialEvent() {
  while (Serial.available()) {   
    char inChar = (char)Serial.read();   
    inputString += inChar;  
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}
void getRemoteValues(){
  String aux;
  if (stringComplete) {
    if(inputString.length()>25&&inputString.length()<35){
      aux=inputString.substring(inputString.indexOf("/ve1:")+5,30);
      aux=aux.substring(0,aux.indexOf("/"));    
      consignaRemota3=atoi(aux.c_str());
      if(abs(consignaRemota3)>abs(consignaRemota1)){
        consignaRemota1=consignaRemota1*0.995+consignaRemota3*0.005;
      }else{
        consignaRemota1=consignaRemota1*0.9+consignaRemota3*0.1;
      }
      aux=inputString.substring(inputString.indexOf("/ve2:")+5,30);
      aux=aux.substring(0,aux.indexOf("/"));     
      consignaRemota2=consignaRemota2*0.9+0.1*atoi(aux.c_str());
      
      aux=inputString.substring(inputString.indexOf("/wac:")+5,30);
      aux=aux.substring(0,aux.indexOf("/"));
      
      if(atoi(aux.c_str())==1){
        watchDog=watchDog+1;
      }else{
        watchDog=0;
      };
      
      if(abs(consignaRemota1)>2000){
        consignaRemota1=consignaRemotaAnterior1;
      }else{
        consignaRemotaAnterior1=consignaRemota1;  
      }
      if(abs(consignaRemota2)>2000){
        consignaRemota2=consignaRemotaAnterior2;
      }else{
        consignaRemotaAnterior2=consignaRemota2;   
      }
     
      // clear the string:
      inputString = "";
      stringComplete = false;
    }else{
      inputString = "";
      stringComplete = false;
    }
  } 
}
