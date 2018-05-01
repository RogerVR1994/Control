#include "Wire.h"     //Importación de librería para funcionamiento de I2C
#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <SoftwareSerial.h>

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

int LED=7;
int LED2=8;
int LED3=6;
int LED4=5;
String k1;
String k2;
String k3;
String k4;
int n;
int n2;
int n3;
int n4;


SoftwareSerial BT(12,13); //rx,tx


int ax, ay, az;       //Declaración de variables de aceleración en x, y y z

int motA1 = 6;        //Pin 6 de microcontrolador un polo del motor A
int motA2 = 7;        //Pin 7 de microcontrolador un polo del motor A
int motB1 = 10;       //Pin 10 de microcontrolador un polo del motor A
int motB2 = 11;       //Pin 11 de microcontrolador un polo del motor A
int en1 = 8;          //Pin 8 funciona como enable del motor A. Recibe 
                      //señales de PWM para controlar la velocidad del motor
//int boton = 3;
float error = 0;
                      
int en2 = 9;          //Pin 8 funciona como enable del motor A. Recibe 
                      //señales de PWM para controlar la velocidad del motor
                      
int encoder1 = A0;    //Pin Análogo 0 va funciona para recibir señal de encoder 1
int encoder2 = A1;    //Pin Análogo 0 va funciona para recibir señal de encoder 2
float duration,rpm1, rpm, rpm2, periodo; 
//Variables para medir el ancho de pulso, el periodo del encoder y la velocidad de las ruedas

int area = 0;
int direccion;
double angle1 = 0;
double angle2 = 0;

int setPoint = -5;

int K1 = 1;
int K2 = 47;
int K3 = 1;
int K4 = 7;
int velocidad;

int punta = 13;
float output;
 

void setup() {
  //Iniciar pines para motores como salidas
  pinMode(motA1, OUTPUT);
  pinMode(motB1, OUTPUT);
  pinMode(motA2, OUTPUT);
  pinMode(motB2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(punta, OUTPUT);
 // pinMode(boton, OUTPUT);
  //Iniciar pines para motores como entradas
  pinMode(encoder1, INPUT);
  pinMode(encoder2, INPUT);
  BT.begin(9600);   
  //Iniciar comunicación por I2C
  Wire.begin();             
 
  Serial.begin(9600);

  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();

  //Prueba de comunicación entre sendor y microcontrolador
  //if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  //else Serial.println("Error al iniciar el sensor");
 
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float angle1, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    angle1 = filter.getRoll();
    Serial.println(angle1);
    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
 // if(digitalRead(boton)){
//   error = angle1;
  //}
 // Serial.println(error);
 
  area += integrar(angle1, angle2);
  velocidad = derivar(angle1, angle2);
  rpm1 = pos_rueda(encoder1);
 
  float outPut = K1*angle1 + K2*velocidad + K3*rpm1*0.09 + K4*area*0.09;
  if(outPut<120 && outPut>0){
    outPut = 150;
  }

  else if(outPut<0 && outPut >-120){
    outPut = -150;
  }

  else if(outPut>255){
    outPut = 255;
  }
  else if(outPut<-255){
    outPut = -255;
  }

  else if(outPut <0){
    outPut = outPut*-1;
  }
  

  if(angle1>1.2){
    atras(outPut);
    int direccion = 100;
  }

  else if(angle1<1){
    adelante(outPut*1.5);
    int direccion = -100;
  }
  Serial.println(outPut);
   periodo = duration*2;              //Cálculo de periodo de señal de encoder 1
  rpm = pos_rueda(encoder1);
  if(rpm>120){
    rpm = 0;
  }
  else if(rpm<-120){
    rpm=0;
  }
   BT.print(angle1);
   BT.print(",");
   BT.println(direccion);
   
   if(BT.available() >0){


      String num= BT.readString();
      
      n=num.indexOf(',');
      k1= num.substring(0,n);
      K1=k1.toInt();

      n2=num.indexOf(',',n+1);
      k2=num.substring(n+1,n2);
      K2=k2.toInt();

      n3=num.indexOf(',',n2+1);
      k3=num.substring(n2+1,n3);
      K3=k3.toInt();

      n4=num.indexOf(',',n3+1);
      k4=num.substring(n3+1,n4);
      K4=k4.toInt();
   }

   
   angle2 = angle1;
    rpm2 = rpm1;
  
}

void adelante(float output){
    analogWrite(en1, output);
    analogWrite(en2, output);
    digitalWrite(motA1, HIGH);
    digitalWrite(motB1, HIGH);
    digitalWrite(motA2, LOW);
    digitalWrite(motB2, LOW);
}

void atras(float output){
    analogWrite(en1, output);
    analogWrite(en2, output);
    digitalWrite(motA1, LOW);
    digitalWrite(motB1, LOW);
    digitalWrite(motA2, HIGH);
    digitalWrite(motB2, HIGH);
}

float pos_rueda(int encoder){
  duration = pulseIn(encoder, HIGH); //detección de duración de pulso en encoder 1
  periodo = duration*2;              //Cálculo de periodo de señal de encoder 1
  rpm = -0.000000018*pow(periodo, 3)+0.00013*pow(periodo, 2)-0.24*periodo+203.94;
  return rpm;
 
  
}

int integrar(double punto1, double punto2){
  int area = punto1 +((punto2-punto1)/2);
  return area;
}

int derivar(double punto1, double punto2){
  int m = (punto2-punto1);
  return m;
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
 
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
 
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}