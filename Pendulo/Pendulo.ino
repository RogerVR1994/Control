#include "I2Cdev.h"   //Importación de librería para protocolo parseo rápido de datos en I2C
#include "MPU6050.h"  //Importación de librería para funcionamiento de Giroscopio
#include "Wire.h"     //Importación de librería para funcionamiento de I2C

MPU6050 sensor;       //Creación de objeto Sensor

int ax, ay, az;       //Declaración de variables de aceleración en x, y y z

int motA1 = 6;        //Pin 6 de microcontrolador un polo del motor A
int motB1 = 7;        //Pin 7 de microcontrolador un polo del motor A
int motA2 = 10;       //Pin 10 de microcontrolador un polo del motor A
int motB2 = 11;       //Pin 11 de microcontrolador un polo del motor A
int en1 = 8;          //Pin 8 funciona como enable del motor A. Recibe 
                      //señales de PWM para controlar la velocidad del motor
                      
int en2 = 9;          //Pin 8 funciona como enable del motor A. Recibe 
                      //señales de PWM para controlar la velocidad del motor
                      
int encoder1 = A0;    //Pin Análogo 0 va funciona para recibir señal de encoder 1
int encoder2 = A1;    //Pin Análogo 0 va funciona para recibir señal de encoder 2
float duration1, duration2, rpm1, rpm2, periodo1, periodo2; 
//Variables para medir el ancho de pulso, el periodo del encoder y la velocidad de las ruedas

float accel_ang_x;    //Variable para medir la inclinación del encoder en el eje x del péndulo

void setup() {
  //Iniciar pines para motores como salidas
  pinMode(motA1, OUTPUT);
  pinMode(motB1, OUTPUT);
  pinMode(motA2, OUTPUT);
  pinMode(motB2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  //Iniciar pines para motores como entradas
  pinMode(encoder1, INPUT);
  pinMode(encoder2, INPUT);
  Serial.begin(115200);   
  //Iniciar comunicación por I2C
  Wire.begin();             
  sensor.initialize();  //Función para inicializar giroscopio

  //Prueba de comunicación entre sendor y microcontrolador
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}

void loop() {
  
  sensor.getAcceleration(&ax, &ay, &az); //Obtener Inclinación de péndulo
  accel_ang_x=atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14); //Ecuación usada para caracterizar giroscopio
  if (accel_ang_x>8){
    atras();
  }
  else if(accel_ang_x<7){
    adelante();
  }
  else{
    analogWrite(en1, 255);
    analogWrite(en2, 255);
    digitalWrite(motA1, LOW);
    digitalWrite(motB1, LOW);
    digitalWrite(motA2, LOW);
    digitalWrite(motB2, LOW);
  }
}

void adelante(){
    analogWrite(en1, 255);
    analogWrite(en2, 255);
    digitalWrite(motA1, HIGH);
    digitalWrite(motB1, LOW);
    digitalWrite(motA2, HIGH);
    digitalWrite(motB2, LOW);
    printData();
}

void atras(){
    analogWrite(en1, 255);
    analogWrite(en2, 255);
    digitalWrite(motA1, LOW);
    digitalWrite(motB1, HIGH);
    digitalWrite(motA2, LOW);
    digitalWrite(motB2, HIGH);
    printData();
}

void printData(){
  duration1 = pulseIn(encoder1, HIGH); //detección de duración de pulso en encoder 1
  duration2 = pulseIn(encoder2, HIGH); //detección de duración de pulso en encoder 2
  periodo1 = duration1*2;              //Cálculo de periodo de señal de encoder 1
  periodo2 = duration2*2;              //Cálculo de periodo de señal de encoder 2
  rpm1 = -0.000000018*pow(periodo1, 3)+0.00013*pow(periodo1, 2)-0.24*periodo1+203.94;  //Ecuaciones características para motores 
  rpm2 = -0.000000018*pow(periodo2, 3)+0.00013*pow(periodo2, 2)-0.24*periodo2+203.94;
  Serial.println("Motor1      Motor2    Inclinacion"); //Despliegue de datos
  Serial.print(rpm1);
  Serial.print("rpm   ");
  Serial.print(rpm2);
  Serial.print("rpm   ");
  Serial.println(accel_ang_x);
 
}
