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

double accel1, accel2;    //Variable para medir la inclinación del encoder en el eje x del péndulo
int setPoint;
double error;

double gyro1 = 0;
double gyro2 = 0;

int setPoint = 0;

int k1 = 1;
int k2 = 1;
int k3 = 1;
int k4 = 1;
int punta = 13;

void setup() {
  //Iniciar pines para motores como salidas
  pinMode(motA1, OUTPUT);
  pinMode(motB1, OUTPUT);
  pinMode(motA2, OUTPUT);
  pinMode(motB2, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(punta, OUTPUT);
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

  digitalWrite(punta, HIGH);
	sensor.getAcceleration(&ax, &ay, &az); //Obtener Inclinación de péndulo
	gyro1 = atan(ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
  rpm1=rpm(encoder1);
  rpm2=rpm(encoder2);
	
	area += integrar(rpm1, rpm2);
  velocidad  += derivar(gyro1, gyro2);
  output = k1*velocidad+k2*gyro1+k3*rpm+k4*area;
  if (area>0){
    adelante();
  }
  else if(area<0){
    area = area*-1;
    atras();
  }

  gyro2 = gyro1;
  digitalWrite(punta, LOW);
}

void adelante(int output){
    analogWrite(en1, output);
    analogWrite(en2, output);
    analogWrite(motA1, HIGH);
    analogWrite(motB1, LOW);
    digitalWrite(motA2, HIGH);
    digitalWrite(motB2, LOW);
    printData();
}

void atras(int output){
    analogWrite(en1, output);
    analogWrite(en2, output);
    digitalWrite(motA1, LOW);
    digitalWrite(motB1, HIGH);
    digitalWrite(motA2, LOW);
    digitalWrite(motB2, HIGH);
    printData();
}

int rpm(int encoder){
  duration1 = pulseIn(encoder, HIGH); //detección de duración de pulso en encoder 1
  periodo1 = duration1*2;              //Cálculo de periodo de señal de encoder 1
  rpm2 = -0.000000018*pow(periodo2, 3)+0.00013*pow(periodo2, 2)-0.24*periodo2+203.94;
}

double integrar(double gyro1, double gyro2, double error){
	area = gyro1 +((gyro2-gyro1)/2);
	return area;
}

double derivar(double gyro1, double gyro2, double error){
	double m = (gyro2-gyro1);
	return m;
}