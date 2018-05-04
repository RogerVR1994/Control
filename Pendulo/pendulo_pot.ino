int salida_controlador;
int k1, k2, k3, k4;
/*
    k1: ganancia de posición angular del péndulo
    k2: ganancia de velocidad angular del péndulo
    k3: ganancia de posición del motor
    k4: ganancia de velocidad del motor
    
    Se pueden poner en cualquier órden, solo hay que checar
    que coincidan con la ecuación del controlador


*/

int pinEncoder;
int pinPotenciometro;
int en;
int motA;
int motB;

int tiempoDeMuestreo;
int outputControlador;

int angulo1;
int angulo2;

int setPoint;


void setup(){
    pinmode(en, OUTPUT);
    pinmode(motA1, OUTPUT);
    pinmode(motA2, OUTPUT);
    pinmode(motB1, OUTPUT);
    pinmode(motB2, OUTPUT);
}

void loop(){
    angulo1 = analogRead(pinPotenciometro);
    velocidad1 = velocidadMotor(pinEncoder);
    outputControlador = k1*anguloPendulo(pinPotenciometro) + k2*velocidadAngularPendulo(angulo1, angulo2)+ k3*posicionMotor(pinEncoder) + k4*velocidadMotor(velocidad1, velocidad2);
    if(angulo1 < setPoint){
        izquierda(output);
    }
    else{
        derecha(output);
    }

    angulo2 = angulo1;
    velocidad2 = velocidad1;
}


void izquierda(float output){
    analogWrite(en, output);
    digitalWrite(motA, HIGH);
    digitalWrite(motB, LOW);
}

void derecha(float output){
    analogWrite(en, output);
    digitalWrite(motA, LOW);
    digitalWrite(motB, HIGH);
}

int velocidadAngularPendulo(int angulo1, int angulo2){
    int velocidad = (angulo2-angulo1)/(tiempoDeMuestreo);
    return velocidad;
}

int anguloPendulo(int pinPotenciometro){
    /*
        Aquí va el código para leer el ángulo del péndulo
        Como es un pot, puede ser:
        int angulo = analogRead(pinPotenciometro); tu ecuación característica del pot puede ser lo que lee el arduino por algún número 
        return angulo;
    */
}

float velocidadMotor(int pinEncoder){
  float duracionPulso = pulseIn(pinEncoder, HIGH); //detección de duración de pulso en encoder 1
  float periodo = duracionPulso*2;              //Cálculo de periodo de señal de encoder 1
  float rpm = -0.000000018*pow(periodo, 3)+0.00013*pow(periodo, 2)-0.24*periodo+203.94; //Esta es la ecuación del motor. La que está en el Excel
  return rpm;
}

float posicionMotor(float velocidad1, float velocidad2){
    /*Aquí se usa el método trapezoidal*/
    float areaBajoCurva = velocidad2*tiempoDeMuestreo + ((velocidad2-velocidad1)*tiempoDeMuestreo)/2;
    return areaBajoCurva;
}