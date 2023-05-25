#include <Math.h>
#include <Encoder.h>

#define ENCODER_A       2 // Pin A del encoder
#define ENCODER_B       8 // Pin B del encoder
#define BUTTON_FORWARD  5 // Giro a la derecha
#define BUTTON_BACKWARD 4 // Giro a la izquierda
#define ERROR_THRESHOLD 0.1
#define REFERENCE_SPEED 700

float error = 0;

const int analogPin = A0; // Pin analógico utilizado para la lectura
const int motor_pin1 = 11;  // Pin de control del motor H-bridge (1A)
const int motor_pin2 = 3; // Pin de control del motor H-bridge (2A)

Encoder encoder(ENCODER_A, ENCODER_B);

volatile unsigned tiempoInterrupcionActual=0;
volatile unsigned tiempoInterrupcionAnterior=0;
volatile unsigned deltaTiempoInterrupcion=0;
volatile unsigned cambioGiroTAnterior = 0;
volatile unsigned deltaCambioGiro = 0;
volatile unsigned pausaAnterior = 0;
volatile unsigned deltaPausa = 0;


volatile unsigned muestreoAnterior=0; //variables de medicion de tiempo
volatile unsigned muestreoActual=0;   //controlan el tiempo de muestreo
volatile double deltaMuestreo=0;
float resolucion = 48.0; // 48 cuentas por vuelta

int contador=0;
double frecuencia =0;   //frecuencia de las interrupciones
double rpm =0;        //velocidad llanta en rpm (revoluciones por minuto)
double posicionGrados = 0; // Posicion en grados
bool giro = true;
int countPV = 0;
int cambiosGiro = 0;
int sentidoGiro = 0;
int pwmValue = 225;
/*
 * 124 c = 5.02V
 * 149 c = 6.02V
 * 177 c = 7.02V
 * 202 c = 8.01V
 * 227 c = 9.01V
 */

void setup() {
  //attachInterrupt(digitalPinToInterrupt(ENCODER_A),Encoder,FALLING);
  
  //Encoders como entradas
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  //Pulsadores
  pinMode(BUTTON_FORWARD, INPUT);
  pinMode(BUTTON_BACKWARD, INPUT);  

  pinMode(motor_pin1, OUTPUT);
  pinMode(motor_pin2, OUTPUT);
  
  Serial.begin(9600);             //Inicio de comunicacion serial, velocidad de comunicacion

}

float inputAnalog() {
  int ia = analogRead(analogPin);  

  return map(ia, 0, 1023, 0, 5) * 5;
}


void encoder_1 ()
{
  //contador++;  
  contador = encoder.read();

  //Serial.println(contador);

  //if (contador == resolucion) {
    //contador = 0;
    frecuencia = (1000)/ ((double) deltaTiempoInterrupcion); //frecuencia de las interrupciones
  
    tiempoInterrupcionAnterior= tiempoInterrupcionActual;   
  //}  
}

void controlOnOff(float velocidad_ang) {
  // Calcular el error relativo
  error = (float)(REFERENCE_SPEED - velocidad_ang);
  float errorPorcentaje = (float) error / REFERENCE_SPEED;

  //Serial.println(errorPorcentaje);
  
  // Control On-Off
  /*if (errorPorcentaje > ERROR_THRESHOLD) {
    // Aumentar la velocidad del motor
    analogWrite(motor_pin1, 255);    
  } else {
    // Mantener la velocidad actual
    digitalWrite(motor_pin1, LOW);    
  } 
  digitalWrite(motor_pin2, LOW); */
  // Control On-Off
  if (errorPorcentaje < -ERROR_THRESHOLD) {
    // Aumentar la velocidad del motor
    digitalWrite(motor_pin1, LOW);  
  } else {
    // Mantener la velocidad actual
    analogWrite(motor_pin1, 255);    
  } 
  digitalWrite(motor_pin2, LOW); 
}


void loop() {    
  
  tiempoInterrupcionActual= millis();
  muestreoActual=millis(); //tiempo actual
  deltaMuestreo = (double) muestreoActual - muestreoAnterior; //Diferencia de tiempo (delta de tiempo de muestro)
  //deltaCambioGiro = (double) muestreoActual - cambioGiroTAnterior;    

  if(deltaMuestreo >= 15) //Si la diferencia es mayor o igual a un milisegundo
  {
    encoder_1();
    
    float vueltas = contador/resolucion;
    float gradosGirados = vueltas*360; // En grados       

    float velocidad_ang;
    float pos_ang;
    int a = digitalRead(ENCODER_A);
    int b = digitalRead(ENCODER_B);    
    
    deltaTiempoInterrupcion = tiempoInterrupcionActual -tiempoInterrupcionAnterior;

    rpm = 60 * vueltas * (1000/20);//frecuencia;  

    velocidad_ang = 2*PI*rpm/60;
    pos_ang = PI*posicionGrados/180;    

    // Pulsador hacia adelante
    if(digitalRead(BUTTON_FORWARD)){
      controlOnOff(velocidad_ang);
      /*analogWrite(motor_pin1, 255);
      analogWrite(motor_pin2, 0);    */
    }  
    else{
      analogWrite(motor_pin1, 0);
      analogWrite(motor_pin2, 0);    
    }      

    Serial.print(velocidad_ang);    
    Serial.print(",");
    Serial.println(error);
    //Serial.print(",");
    //Serial.println(tiempoInterrupcionActual);
    //Serial.println(inputAnalog());
    //Serial.println(pos_ang);
    //Serial.println(velocidad_ang);    
    //Serial.print(",");
    //Serial.println(pos_ang);
    /*Serial.print(",");
    Serial.println(muestreoActual);        
    Serial.print(",");
    Serial.print(a*5);
    Serial.print(",");
    Serial.println(b*5);*/
    
    muestreoAnterior = muestreoActual;
    contador = 0;
    encoder.write(0);
  }

}

void imprimir_cuadratura(){
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  Serial.print(a*5);
  Serial.print(" ");
  Serial.println(b*5);
}
