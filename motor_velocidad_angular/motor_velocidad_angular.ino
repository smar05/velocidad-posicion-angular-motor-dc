#include <Math.h>

#define ENCODER_A       18 // Pin A del encoder
#define ENCODER_B       19 // Pin B del encoder
#define BUTTON_FORWARD  5 // Giro a la derecha
#define BUTTON_BACKWARD 4 // Giro a la izquierda
const int motor_pin1 = 7;  // Pin de control del motor H-bridge (1A)
const int motor_pin2 = 6; // Pin de control del motor H-bridge (2A)

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

void setup() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_A),Encoder,FALLING);
  
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


void Encoder ()
{
  contador++;

  //if (contador == resolucion) {
    //contador = 0;
    frecuencia = (1*1000)/ (double) deltaTiempoInterrupcion; //frecuencia de las interrupciones
  
    tiempoInterrupcionAnterior= tiempoInterrupcionActual;   
  //}  
}


void loop() {  

  if (cambiosGiro > 4) {
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin2, LOW);    
    return;
  }

  /*// Pulsador hacia adelante
  if(digitalRead(BUTTON_FORWARD)){
    digitalWrite(motor_pin1, HIGH);
    digitalWrite(motor_pin2, LOW);    
        
  }
  else if(digitalRead(BUTTON_BACKWARD)){
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin2, HIGH);    
    
  }
  else{
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin2, LOW);    
  }*/
  //imprimir_cuadratura();
  
  tiempoInterrupcionActual= millis();
  muestreoActual=millis(); //tiempo actual
  deltaMuestreo = (double) muestreoActual - muestreoAnterior; //Diferencia de tiempo (delta de tiempo de muestro)
  deltaCambioGiro = (double) muestreoActual - cambioGiroTAnterior;  

  if (deltaCambioGiro >= 5000) {    

    if (countPV == 0){
      countPV = 1;    
      pausaAnterior = muestreoActual;      
    }
    
    deltaPausa = (double) muestreoActual - pausaAnterior;    

    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin2, LOW);    

    if (deltaPausa >= 500) {
      giro = !giro;           
  
      // Pulsador hacia adelante
      if(giro){
        sentidoGiro = 0;
        digitalWrite(motor_pin1, HIGH);
        digitalWrite(motor_pin2, LOW);    
            
      }
      else if(!giro){
        sentidoGiro = 1;
        digitalWrite(motor_pin1, LOW);
        digitalWrite(motor_pin2, HIGH);    
        
      }        

      cambioGiroTAnterior = muestreoActual;
      pausaAnterior = muestreoActual;      
      countPV = 0;
      cambiosGiro ++;
    }      
  }

  if(deltaMuestreo >=15) //Si la diferencia es mayor o igual a un milisegundo
  {
    float vueltas = contador/resolucion;
    float gradosGirados = vueltas*360; // En grados

    // Izquierda
    if(sentidoGiro == 0){
        if(posicionGrados - gradosGirados <= 0) {
          posicionGrados = 360 - (gradosGirados - posicionGrados);
        } else {
          posicionGrados -= gradosGirados;            
        }        
    }
    // Derecha
    else if(sentidoGiro == 1){
      if (posicionGrados + gradosGirados >= 360) {
        posicionGrados =(posicionGrados + gradosGirados - 360);
      } else {
        posicionGrados += gradosGirados;
      }      
    }      

    float velocidad_ang;
    float pos_ang;
    int a = digitalRead(ENCODER_A);
    int b = digitalRead(ENCODER_B);    
    
    deltaTiempoInterrupcion = tiempoInterrupcionActual -tiempoInterrupcionAnterior;

    rpm = frecuencia*60*vueltas;        

    velocidad_ang = 2*PI*rpm/60;
    pos_ang = PI*posicionGrados/180;

    //Serial.println(velocidad_ang);    
    //Serial.print(",");
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
  }

}

void imprimir_cuadratura(){
  int a = digitalRead(ENCODER_A);
  int b = digitalRead(ENCODER_B);
  Serial.print(a*5);
  Serial.print(" ");
  Serial.println(b*5);
}
