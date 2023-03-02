// Definir los pines de entrada y salida necesarios para el control del motor y el encoder
const int motor_pin1 = 2;  // Pin de control del motor H-bridge (1A)
const int motor_pin2 = 3; // Pin de control del motor H-bridge (2A)
const int encoder_pinA = 18; // Pin A del encoder
const int encoder_pinB = 19; // Pin B del encoder

// Definir las variables necesarias para el control del motor y el encoder
float motor_speed = 0;       // Velocidad actual del motor (en rpm)
float motor_position = 0;  // Posición actual del motor (en rad)
int motor_direction = 0;   // Dirección actual del motor (-1 para giro en un sentido, 1 para giro en el otro)
int encoder_count = 0;     // Conteo actual de pulsos del encoder
int encoder_lastA = 0;     // Último valor leído del pin A del encoder
int encoder_lastB = 0;     // Último valor leído del pin B del encoder
long last_time = 0;        // Último momento en el que se realizó una medición de velocidad
int cantidad_cambios_giro = 0; // Cantidad de cambios de giro del motor
volatile int encoderPos = 0;
volatile int encoderLastPos = 0;
volatile unsigned long encoderLastTime = 0;

// Definir las constantes para la conversión de señal del encoder a velocidad angular y posición angular
const float countsPerRev = 48; // cuentas por revolución del encoder
const float degPerCount = 360.0 / countsPerRev; // grados por cuenta del encoder
const float radPerDeg = PI / 180.0; // radianes por grado
const float radPerCount = radPerDeg * degPerCount; // radianes por cuenta del encoder

void setup() {
  // Configurar los pines de entrada y salida
  pinMode(motor_pin1, OUTPUT);
  pinMode(motor_pin2, OUTPUT);
  pinMode(encoder_pinA, INPUT);
  pinMode(encoder_pinB, INPUT);
  // Establecer la velocidad y dirección inicial del motor
  motor_speed = 0;
  motor_position = 0;
  motor_direction = 1; // Giro en un sentido

  // Configurar las interrupciones del encoder
  attachInterrupt(digitalPinToInterrupt(encoder_pinA), encoderISR, CHANGE);

  Serial.begin(9600);
  
  // Escribir encabezado del archivo
  Serial.println("Tiempo (ms), Velocidad (rad/s), Posición (rad), Diferencia de potencial (V)");
}

void loop() {
  // No pasar mas giros del motor
  if (cantidad_cambios_giro >= 5 ) {
    return;  
  }
  
  // Leer las señales del encoder
  /*
  int encoder_valA = digitalRead(encoder_pinA);
  int encoder_valB = digitalRead(encoder_pinB);
  // Determinar la dirección del giro del motor a partir de las señales del encoder
  if (encoder_valA != encoder_lastA) {
    if (encoder_valB != encoder_valA) {
      encoder_count++;
    } else {
      encoder_count--;
    }
  }
  encoder_lastA = encoder_valA;
  encoder_lastB = encoder_valB;
  */
  // Calcular la velocidad y posición actual del motor
  long current_time = millis();
  long time_diff = current_time - last_time;
  if (time_diff >= 10) { // Medir la velocidad y posición cada 10 ms
    float deltaPos = encoderPos - encoderLastPos;
    motor_speed = deltaPos * radPerCount / time_diff * 1000; // velocidad angular en rad/s    
    motor_position = encoderPos * radPerCount; // posición angular en rad        
        
    // Escribir los datos en el puerto serial
    Serial.print(current_time);
    Serial.print(",");
    Serial.print(motor_speed);
    Serial.print(",");
    Serial.print(motor_position);
    Serial.print(",");
    if (motor_direction == 1) {
      Serial.println(12);
    } else if (motor_direction == -1) {
      Serial.println(-12);
    }  
    encoderLastPos = encoderPos;
    last_time = current_time;
  }  
  
  // Controlar el sentido de giro del motor
  if (motor_direction == 1) {
    digitalWrite(motor_pin1, HIGH);
    digitalWrite(motor_pin2, LOW);
  } else {
    digitalWrite(motor_pin1, LOW);
    digitalWrite(motor_pin2, HIGH);
  }
  // Cambiar la dirección del giro del motor después de 5 segundos
  if (millis() > 5000) {
    motor_direction *= -1; // Cambiar la dirección del giro
    cantidad_cambios_giro ++;
    delay(500); // Esperar un tiempo para evitar cambios bruscos en el motor
  }
}

void encoderISR() {
  // Leer la señal del encoder y actualizar la posición
  int encoderAPin = digitalRead(encoder_pinA);
  int encoderBPin = digitalRead(encoder_pinB);
  if (encoderAPin == encoderBPin) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}