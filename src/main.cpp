#include <Arduino.h>
/* Seguidor de linea, usa un DRV8833 */

/*
D0 = sleepPin (EEP)
D1 = IN4
D2 = IN3
D3 = IN2
D4 = IN1
D8 = DIAGPING //Este es un pulldown, low es error

//Y para los sensores
D5 = S1 //Derecha
D6 = S2 //Medio
D7 = S3 //Izq
 */
#define SLEEP_PIN D0
#define IN4 D1 // adelante
#define IN3 D2 // reversa
#define IN2 D3 // reversa
#define IN1 D4 // adelante
#define DIAGPIN D8
#define S1 D5 // derecha
#define S2 D6 // medio
#define S3 D7 // izquierda

// put function declarations here:
const int pwmFrequency = 20000; // PWM frequency in Hz
const int pwmResolution = 10;   // PWM resolution (8 bits)
void setup()
{
  // Start pin sleep as 1
  digitalWrite(D0, HIGH);

  // Set pines
  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(DIAGPIN, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  analogWriteRange(1023);
  analogWriteFreq(pwmFrequency);

  Serial.begin(9600);
}

int actualSpeedMotorA = 0;
int actualSpeedMotorB = 0;
// funcion moverAdelante activa ambos motores

void moverAdelante()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(IN1, actualSpeedMotorA);
  analogWrite(IN4, actualSpeedMotorB);
}
void girarIzquierda()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(IN1, actualSpeedMotorA);
  analogWrite(IN4, actualSpeedMotorB);
}
void girarDerecha()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  analogWrite(IN1, actualSpeedMotorA);
  analogWrite(IN4, actualSpeedMotorB);
}

void seguirLinea()
{
  int sensorIzquierdo = digitalRead(S3);
  int sensorCentral = digitalRead(S2);
  int sensorDerecho = digitalRead(S1);

  if (sensorIzquierdo == LOW && sensorCentral == LOW && sensorDerecho == LOW)
  {
    // Caso 1: Todos los sensores están en LOW
    actualSpeedMotorA = 1023;
    actualSpeedMotorB = 1023;
    moverAdelante();
  }
  else if (sensorIzquierdo == LOW && sensorCentral == LOW && sensorDerecho == HIGH)
  {
    // Caso 2: Sensor derecho está en HIGH, los demás en LOW
    actualSpeedMotorA = 800; // Ajusta la velocidad de los motores según sea necesario
    actualSpeedMotorB = 1023;
    girarIzquierda();
  }
  else if (sensorIzquierdo == LOW && sensorCentral == HIGH && sensorDerecho == LOW)
  {
    // Caso 3: Sensor central está en HIGH, los demás en LOW
    actualSpeedMotorA = 1023;
    actualSpeedMotorB = 1023;
    moverAdelante();
  }
  else if (sensorIzquierdo == LOW && sensorCentral == HIGH && sensorDerecho == HIGH)
  {
    // Caso 4: Sensor central y derecho están en HIGH, izquierdo en LOW
    actualSpeedMotorA = 1023;
    actualSpeedMotorB = 800; // Ajusta la velocidad de los motores según sea necesario
    girarDerecha();
  }
  else if (sensorIzquierdo == HIGH && sensorCentral == LOW && sensorDerecho == LOW)
  {
    // Caso 5: Sensor izquierdo está en HIGH, los demás en LOW
    actualSpeedMotorA = 800; // Ajusta la velocidad de los motores según sea necesario
    actualSpeedMotorB = 1023;
    girarIzquierda();
  }
  else if (sensorIzquierdo == HIGH && sensorCentral == LOW && sensorDerecho == HIGH)
  {
    // Caso 6: Sensor izquierdo y derecho están en HIGH, central en LOW
    actualSpeedMotorA = 1023;
    actualSpeedMotorB = 800; // Ajusta la velocidad de los motores según sea necesario
    girarDerecha();
  }
  else if (sensorIzquierdo == HIGH && sensorCentral == HIGH && sensorDerecho == LOW)
  {
    // Caso 7: Sensor izquierdo y central están en HIGH, derecho en LOW
    actualSpeedMotorA = 1023;
    actualSpeedMotorB = 1023;
    moverAdelante();
  }
  else if (sensorIzquierdo == HIGH && sensorCentral == HIGH && sensorDerecho == HIGH)
  {
    // Caso 8: Todos los sensores están en HIGH
    actualSpeedMotorA = 0; // Detener motores
    actualSpeedMotorB = 0;
    // Realizar alguna acción cuando se encuentra una intersección o límite
  }
  else
  {
    // Caso 9: Todos los sensores están en HIGH
    actualSpeedMotorA = 0; // Detener motores
    actualSpeedMotorB = 0;
    // Realizar alguna acción cuando se encuentra una intersección o límite
  }
}
void loop()
{
  seguirLinea();
}

// put function definitions here:
