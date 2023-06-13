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
const int pwmFrequency = 40000; // PWM frequency in Hz

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
  analogWriteRange(255);
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
void stop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW); //
  digitalWrite(IN3, LOW);
}

void seguirLinea()
{
  int sensorDerecho = digitalRead(S1);
  int sensorIzquierdo = digitalRead(S3);
  int sensorCentral = digitalRead(S2);
  Serial.print("Sensor derecho: ");
  Serial.print(sensorDerecho);
  Serial.print(" Sensor central: ");
  Serial.print(sensorCentral);
  Serial.print(" Sensor izquierdo: ");
  Serial.println(sensorIzquierdo);

  // Estado de los motores
  Serial.print("Motor A: ");
  Serial.print(actualSpeedMotorA);
  Serial.print(" Motor B: ");
  Serial.println(actualSpeedMotorB);

  if (sensorIzquierdo == HIGH && sensorCentral == HIGH && sensorDerecho == HIGH)
  {
    // Caso 1: Todos los sensores están en HIGH
    actualSpeedMotorA = 0;
    actualSpeedMotorB = 0;
    moverAdelante();
  }
  else if (sensorIzquierdo == HIGH && sensorCentral == HIGH && sensorDerecho == LOW)
  {
    // Caso 2: Sensor derecho está en LOW, los demás en HIGH
    actualSpeedMotorA = 0; // Ajusta la velocidad de los motores según sea necesario
    actualSpeedMotorB = 1023;
    girarIzquierda();
  }
  else if (sensorIzquierdo == HIGH && sensorCentral == LOW && sensorDerecho == HIGH)
  {
    // Caso 3: Sensor central está en LOW, los demás en HIGH
    actualSpeedMotorA = 1023;
    actualSpeedMotorB = 1023;
    moverAdelante();
  }
  else if (sensorIzquierdo == HIGH && sensorCentral == LOW && sensorDerecho == LOW)
  {
    // Caso 4: Sensor central y derecho están en LOW, izquierdo en HIGH
    actualSpeedMotorA = 1023;
    actualSpeedMotorB = 0; // Ajusta la velocidad de los motores según sea necesario
    girarDerecha();
  }
  else if (sensorIzquierdo == LOW && sensorCentral == HIGH && sensorDerecho == HIGH)
  {
    // Caso 5: Sensor izquierdo está en LOW, los demás en HIGH
    actualSpeedMotorA = 0; // Ajusta la velocidad de los motores según sea necesario
    actualSpeedMotorB = 1023;
    girarIzquierda();
  }
  else if (sensorIzquierdo == LOW && sensorCentral == HIGH && sensorDerecho == LOW)
  {
    // Caso 6: Sensor izquierdo y derecho están en LOW, central en HIGH
    actualSpeedMotorA = 1023;
    actualSpeedMotorB = 0; // Ajusta la velocidad de los motores según sea necesario
    girarDerecha();
  }
  else if (sensorIzquierdo == LOW && sensorCentral == LOW && sensorDerecho == HIGH)
  {
    // Caso 7: Sensor izquierdo y central están en LOW, derecho en HIGH
    actualSpeedMotorA = 1023;
    actualSpeedMotorB = 1023;
    moverAdelante();
  }
  else if (sensorIzquierdo == LOW && sensorCentral == LOW && sensorDerecho == LOW)
  {
    // Caso 8: Todos los sensores están en LOW
    actualSpeedMotorA = 0; // Detener motores
    actualSpeedMotorB = 0;
    stop();
    // Realizar alguna acción cuando se encuentra una intersección o límite
  }
  else
  {
    // Caso 9: Otro caso
    actualSpeedMotorA = 0; // Detener motores
    actualSpeedMotorB = 0;
  }
}
void loop()
{
  seguirLinea();
  delay(50);
}

// put function definitions here:
