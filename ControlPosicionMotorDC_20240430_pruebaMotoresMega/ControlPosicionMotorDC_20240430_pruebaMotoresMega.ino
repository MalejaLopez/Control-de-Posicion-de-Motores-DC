#include <PID_v1.h>  // Librería PID
#include <PWM.h>

#define ppr 64  //Pulses per revolution per one chanel
#define gr 50   //Gear ratio

#define _PP(a) Serial.print(a);
#define _PL(a) Serial.println(a);

unsigned long time = 0;

// definicion e inicialización de variables a utilizar
const int encoderA = 19;
const int encoderB = 18;
const int encoderA2 = 21;
const int encoderB2 = 20;

volatile long contador = 0;
volatile long contador2 = 0;// En esta variable se guardará los pulsos del encoder y que interpreremos como ángulo

int IN1 = 5;
int IN2 = 4;
int IN3 = 7;
int IN4 = 6;
boolean sensor1=0;
boolean sensor2=0;

// ************************************************ Variables PID *****************************************************************
double Setpoint = 0.0, Input = 0.0, Output = 0.0;  // Setpoint=Posición designada; Input=Posición del motor; Output=Tensión de salida para el motor.
double Setpoint2 = 0.0, Input2 = 0.0, Output2 = 0.0;
double kp = 0.0, ki = 0.0, kd = 0.0;               // Constante proporcional, integral y derivativa.
double outMax = 0.0, outMin = 0.0;                 // Límites para no sobrepasar la resolución del PWM.
double Grados = 0.0, Grados1 = 0.0, Grados2 = 0.0, Respuesta = 0.0, Respuesta2 = 0.0;
int Send=0;

// **************************************************** Otras Variables ***********************************************************
byte ant = 0, act = 0, ant2 = 0, act2 = 0, bandera = 1;  // Sólo se utiliza los dos primeros bits de estas variables y servirán para decodificar el encoder. (ant=anterior, act=actual.)
boolean antt2=0, ant1=0;
byte cmd = 0;           // Un byte que utilizamos para la comunicación serie. (cmd=comando.)
unsigned int tmp = 0;   // Variable que utilizaremos para poner el tiempo de muestreo.


PID myPID(&Input, &Output, &Setpoint, 0.0, 0.0, 0.0, DIRECT);  // Parámetros y configuración para invocar la librería.
PID myPID2(&Input2, &Output2, &Setpoint2, 0.0, 0.0, 0.0, DIRECT);

void setup() {

  Serial.begin(9600);
  InitTimersSafe();
  
  PCICR = B00000100;
  DDRD = B00000000;
  PORTD = B00000000;

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(encoderA2, INPUT);
  pinMode(encoderB2, INPUT);
 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(8, INPUT);//sensor 1
  pinMode(13, INPUT); // sensor 2

  // Configuracion de la posicion inicial 
  sensor1 = digitalRead(8); 
  sensor2 = digitalRead(13); 
  if (sensor1 == 1 && sensor2 == 2)
  {
    Serial.println("Ambos motores estan en home");
  }
  else{ 
    Serial.println("Los motores no se encuentran en home");
    Home();
  }
 
  //generación de interrupciones para contar los flancos de subida
  attachInterrupt(digitalPinToInterrupt(encoderA), encoder, CHANGE); //M1
  attachInterrupt(digitalPinToInterrupt(encoderB), encoder, CHANGE); //M1
  attachInterrupt(digitalPinToInterrupt(encoderA2), encoder2, CHANGE); //M2
  attachInterrupt(digitalPinToInterrupt(encoderB2), encoder2, CHANGE); //M2

  outMax = 50.0;    // Límite máximo del PWM.
  outMin = -outMax;  // Límite mínimo del PWM.

  tmp = 10;  // Tiempo de muestreo en milisegundos

  kp = 0.3603;  //0.3603tambien funciona/ Constantes PID iniciales. 
  ki = 1;  
  kd = 0;

  myPID.SetSampleTime(tmp);               // Envía a la librería el tiempo de muestreo.
  myPID.SetOutputLimits(outMin, outMax);  // Límites máximo y mínimo; corresponde a Max.: 0=0V hasta 255=5V (PWMA), y Min.: 0=0V hasta -255=5V (PWMB). Ambos PWM se convertirán a la salida en valores absolutos, nunca negativos.
  myPID.SetTunings(kp, ki, kd);           // Constantes de sintonización.
  myPID.SetMode(AUTOMATIC);               // Habilita el control PID (por defecto).
  Setpoint = (double)contador;            // Para evitar que haga cosas extrañas al inciarse, igualamos los dos valores para que comience estando el motor parado.

  myPID2.SetSampleTime(tmp);               // Envía a la librería el tiempo de muestreo.
  myPID2.SetOutputLimits(outMin, outMax);  // Límites máximo y mínimo; corresponde a Max.: 0=0V hasta 255=5V (PWMA), y Min.: 0=0V hasta -255=5V (PWMB). Ambos PWM se convertirán a la salida en valores absolutos, nunca negativos.
  myPID2.SetTunings(kp, ki, kd);           // Constantes de sintonización.
  myPID2.SetMode(AUTOMATIC);               // Habilita el control PID (por defecto).
  Setpoint2 = (double)contador2;
}

void loop() {
  // Recepción de datos para posicionar el motor, o modificar las constantes PID, o el tiempo de muestreo. Admite posiciones relativas y absolutas.
  while (Serial.available() > 0) {
    time = millis();
    cmd = 0;  // Por seguridad "limpiamos" cmd.
    cmd = Serial.read(); // "cmd" guarda el byte recibido.

    if (cmd > 31) {
      if (cmd > 'Z') {
        cmd -= 32;
      }
    // Decodificador para modificar las constantes PID.
      switch(cmd)                                                                 // Si ponemos en el terminal serie, por ejemplo "p2.5 i0.5 d40" y pulsas enter  tomará esos valores y los cargará en kp, ki y kd.
      {                                                                           // También se puede poner individualmente, por ejemplo "p5.5", sólo cambiará el parámetro kp, los mismo si son de dos en dos.
        case 'P': kp  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); myPID2.SetTunings(kp, ki, kd); break; 
        case 'I': ki  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); myPID2.SetTunings(kp, ki, kd); Serial.print(ki); break;
        case 'D': kd  = Serial.parseFloat(); myPID.SetTunings(kp, ki, kd); myPID2.SetTunings(kp, ki, kd); break;
        case 'T': tmp = Serial.parseInt();   myPID.SetSampleTime(tmp);     myPID2.SetSampleTime(tmp);     break;
        case 'G': Grados = Serial.parseFloat();  break;
        case 'S': Send =  1; break;
      }
    }
  }

  if (Grados > 0 && Grados > 90){
    Grados1 = Grados;
    Grados2 = Grados - 70;
    Movimiento(Grados1,Grados2);
  }

  if (Grados < 0 && Grados < -90){
    Grados2 = Grados;
    Grados1 = Grados + 70;
    Movimiento(Grados1,Grados2);
  }
  if (Grados > 0 && Grados <= 90){
    Grados1 = Grados;
    Grados2 = Grados - 30;
    Movimiento(Grados1,Grados2);
  } 
  if (Grados < 0 && Grados >= -90){
    Grados2 = Grados;
    Grados1 = Grados + 30;
    Movimiento(Grados1,Grados2);
  }
  if (Grados == 0){
    Grados2 = Grados;
    Grados1 = Grados;
    Movimiento(Grados1,Grados2);
  } 
}
 
//Cuando se produzca cualquier cambio en el encoder esta parte hará que incremente o decremente el contador.
void encoder() { 
  ant = act;                             // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.
  act = PIND & 12;                       // Guardamos en 'act' el valor que hay en ese instante en el encoder y hacemos un
                                      // enmascaramiento para aislar los dos únicos bits que utilizamos para esta finalidad.
    if(ant==12 && act==4)  contador++;// Incrementa el contador si el encoder se mueve hacia delante.
    if(ant==4  && act==0)  contador++;
    if(ant==0  && act==8)  contador++;
    if(ant==8  && act==12) contador++;
    
    if(ant==4  && act==12) contador--;// Decrementa el contador si el encoder se mueve hacia atrás.
    if(ant==0  && act==4)  contador--;
    if(ant==8  && act==0)  contador--;
    if(ant==12 && act==8)  contador--;  
     
    }

void encoder2() {
  ant2 = act2;                              // Guardamos el valor 'act' en 'ant' para convertirlo en pasado.
  act2 = PIND & 3;           // Guardamos en 'act' el valor que hay en ese instante en el encoder y hacemos un
                                          // enmascaramiento para aislar los dos únicos bits que utilizamos para esta finalidad.
    if(ant2==3 && act2==1)  contador2++;// Incrementa el contador si el encoder se mueve hacia delante.
    if(ant2==1  && act2==0)  contador2++;
    if(ant2==0  && act2==2)  contador2++;
    if(ant2==2  && act2==3) contador2++;
    
    if(ant2==1  && act2==3) contador2--;// Decrementa el contador si el encoder se mueve hacia atrás.
    if(ant2==0  && act2==1)  contador2--;
    if(ant2==2  && act2==0)  contador2--;
    if(ant2==3 && act2==2)  contador2--;       
}

// ##########################################################################################################

void Movimiento(double x, double y){

  //El valor de 8.888888889 sale entre la razón de 3200 pulsos y 360 grados, como una vuelta completa en el motor son 3200 pulsos del encoder,entonces en grados son 360
  //Y como el sistema funciona con base en los pulsos, por ejemplo, el calculo del error se realiza entre los pulsos requeridos por el setpoint menos los pulsos contados
  //Se hace la divición de 3200/360 que da igual a 8.888888889, entonces cuando se ingrese en el setpoint una posición en grados por ejemplo 180° este valor en el programa
  //se mulriplicará por 8.888888889 para que puede hacer el calculo del error en pulsos.


// *********************************************** Control del Motor 1 *************************************************
  
  Setpoint = (Grados1) * 8.888888889;     //Es para ingresar valores de 0 a 360 grados como setpoint, asi cuando se le ingrese un valor a la variable grados desde el serialploter, el valor introducido se multiplicara por 8.888888889 y hara la conversion de grados a pulsos que es lo que lee la variable contador para hacer el calculo del error.
  Respuesta = contador / 8.888888889;  //En este caso el contador que seria la respuestas en forma de pulsos, sera divida por 8.888888889, para que en la grafica de respuesta muestre grados en lugar de pulsos.

  Input = (double)contador;  // Lectura del encoder. El valor del contador se incrementa/decrementa a través de las interrupciones externas 
//  _PL(Respuesta);

  _PP("M1");_PP("__");_PP(Respuesta);_PP("  ");
  _PP("M2");_PP("__");_PP(Respuesta2); _PP("  "); _PP("Setpoint: ");_PL(Setpoint);

  while (!myPID.Compute());  // Mientras no se cumpla el tiempo de muestreo, se queda en este bucle.
  if (((long)Setpoint - contador) == 0)  // Cuando está en el punto designado, parar el motor.
  {
    digitalWrite(IN1, LOW); 
    digitalWrite(IN2, LOW); 
  }

  else  // En caso contrario hemos de ver si el motor ha de ir hacia delante o hacia atrás. Esto lo determina el signo de la variable "Output".
  {
    if (Output > 0.0)  // Mueve el motor hacia delante con el PWM correspondiente a su posición.
    {
      digitalWrite(IN2, LOW);    //M1 --> antihorario        
      digitalWrite(IN1, HIGH);    //M1 --> horario         
      pwmWrite(11, abs(Output)); //M1  
      
    } else  // Mueve el motor hacia  atrás con el PWM correspondiente a su posición.
    {
      digitalWrite(IN2, HIGH);          
      digitalWrite(IN1, LOW);    
      pwmWrite(11, abs(Output));  
    }
  }

  // *********************************************** Control del Motor 2 *************************************************

  Setpoint2 = (Grados2) * 8.888888889;     //Es para ingresar valores de 0 a 360 grados como setpoint, asi cuando se le ingrese un valor a la variable grados desde el serialploter, el valor introducido se multiplicara por 8.888888889 y hara la conversion de grados a pulsos que es lo que lee la variable contador para hacer el calculo del error.
  Respuesta2 = contador2 / 8.888888889;  //En este caso el contador que seria la respuestas en forma de pulsos, sera divida por 8.888888889, para que en la grafica de respuesta muestre grados en lugar de pulsos.

  Input2 = (double)contador2;  // Lectura del encoder. El valor del contador se incrementa/decrementa a través de las interrupciones externas (pines 2 y 3).
//  _PL(Respuesta2);
  
  while (!myPID2.Compute());  // Mientras no se cumpla el tiempo de muestreo, se queda en este bucle.
  if (((long)Setpoint2 - contador2) == 0)  // Cuando está en el punto designado, parar el motor.
  {
    digitalWrite(IN4, LOW); 
    digitalWrite(IN3, LOW); 
  }

  else  // En caso contrario hemos de ver si el motor ha de ir hacia delante o hacia atrás. Esto lo determina el signo de la variable "Output".
  {
    if (Output2 > 0.0)  // Mueve el motor hacia delante con el PWM correspondiente a su posición.
    {
      digitalWrite(IN4, LOW);    //M2 --> antihorario          
      digitalWrite(IN3, HIGH);    //M2 --> horario           
      pwmWrite(12, abs(Output2));  //M2

    } else  // Mueve el motor hacia  atrás con el PWM correspondiente a su posición.
    {
      digitalWrite(IN4, HIGH);    
      digitalWrite(IN3, LOW);      
      pwmWrite(12, abs(Output2));  
    }
  }
  
}
// ###########################################################################################################
void Home(){
  while((sensor1 == 0 && sensor2 == 0) || (sensor1 == 1 && sensor2 == 0) || (sensor1 == 0 && sensor2 == 1)){
    sensor1 = digitalRead(8);
    sensor2 = digitalRead(13);
    digitalWrite(IN1, LOW);          
    digitalWrite(IN2, HIGH);
    digitalWrite(IN4, LOW);          
    digitalWrite(IN3, HIGH);
    pwmWrite(12, abs(20));
    pwmWrite(11, abs(20));
    
    if (sensor1 == 1){
      digitalWrite(IN1, LOW);          
      digitalWrite(IN2, LOW);
      ant1 = 1;
      contador = 0;
      contador2 = 0;
      Serial.println("El motor 1 ha lleguado al Home");
    }

    if (sensor2 == 1){
      digitalWrite(IN3, LOW);          
      digitalWrite(IN4, LOW);
      antt2 = 1;     
      contador = 0;
      contador2 = 0;
      Serial.println("El motor 2 ha lleguado al Home");
    }
    if (ant1 == 1 && antt2 == 1){
      contador = 0;
      contador2 = 0;
      break;
    }
  }

  }
