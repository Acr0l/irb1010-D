/*
 * -----------------------------
 * IRB2001
 * Modulo: Control de Robots
 * Profesor: David Acuña
 * -----------------------------
 * Programa Base
 * -----------------------------
 */

//-----------------------------------
// Importar librerias
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
#include <QTRSensors.h>

// Definicion de PINs
#define encoder0PinA  19
#define encoder0PinB  18
#define encoder1PinA  20
#define encoder1PinB  21

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Variables tiempo
unsigned long time_ant = 0;
const int Period = 10000;   // 10 ms = 100Hz
const float dt = Period *0.000001f;   // Tiempo de muestreo
float voltage_m0 = 0.0;   // Voltaje que se aplica a motor 0
float voltage_m1 = 0.0;   // Voltaje que se aplica a motor 1

// Variables de los encoders y posicion
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
long newposition0;
long oldposition0 = 0;
long newposition1;
long oldposition1 = 0;
unsigned long newtime;
float vel0;   // Velocidad del motor 0 en RPM
float vel1;   // Velocidad del motor 1 en RPM

float baseVel = 0;

float Kpr = 0.4, Kir = 0.0001, Kdr = 0.;
float Kpl = 0.2, Kil = 0.0001, Kdl = 0.;
float Kpa = 0.9, Kia = 0.0001, Kda = 0.0;

float inputr, outputr, setpointr;
float inputl, outputl, setpointl;
float inputa, outputa, setpointa;

unsigned long time, timePrev;
float elapsedTime;
float errorR, lastErrorR, cumErrorR, rateErrorR =0;
float errorL, lastErrorL, cumErrorL, rateErrorL;
float errorA, lastErrorA, cumErrorA, rateErrorA;

float angle, dist;

char msgEnd = ';';
String instruccion;
bool newMsg = false;

float speed1 = 0;

float computePIDR(float inp, float target){
  time = millis();
  elapsedTime = .000001;

  errorR = target - inp;
  cumErrorR += errorR * elapsedTime;
  rateErrorR = (errorR - lastErrorR)/elapsedTime;
 
  float out = Kpr*errorR + Kir*cumErrorR + Kdr*rateErrorR;

  lastErrorR = errorR;
  timePrev = time;

  return out;
}

float computePIDL(float inp, float target){
  time = millis();
  elapsedTime = .000001;

  errorL = target - inp;
  cumErrorL += errorL * elapsedTime;
  rateErrorL = (errorL - lastErrorL)/elapsedTime;

  float out = Kpl*errorL + Kil*cumErrorL + Kdl*rateErrorL;

  lastErrorL = errorL;
  timePrev = time;

  return out;
}

float computePIDA(float inp){
  time = millis();
  elapsedTime = (float)(time - timePrev);

  errorA = setpointa - inp;
  cumErrorA += errorA * elapsedTime;
  rateErrorA = (errorA - lastErrorA)/elapsedTime;

  float out = Kpa*errorA + Kia*cumErrorA + Kda*rateErrorA;

  lastErrorA = errorA;
  timePrev = time;

  return out;
}

//-----------------------------------
// CONFIGURANDO INTERRUPCIONES
void doEncoder0A()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}

void doEncoder0B()
{
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos--;
  } else {
    encoder0Pos++;
  }
}

void doEncoder1A()
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }
}

void doEncoder1B()
{
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos--;
  } else {
    encoder1Pos++;
  }
}


//-----------------------------------
// CONFIGURACION
void setup()
{
  // Configuracion de MotorShield
  md.init();

  // Configuracion de encoders
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // Incluir una resistencia de pullup en la entrada
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // Incluir una resistencia de pullup en la entrada
  pinMode(encoder1PinA, INPUT);
  digitalWrite(encoder1PinA, HIGH);       // Incluir una resistencia de pullup en la entrada
  pinMode(encoder1PinB, INPUT);
  digitalWrite(encoder1PinB, HIGH);       // Incluir una resistencia de pullup en la entrada
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);  // encoder 0 PIN A
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);  // encoder 0 PIN B
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);  // encoder 1 PIN A
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);  // encoder 1 PIN B


  // Configuracion de Serial Port
  Serial.begin(115200);           // Inicializacion del puerto serial (Monitor Serial)
  Serial.println("start");

  //iniciar el puerto 3
  Serial3.begin(38400);
  Serial.println("start 3");

  inputr = 0;
  inputl = 0;
  inputa = 0;
  setpointr = 0;
  setpointl = 0;
  setpointa = 0;
}

//-----------------------------------
// LOOP PRINCIPAL
void loop() {
  if ((micros() - time_ant) >= Period)
  {
    newtime = micros();
    //-----------------------------------
    // Actualizando informacion de los encoders
    newposition0 = encoder0Pos;
    newposition1 = encoder1Pos;

    //-----------------------------------
    // Calculando velocidad del motor en unidades de RPM
    float rpm = 31250;
    vel0 = (float)(newposition0 - oldposition0) * rpm / (newtime - time_ant); //RPM
    vel1 = (float)(newposition1 - oldposition1) * rpm / (newtime - time_ant); //RPM
    oldposition0 = newposition0;
    oldposition1 = newposition1;
    //-----------------------------------
    if (Serial3.available() > 0) {
      instruccion = readBuff(); //Leer el mensaje entrante
      int iSeparador = instruccion.indexOf('D');
      angle = (instruccion.substring(1, iSeparador)).toFloat();
      dist = (instruccion.substring(iSeparador + 1)).toFloat();
      Serial.println(angle);
      Serial.println(dist);
    }
    // Voltaje aplicado a motores (modificar aquí para implementar control)}
    setpointa = 0; 
    outputa = computePIDA(angle);

    inputr = dist + outputa;
    setpointr = 0; //baseVel + outputa; // Cambiar a VEL + outputa
    outputr = computePIDR(inputr, setpointr);
    
    inputl = -dist + outputa;
    setpointl = 0; //(baseVel + outputa); // Cambiar a VEL + outputa
    outputl = computePIDL(inputl, setpointl);

    voltage_m0 = outputl;
    voltage_m1 = outputr;
    
    //Seguridad: Establece voltaje máximo para motores
    if (voltage_m0 > 12.0){
      voltage_m0 = 12.0;
    }
    else if (voltage_m0 < -12.0){
      voltage_m0 = -12.0;
    }

    if (voltage_m1 > 12.0){
      voltage_m1 = 12.0;
    }
    else if (voltage_m1 < -12.0){
      voltage_m1 = -12.0;
    }

    // Motor Voltage
    md.setM1Speed(voltage_m0*400.0/12.0);
    md.setM2Speed(voltage_m1*400.0/12.0);

    // Serial.print(vel0);
    // Serial.print(",");
    // Serial.print(vel1);
    // Serial.print(",");
    // Serial.print(voltage_m0);
    // Serial.print(",");
    // Serial.println(voltage_m1);
    

    time_ant = newtime;
  }

}

String readBuff() {
  String buffArray;
  //int i = 0;

  while (Serial3.available() > 0) { //Entro a este while mientras exista algo en el puerto serial
    char buff = Serial3.read(); //Leo el byte entrante
    if (buff == msgEnd) {
      newMsg = true;
      break; //Si el byte entrante coincide con mi delimitador, me salgo del while
    } else {
      buffArray += buff; //Si no, agrego el byte a mi string para construir el mensaje
      //i += 1;
    }
    delay(10);
  }

  return buffArray;  //Retorno el mensaje
}


