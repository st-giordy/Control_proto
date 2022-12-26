#include "max6675.h"            // max6675.h file is part of the library that you should download from Robojax.com
#include <SoftwareSerial.h>     ///////////IDK
#include <Wire.h>               // this is for the liquid crystal support
#include <LiquidCrystal_I2C.h>  //I2C is the accesory which conenects directly the LCD
//Init de objetos
LiquidCrystal_I2C lcd(0x27, 20, 4);  //LCD Size for wrtiting, was thw one that was working
//SENSORES
//*********************************************Termocuplas****************************
int soPin_calefon = 8;      //11                                          // SO=Serial Out
int csPin_calefon = 9;         //13                                       // CS = chip select CS pin
int sckPin_calefon = 10;             //12                                  // SCK = Serial Clock pin
MAX6675 Termoc_calefon(sckPin_calefon, csPin_calefon, soPin_calefon);  // create instance object of MAX6675
float temp_calefon;
int soPin = 11;                         //9     // SO=Serial Out
int csPin = 12;                         //10     // CS = chip select CS pin
int sckPin = 13;                       //8     // SCK = Serial Clock pin
MAX6675 Termoc_tank(sckPin, csPin, soPin);  // create instance object of MAX6675
float temp_tank;
//*********************************************Flujometro****************************
//int sensorPin = 3;    // Sensor Input
volatile long pulse;  // Measures flow sensor pulses
unsigned int caudal; // Calculated litres/minute
//***********
#define TIME_CICLE 1000 //1 second
#define RELE_VALVE 4
#define RELE_PUMP 6
#define RELE_PUMP2 7 
#define RELE_CALEFON 5  //rele de estado solido Calefon
#define SENSOR_PIN 2
//************
unsigned long currentTime;
unsigned long cloopTime;
unsigned long difference;
unsigned long five_Time;

void setup() {
  // PinMode Definition
  pinMode(RELE_VALVE, OUTPUT);
  pinMode(RELE_PUMP, OUTPUT);
  pinMode(RELE_PUMP2, OUTPUT);
  pinMode(RELE_CALEFON, OUTPUT);
  //**********
  pinMode(SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), increase, RISING);  //RISING to trigger when the pin goes from low to high
  //
  Serial.begin(9600);  // initialize serial monitor with 9600 baud
  lcd.init();          // initialize the lcd
  lcd.init(); 
  lcd.backlight();
  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("Tcal  Ttank  Q");
  
  ///
  currentTime = millis();// first setting of time -- este setteo 
  cloopTime = currentTime;
}

void loop() {
  currentTime = millis();
if(currentTime >= (cloopTime))
{
 cloopTime = currentTime; // Updates cloopTime
// Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
 caudal = (pulse/7.5); 
 pulse = 0; // Reset Counter
}
  difference=currentTime-cloopTime;
  five_Time+=difference;

  temp_calefon = Termoc_calefon.readCelsius();
  temp_tank = Termoc_tank.readCelsius();
  //Serial.println(currentTime);
  //**************Control VOIDs*********
  control_valvula_4_vias();
  control_motor_Temp();
  //, una vez prendido el motor, no se deberia apagar
  // if (RELE_PUMP2==LOW || RELE_PUMP==LOW && condicion de tiempo){ no ejecute el void
  control_calefon();
  control_motor_Caudal();
  //cloopTime = currentTime; // Updates cloopTime
  // fisrt, read then
  delay(1000);
  Serial.print("temp Calefon = ");
  Serial.println(temp_calefon);
    Serial.print("temp Tanque = ");
  Serial.println(temp_tank);
    Serial.print("Caudal = ");
  Serial.println(caudal);

  lcd.setCursor(0, 1);
  lcd.print(temp_calefon);
  lcd.setCursor(6, 1);
  lcd.print(temp_tank);
  lcd.setCursor(13, 1);
  lcd.print(caudal);
  //  lcd.print(digitalRead(RELE_PUMP));
  //  lcd.print(digitalRead(RELE_CALEFON));
}
void control_valvula_4_vias() {  
  //TemperaturaTank<=26.00 ------- Tambien se deberia considerar que el HVAC este encendido--
  //(currentTime-cloopTime) <= (TIME_CICLE*300)
  if (( temp_tank <= 30.00 ) ) {//para que no se prenda y apague se hace cada 5 min.
    digitalWrite(RELE_VALVE, LOW);  //encendido
  } else {
    digitalWrite(RELE_VALVE, HIGH);  //apagado
  }
}
void control_calefon() {
  //Flujo >= 1--detecte-- && Temperatura a la salida calefon <=42 && tiempo cada seg

  if (( temp_calefon <= 30.00 ) && ( caudal >= 1.00)  ) {
    digitalWrite(RELE_CALEFON, LOW);  //encendido
  } else {
    digitalWrite(RELE_CALEFON, HIGH);  //apagado
  }
}
void control_motor_Temp() {
  //TemperaturaF>=1
  if (( temp_tank <= 30.00 )) {
    digitalWrite(RELE_PUMP, LOW);  //encendido
  } else {
    digitalWrite(RELE_PUMP, HIGH);  //apagado
  }
}
void control_motor_Caudal() {
  //TemperaturaF>=1
  if (( caudal >= 1.00 ) ) {
    digitalWrite(RELE_PUMP2, LOW);  //encendido
  } else {
    digitalWrite(RELE_PUMP2, HIGH);  //apagado
  }
}
void increase() {  // Interrupt function
  pulse++;
}