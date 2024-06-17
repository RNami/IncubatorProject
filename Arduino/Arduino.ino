# include <Wire.h>
# include <DHT11.h>
# include <hd44780.h>
# include <hd44780ioClass/hd44780_I2Cexp.h>
# include "PID.h"
# include "Protection.h"

# define DHT11_PIN 2
# define HEATER_PIN 5
# define FAN_PIN 3
# define BUZZER_PIN 7
# define TEMP_SETPOINT_POT 9
# define LED_ALERT_LOW_PIN A1
# define LED_ALERT_HIGH_PIN A2

/* ////////////////////////////////
          Global Variables
*/ ////////////////////////////////

const int heaterMaximumLoad = 210;
const int fanMaximumLoad = 255;
const int heaterMinimumLoad = 0;
const int fanMinimumLoad = 50;

DHT11 dht11_sens (DHT11_PIN);
hd44780_I2Cexp lcd (0x27);
Protection protection (BUZZER_PIN, LED_ALERT_LOW_PIN, LED_ALERT_HIGH_PIN, & lcd);

double Setpoint, Input, Output;
double tempKp = 8, tempKi = 5, tempKd = 1; // temp PID tuning parameters
PID tempPID (& Input, & Output, & Setpoint, tempKp, tempKi, tempKd, DIRECT);

int heaterLoad;
int fanLoad;

int temp;
int humd;


/* ////////////////////////////////
          Prototypes
*/ ////////////////////////////////

void DisplayLCDSensor ();
void DisplayTempSetpoint ();
void DisplayControllerOutput ();
void InitialHeating ();
void HeatControl ();

/* ////////////////////////////////
          Setup Function
*/ ////////////////////////////////

void setup() {

  // Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("Serial Monitor OK");

  // Setup LCD Display
  lcd.begin (16, 2);
  lcd.print ("LCD OK");
  lcd.clear ();
  lcd.noBacklight();

  // Setup Pins Mode
  pinMode(FAN_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TEMP_SETPOINT_POT, INPUT_PULLUP);

  temp = dht11_sens.readTemperature ();

  // Setpoint = analogRead(TEMP_SETPOINT_POT);
  Setpoint = 38;
  Output = heaterMinimumLoad;
  tempPID.SetMode(AUTOMATIC);  
  tempPID.SetOutputLimits (heaterMinimumLoad, heaterMaximumLoad);

  InitialHeating ();
}


/* ////////////////////////////////
          Loop Function
*/ ////////////////////////////////

void loop() {

  Serial.println();

  temp = dht11_sens.readTemperature ();
  humd = dht11_sens.readHumidity ();

  // Setpoint = analogRead(TEMP_SETPOINT_POT) / 20.9761;
  HeatControl ();


  if (temp == dht11_sens.ERROR_TIMEOUT || humd == dht11_sens.ERROR_TIMEOUT) {
    Serial.println("Failed to read from DHT sensor!(Timeout)");
  } else {
    Serial.print(temp);
    Serial.println();
    Serial.print (humd);
    Serial.println();

    DisplayLCDSensor ();
  }

  DisplayTempSetpoint ();
  DisplayControllerOutput();

  delay (3000);
}

void DisplayLCDSensor () {

    lcd.setCursor (0, 0);
    lcd.print ("T:");
    lcd.setCursor (2, 0);
    lcd.print (temp);
    lcd.setCursor(0, 1);
    lcd.print ("H:");
    lcd.setCursor(2, 1);
    lcd.print (humd);
}

void DisplayTempSetpoint () {

  lcd.setCursor(6, 0);
  lcd.print ("TSP:");
  lcd.setCursor(11, 0);
  lcd.print (Setpoint);
}

void DisplayControllerOutput () {
  lcd.setCursor(6, 1);
  lcd.print ("CO:");
  lcd.setCursor(11, 1);
  lcd.print (Output);
}

void InitialHeating () {

  analogWrite(HEATER_PIN, heaterMaximumLoad);
  analogWrite(FAN_PIN, fanMaximumLoad);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print ("Preheat... to 38 C°");
  lcd.setCursor(0, 1);
  lcd.print ("Temp:");

  while ((Setpoint - temp) >= 1) {

    lcd.setCursor(5, 1);
    lcd.print (temp);

    temp = dht11_sens.readTemperature ();
    delay (3000);
  }
}

void HeatControl () {

  Input = temp;
  tempPID.Compute();

  heaterLoad = Output;
  fanLoad = (Output >= fanMinimumLoad ? Output : fanMinimumLoad);

  analogWrite(FAN_PIN, fanLoad);
  if (tempPID.GetDirection() == DIRECT) {
    analogWrite(HEATER_PIN, heaterLoad);
  } else if (tempPID.GetDirection() == REVERSE) {
    analogWrite(HEATER_PIN, heaterMinimumLoad);
  }

}
