# include "Protection.h"

Protection::Protection (short buzzerPin, short ledLowPin, short ledHighPin, hd44780_I2Cexp * lcd) {

  buzzerPin = buzzerPin;
  ledLowPin = ledLowPin;
  ledHighPin = ledHighPin;
  lcd = lcd;

  // timer1.initialize (ledBlinkInterval);
  // timer1.attachInterrupt (BlinkLED);
  // timer1.noInterrupts ();
}

void Protection::CheckCondition (int temp, int humidity) {
  CheckTempCondition (temp);
  CheckHumidityCondition (humidity);
}

short Protection::CheckTempCondition (int temp) {
  
  if (temp >= tempHighHigh) {
    return STATE_TEMP_HIGH_HIGH;
  } else if (temp >= tempHigh) {
    return STATE_TEMP_HIGH;
  } else if (temp <= tempLowLow) {
    return STATE_TEMP_LOW_LOW;
  } else if (temp <= tempLow) {
    return STATE_TEMP_LOW;
  }

  // timer1.noInterrupts ();
  return STATE_TEMP_NORMAL;
}

short Protection::CheckHumidityCondition (int humidity) {

  if (humidity >= humidityHighHigh) {
    return STATE_HUMIDITY_HIGH_HIGH;
  } else if (humidity >= humidityHigh) {
    return STATE_HUMIDITY_HIGH;
  } else if (humidity <= humidityLowLow) {
    return STATE_HUMIDITY_LOW_LOW;
  } else if (humidity <= humidityLow) {
    return STATE_HUMIDITY_LOW;
  }

  // timer1.noInterrupts ();
  return STATE_HUMIDITY_NORMAL;
}

void Protection::SendTempAlertLow (short state) {
  TriggerBuzzerLow ();
  BlinkLED (ledHighPin);

  switch (state) {
  case STATE_TEMP_HIGH:
    DisplayLCDAlert ("TEMP HIGH");
    break;
  case STATE_TEMP_LOW:
    DisplayLCDAlert ("TEMP LOW");
    break;
  case STATE_HUMIDITY_HIGH:
    DisplayLCDAlert ("HUMIDITY HIGH");
    break;
  case STATE_HUMIDITY_LOW:
    DisplayLCDAlert ("HUMIDITY LOW");
    break;

  default:
    break;
  }

}

void Protection::SendTempAlertHigh (short state) {
  TriggerBuzzerHigh ();
  BlinkLED (ledHighPin);

  switch (state) {

  case STATE_TEMP_HIGH_HIGH:
    DisplayLCDAlert ("TEMP TOO HIGH");
    break;
  case STATE_TEMP_LOW_LOW:
    DisplayLCDAlert ("TEMP TOO LOW");
    break;
  case STATE_HUMIDITY_HIGH_HIGH:
    DisplayLCDAlert ("HUMIDITY TOO HIGH");
    break;
  case STATE_HUMIDITY_LOW_LOW:
    DisplayLCDAlert ("HUMIDITY TOO LOW");
    break;

  default:
    break;
  }

}

void Protection::TriggerBuzzerLow () {
  digitalWrite (buzzerPin, HIGH);
}

void Protection::TriggerBuzzerHigh () {
  digitalWrite (buzzerPin, HIGH);
}

void Protection::TriggerLEDLow () {
  // timer1.interrupts ();
  digitalWrite (ledLowPin, HIGH);
}

void Protection::TriggerLEDHigh () {
  // timer1.interrupts ();
  digitalWrite (ledHighPin, HIGH);
}

void Protection::BlinkLED (short ledPin) {
  digitalWrite (ledPin, !digitalRead (ledPin));
}

void Protection::DisplayLCDAlert (String str) {
  lcd.clear ();
  lcd.setCursor (0, 0);
  lcd.print (str);
}
