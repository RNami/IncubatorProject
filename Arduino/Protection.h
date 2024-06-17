# ifndef Protection_h
# define Protection_h

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

# include <Wire.h>
# include <TimerOne.h>
# include <hd44780.h>
# include <hd44780ioClass/hd44780_I2Cexp.h>

class Protection {

  public:

    # define STATE_TEMP_NORMAL        0
    # define STATE_TEMP_HIGH          1
    # define STATE_TEMP_HIGH_HIGH     2
    # define STATE_TEMP_LOW           3
    # define STATE_TEMP_LOW_LOW       4
    # define STATE_HUMIDITY_NORMAL    5
    # define STATE_HUMIDITY_HIGH      6
    # define STATE_HUMIDITY_HIGH_HIGH 7
    # define STATE_HUMIDITY_LOW       8
    # define STATE_HUMIDITY_LOW_LOW   9

    Protection (short, short, short, hd44780_I2Cexp *);

    void CheckCondition (int, int);
    short CheckTempCondition (int);
    short CheckHumidityCondition (int);
    
    void SendTempAlertLow (short);
    void SendTempAlertHigh (short);

    void TriggerBuzzerLow ();
    void TriggerBuzzerHigh ();

    void TriggerLEDLow ();
    void TriggerLEDHigh ();

    void DisplayLCDAlert (String);

    void BlinkLED (short);

    int GettempHighHigh () { return tempHighHigh; }
    int GettempHigh () { return tempHigh; }
    int GettempLowLow () { return tempLowLow; }
    int GettempLow () { return tempLow; }

    void SetBuzzerPin (short buzzer_pin) { buzzerPin = buzzer_pin; }
    void SetLEDLowPin (short led_low_pin) { ledLowPin = led_low_pin; }
    void SetLEDHighPin (short led_high_pin) { ledHighPin = led_high_pin; }

  private:

    const int tempHighHigh = 41;
    const int tempHigh = 40;
    const int tempLow = 36;
    const int tempLowLow = 35;

    const int humidityHighHigh = 90;
    const int humidityHigh = 80;
    const int humidityLow = 45;
    const int humidityLowLow = 35;

    short buzzerPin;
    short ledLowPin;
    short ledHighPin;      

    const int ledBlinkInterval = 2000;

    TimerOne timer1;
    hd44780_I2Cexp lcd;
};


# endif