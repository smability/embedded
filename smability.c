#include "Arduino.h"
// avr - Version: Latest 
#include <power.h>
#include <sleep.h>
#include <wdt.h>
// serial library
#include <SoftwareSerial.h>

// Power SIM
#define POWERSIM (11)
// Power Switch
#define POWERSW (12)
// Power Sensor
#define POWER (13)
// Analog Port
#define LEVEL_SENSOR A3

float level;
float voltage;

//const unsigned long POSTinterval = 1000UL*60; //one minute interval
//unsigned long POSTtimer;

// This variable is made volatile because it is changed inside
// an interrupt function
volatile int f_wdt=1;
int count;
// Watchdog Interrupt Service. This is executed when watchdog timed out.
ISR(WDT_vect) {
	if(f_wdt == 0) {
		// here we can implement a counter the can set the f_wdt to true if
		// the watchdog cycle needs to run longer than the maximum of eight
		// seconds.
		//int count; otherwise, we are initializing this variable, to 0, every time ISR is called
		//Serial.println(count);
		if(count == 3){
		  f_wdt=1;
		  count=0;
		}
		else
		  count++;
	}
}

// Enters the arduino into sleep mode.
void enterSleep(void)
{
	// There are five different sleep modes in order of power saving:
	// SLEEP_MODE_IDLE - the lowest power saving mode
	// SLEEP_MODE_ADC
	// SLEEP_MODE_PWR_SAVE
	// SLEEP_MODE_STANDBY
	// SLEEP_MODE_PWR_DOWN - the highest power saving mode
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	// Now enter sleep mode.
	sleep_mode();
	// The program will continue from here after the WDT timeout
	// First thing to do is disable sleep.
	sleep_disable();
	// Re-enable the peripherals.
	power_all_enable();
}

// Setup the Watch Dog Timer (WDT)
void setupWatchDogTimer() {
  
	// The MCU Status Register (MCUSR) is used to tell the cause of the last
	// reset, such as brown-out reset, watchdog reset, etc.
	// NOTE: for security reasons, there is a timed sequence for clearing the
	// WDE and changing the time-out configuration. If you don't use this
	// sequence properly, you'll get unexpected results.

	// Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
	MCUSR &= ~(1<<WDRF);

	// Configure the Watchdog timer Control Register (WDTCSR)
	// The WDTCSR is used for configuring the time-out, mode of operation, etc

	// In order to change WDE or the pre-scaler, we need to set WDCE (This will
	// allow updates for 4 clock cycles).

	// Set the WDCE bit (bit 4) and the WDE bit (bit 3) of the WDTCSR. The WDCE
	// bit must be set in order to change WDE or the watchdog pre-scalers.
	// Setting the WDCE bit will allow updates to the pre-scalers and WDE for 4
	// clock cycles then it will be reset by hardware.
	WDTCSR |= (1<<WDCE) | (1<<WDE);

	/**
	 *	Setting the watchdog pre-scaler value with VCC = 5.0V and 16mHZ
	 *	WDP3 WDP2 WDP1 WDP0 | Number of WDT | Typical Time-out at Oscillator Cycles
	 *	0    0    0    0    |   2K cycles   | 16 ms
	 *	0    0    0    1    |   4K cycles   | 32 ms
	 *	0    0    1    0    |   8K cycles   | 64 ms
	 *	0    0    1    1    |  16K cycles   | 0.125 s
	 *	0    1    0    0    |  32K cycles   | 0.25 s
	 *	0    1    0    1    |  64K cycles   | 0.5 s
	 *	0    1    1    0    |  128K cycles  | 1.0 s
	 *	0    1    1    1    |  256K cycles  | 2.0 s
	 *	1    0    0    0    |  512K cycles  | 4.0 s
	 *	1    0    0    1    | 1024K cycles  | 8.0 s
	*/
	WDTCSR  = (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
	// Enable the WD interrupt (note: no reset).
	WDTCSR |= _BV(WDIE);
}

void setup() {
    Serial.begin(9600);
	  // Set PIN modes
	  pins_init();
	  Serial.println("Initialising..."); 
	  delay(100);
	  setupWatchDogTimer();
    Serial.println("Initialisation complete."); 
	  delay(100);
}

void pins_init()
{
  pinMode(LEVEL_SENSOR, INPUT);
  pinMode(POWER, OUTPUT);
  pinMode(POWERSIM, OUTPUT);
  pinMode(POWERSW, OUTPUT);
  
}
// Get raw sensor value
int getMaxValue()
  {
    int sensorValue;

    int sensorMax = 0;
  
    uint32_t start_time = millis();

    while ((millis()-start_time)<100) // sensor reading gets attenuated if sample <= 10ms, WHY?
    {
      sensorValue = analogRead(LEVEL_SENSOR);

      if(sensorValue > sensorMax)
      {
        sensorMax = sensorValue;
      }
    }
    return sensorMax;
  }
// Power ON hall sensor
void powerONSensor()
  {
  digitalWrite (POWER, LOW);  // turn power ON
  delay(1); // give time to power up    
  }  // end of powerONSensor

// Power OFF hall sensor  
void powerOFFSensor()
  {
  digitalWrite (POWER, HIGH);  // turn power OFF
  delay(1); // give time to power down    
  }  // end of powerOFFSensor
  
// Power ON Boost
void powerONBoost()
  {
  digitalWrite (POWERSW, LOW);  // turn power ON
  delay(1); // give time to power up    
  }  // end of powerON SIM

// Power OFF Boost
void powerOFFBoost()
  {
  digitalWrite (POWERSW, HIGH);  // turn power OFF
  delay(1); // give time to power down    
  }  // end of powerOFF SIM

// Power ON SIM
void powerONSIM()
  {
  digitalWrite (POWERSIM, HIGH);  // turn transistor ON
  delay(1); // give time to power up    
  }  // end of powerON SIM

// Power OFF SIM
void powerOFFSIM()
  {
  digitalWrite (POWERSIM, LOW);  // turn transistor OFF
  delay(1); // give time to power down    
  }  // end of powerOFF SIM

enum SIM
    {
      START,SENSOR, 
      PWRSIM,ATTACHGPRS,
      OPENGPRS,QUERYGPRS,
      INITHTTP,SENDPARA,
      ADDLEVEL,CLOSEURL,
      ENDPARA,ENDGPRS,
      SIMOFF 
    };

SIM SIM_state  = START;
uint32_t stateTime;


void loop() {
  
  switch (SIM_state)
  {
    case START:
    	if(f_wdt != 1){  //Wait here indefenetly until a flag is rised; TRUE if f_wdt = 0
    	  //SIM_state = START;
    	  //break;
		    return;
	    }
	    
	      SIM_state = SENSOR; // wake-up if f_wdt = 1
      break;
      
    //Get reading from Hall sensor
    case SENSOR:
      int sensor_max;
      // Sensor ON
      powerONSensor();
	    sensor_max = getMaxValue();  
      voltage = (sensor_max / 1024.0) * 5000; // Gets you voltage in mV
      level = (voltage/5000) * 100 ; // Get you level in %  
      Serial.println(level,5);// the '5' after level allows you to display 5 digits after decimal point
	    // wait, certain amount of time, then OFF sensor
	    delay(100);
	    // Sensor OFF
	    powerOFFSensor();
	    SIM_state = PWRSIM;
      break;
      
    case PWRSIM:
      //Power Boost Switch ON 
      powerONBoost();
      //PowerSIM ON                               
      powerONSIM(); //High pulse 1.2 seconds, makes Vc = 0 for 1.2 seconds
      delay(1200);
      powerOFFSIM(); // Vc=VBAT
      SIM_state = ATTACHGPRS;
      break;
      
    case ATTACHGPRS:
      Serial.println("AT+CGATT=1"); // Attach to GPRS
      /*
      if (millis() - stateTime >= 2000){ // 2seconds? 
        SIM_state = OPENGPRS;
        stateTime = millis(); //Note the time when 2 seconds elapsed
      }
      */
      SIM_state = OPENGPRS;
      delay(2000);
      break; // do next stuff after the switch-case, after "{"
      
    case OPENGPRS:
      Serial.println("AT+SAPBR=1,1"); // Open a GPRS context
      /*
      if (millis() - stateTime >= 2000){ // 2seconds? 
        SIM_state = QUERYGPRS;
        stateTime = millis(); //Note the time when 2 seconds elapsed
      }
      */
      SIM_state = QUERYGPRS;
      delay(2000);
      break;
      
    case QUERYGPRS:
      Serial.println("AT+SAPBER=2,1"); // To query the GPRS context, 
      //if its OK continue to the next STATE, ELSE go back to ATTACHGPRS
      SIM_state = INITHTTP;
      //else
      //SIM_state = ENDGPRS;
      //SIM_state = INITHTTP; 
      break; 
    //loop ATTACHGPRS, OPENGPRS, QUERYGPRS until you get a valid connection??
    
    
    case INITHTTP:
      Serial.println("AT+HTTPINIT"); // Initialize HTTP, The AT+HTTPINIT command initializes the HTTP service
      delay(1000);
      SIM_state = SENDPARA; 
      break; 
      
    case SENDPARA:
      Serial.println("AT+HTTPPARA=\"URL\",\"http://www.push2gas.com/add_tanklevel.php?l="); // Send PARA command, The AT+HTTPPARA command sets up HTTP parameters for the HTTP call.
      delay(50);
      SIM_state = ADDLEVEL;
      break;
      
    case ADDLEVEL:
      Serial.println(level); // Add tank level to the url, validate this measure with a reference table?
      delay(2000);
      SIM_state = CLOSEURL;
      break;
      
    case CLOSEURL:
      Serial.print("\"\r\n"); // close url
      delay(2000);
      SIM_state = ENDPARA;
      break;
      
    case ENDPARA:
      Serial.println("AT+HTTPPARA=\"CID\",1"); // End the PARA, The AT+HTTPARA=CID, 1 command sets the context ID. It returns OK
      delay(2000);
      Serial.println("AT+HTTPACTION=0"); // HTTP action, The AT+HTTPACTION command is used to perform HTTP actions such HTTP GET or HTTP POST.
      delay(3000);                         // For Method, possible values are, 0: READ, 1: POST, 2: HEAD
      Serial.println("AT+HTTPTERM"); // End HTTP, The AT+HTTPTERM command terminates the connection, but not the GPRS connection
      delay(3000);
      SIM_state = ENDGPRS; //get ready for the next conection
      break; //breaks and continues the program
    
    case ENDGPRS: //Terminate GPRS connection
      Serial.println("AT+CGATT=0"); //
      /*
      if (millis() - stateTime >= 2000){ // 2seconds? 
        SIM_state = SIMOFF;
        stateTime = millis(); //Note the time when 2 seconds elapsed
      }
      */
      delay(2000);
      SIM_state = SIMOFF;
      break;
    
    case SIMOFF: //  Sleep Mode
      
      //PowerSIM OFF!                               
      powerONSIM(); //Vc=low pulse 1.2 seconds
      delay(1200);
      /*
      if (millis() - stateTime >= 1200){ // 1.2seconds to power SIM On
        return;
      }*/
      powerOFFSIM(); //Vc=VBAT
      //power Switch OFF
      powerOFFBoost();
      SIM_state = START; // Start over

      f_wdt = 0;// Clear the flag so we can run above code again after the MCU wake up
      
      enterSleep(); // Re-enter sleep mode.
      
      break;
  }
}
