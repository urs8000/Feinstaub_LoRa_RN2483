/*******************************************************************************
 * Copyright (c) 2016 Maarten Westenberg
 * based on work of Thomas Telkamp, Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This sketch sends a valid LoRaWAN packet with payload a DS18B 20 temperature 
 * sensor reading that will be processed by The Things Network server.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 
*  0.1% in g2). 
 *
 * Change DEVADDR to a unique address! 
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 * History: 
 * Jan 2016, Modified by Maarten to run on ESP8266. Running on Wemos D1-mini
 *
 * 2017-01-29 rxf
 *    adopted, to use SDS011 Particulate Matter Sensor
 *	  Sends data every minute to LoRaWan
 *
 *February 2017 by Urs Marti
 * added LoRa-Module RFM95
 * added Sensors BMP280 , SHT21
 * removed ESP parts
 * transmitted data must be compressed
 *   {"P1":"24.1","P2":"12.1"}
 * tosend:   123456;    SDS011 Sensor ID
 *            137.4;    PM10
 *             48.7;    PM2.5
 *              9.9;    temp
 *             25.5;    humidity
 *            998.4;    pressure
 *              3.8;    Batt voltage
 *              4.0     Solar voltage
 * tosend:  123456;137.4;48.7;9.9;25.5;998.4;3.8;4.0
 * message: 3132333435363B3133372E343B34382E373B392E393B32352E353B3939382E343B332E383B342E30  <- compress!
 *  changed to RN2483
 *   
 *  added sleeping for LiPo Battery usage
 *   -- SDS010  sleeping:    15mA  ,  Pwr disabled 0.5mA  ,  running   90 - 115mA
 *   -- teensy , SDS sleep   34mA  ,                         running  128 - 151mA  Pwr module only fo fan
 *   -- teensy , SDS sleep   67mA  ,                         running  150 - 175mA  Pwr module also 5V for teensy
 *   
 *   
 *****************************************************************************************************************/

#include <rn2483.h>
#include <SparkFunHTU21D.h>             // temp & hum
#include <Adafruit_Sensor.h>            // 
#include <Adafruit_BME280.h>            // pressure  (temp & hum)
#include <ADC.h>                        // Analog Conversion Library for Teensy
#include <Snooze.h>                     // deep sleep module
#include <myDebug.h>                    // personal debug part by bernhard breitenmoser


// Serial port definitions                 for teensy 3.2
#define LoRaSerial Serial1              // HWserial Rx1 = 0, Tx1 =  1       
#define serialSDS  Serial2              // HWserial Rx2 = 9, Tx2 = 10


//---------------------------------------------------------
// LoRaWAN settings (for thethingsnetwork)
//---------------------------------------------------------
// include all keys
#include <Keys_prod_feinstaub_feinstaub_0002_RN2483.h>

// output from "console"   --> Keys.....h
// String AppEUI  = "1234567890123456";
// String NwkSKey = "12345678901234567890123456789012";
// String AppSKey = "12345678901234567890123456789012";
// String addr    = "12345678";
// **********************************************************
// ******   Above settinge have to be adopted !!! ***********
// **********************************************************


//---------------------------------------------------------
// Global Variables
//---------------------------------------------------------
#define my_DEBUG  1                       // set 1 for debug messages on Serial Monitor
String tosend = "";
int led = 13;
String version = "Feinstaub Sensor";
uint32_t ts1;                              // for loop time measurement
uint32_t ts2;

#define rst 2                              // reset RN2483

//create an instance of the rn2483 library,
//giving the software UART as stream to use,
//and using LoRa WAN
 rn2483 myLora(LoRaSerial);

//instance for ADC
const int readPin  = A9;     // ADC0
const int readPin2 = A2;     // ADC1
int sensorBattValue;
int sensorSolarValue;
ADC *adc = new ADC();        // adc object;


//deep sleep parameters                    // take CARE! only modules for Teensy3.2 & timer wake up  are included 
SnoozeTimer timer;                         // with Win10 (probably others) the USB disconnect/connect while sleeping blings
SnoozeBlock config_teensy32(timer);        // also the Serial Monitor will close  :-(
int who;                                   //
int mSecondsTimer = 10000;                 // sleeping time
int longsleep     = 12;                    // 12 x timer (10Seconds) see setup      ********** sleep between transmition **********


//---------------------------------------------------------
// Sensor declarations
//---------------------------------------------------------
#define SDS011    1	                    		// uses SDS011
#define S_DALLAS  0	                  			// Use DS18B20 for temperature
#define S_DHT     0 	                    	// No DHT22
#define S_SHTxx   1                         // temp & humidity , xx depends on the variant. change routine if needed
#define S_BME280  1                         // pressure  (temp & hum)
#define Solar     1                         // Solar Panel attached?


#if SDS011 == 1
//---------------------------------------------------------
// div. timings for SDS011
//---------------------------------------------------------
#define SDS_SAMPLE_TIME 1000
#define SDS_WARMUP_TIME 10
#define SDS_READ_TIME 5

// SDS-Variables
unsigned long act_milli, prev_milli;		// Timer-Ticks to calculate 1 sec
bool is_SDS_running = true;			    		// true, if SDS011 is running
uint8_t timer_SDS;							        // Timer with 1sec ticks for SDS011 timimg
bool SDSread  = false;
bool SDS_FAIL = false;
int SDS_pwr   = 6;                      // control step-up module (pololu 5Vfixed with SHDN*)

// Variables to calculate avereage for SDS011-Data
int sds_pm10_sum  = 0;					
int sds_pm25_sum  = 0;
int sds_val_count = 0;

// Kommands to start and stop SDS011     no longer used, because the module will be powered off
   const byte stop_SDS_cmd[] = {0xFF, 0xAA, 0xB4, 0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0xAB};
   const byte start_SDS_cmd[] = {0xAA, 0xB4, 0x06, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};
#endif

#if S_SHTxx == 1
// SHTxx-Variables
  float humi    = 0.0;
  float temp    = 0.0;
  String humi_s = "";
  String temp_s = "";
  bool SHT_FAIL = false;
#endif  

#if S_BME280
// BME-Variables
  float bmeTemp     = 0.0;
  float bmeHumi     = 0.0;
  float bmePress    = 0.0;
  String bmeTemp_s  = "";
  String bmeHumi_s  = "";
  String bmePress_s = "";
  bool BME_FAIL     = false;
#endif

// ------------------------------------------------------  Dallas not tested
#if S_DALLAS == 1
#define ONE_WIRE_BUS 5                      // GPIO5 / D1  -> Data Pin of DS1820
#include "DallasTemperature.h"
  OneWire oneWire(ONE_WIRE_BUS);
  // Pass our oneWire reference to Dallas Temperature. 
  DallasTemperature sensors(&oneWire);
  int numberOfDevices;                      // Number of temperature devices found
bool DALLAS_FAIL = false;  
#endif


//---------------------------------------------------------
// Strings for the measurements of sensors
// Results came always separated by ';' but not at the end!
// one value = 123.4  /  two values = 123.4;78.9
#if SDS011 == 1
String result_SDS = "";
#endif
#if S_DALLAS == 1
String result_DALLAS = "";
#endif
#if S_SHTxx == 1
String result_SHT = "";
#endif
#if S_BME280 == 1
String result_BME = "";
#endif
#if Solar == 1
String voltageSolar_s = "";
#endif
String voltageBatt_s  = "";


//---------------------------------------------------------
// create instances
#if S_SHTxx == 1
//Create an instance for SHTxx
  HTU21D myHumidity;
#endif

#if S_BME280 == 1
// Create instance for BMP280
// #define BME280_ADDRESS    change in Adafruit_BME280.h from 0x77  --> 0x76 !!!
Adafruit_BME280 bme; 
#endif
 

//****************************************************************
// blink an alert if a sensor fails during setup                 *
//****************************************************************
// this is only useful during testing. if a sensor fails in operation it must be transmitted by value
void blinkAlert() {
  for (int i=0; i<5; i++) {
     digitalWrite(led, HIGH);
     delay(100);
     digitalWrite(led, LOW);
     delay(100);
  }
}


// -----------------------------------------------------------------------------------
#if SDS011
//****************************************************************
// convert float to string with a                                *               result_SDS (routine): 18645.0;24.4;12.0
// precision of 1 decimal place                                  *
//****************************************************************
String Float2String(const float value) {
	// Convert a float to String with two decimals.
	char temp[15];
	String s;

	dtostrf(value,13, 1, temp);
	s = String(temp);
	s.trim();
	return s;
}

// -----------------------------------------------------------------------------------
//****************************************************************
// read SDS011 sensor values                                     *
//****************************************************************
void sensorSDS() {
	char buffer;
	int value;
	int  len = 0;
	int  pm10_serial = 0;
	int  pm25_serial = 0;
  long SDS_ID      = 0;
	int  checksum_is = 0;
	int  checksum_ok = 0;
  SDS_FAIL = false;

	if (! is_SDS_running) {
    SDSread = false;
		return;
	}
	
	// SDS runs: read serial buffer
	while (serialSDS.available() > 0) {
		buffer = serialSDS.read();
//			DEBUG_PLN(String(len)+" - "+String(buffer,DEC)+" - "+String(buffer,HEX)+" - "+int(buffer)+" .");
//			"aa" = 170, "ab" = 171, "c0" = 192
		value = int(buffer);
		switch (len) {
			case (0): if (value != 170) { len = -1; }; break;                      // AA
			case (1): if (value != 192) { len = -1; }; break;                      // C0 
			case (2): pm25_serial = value;         checksum_is  = value; break;
			case (3): pm25_serial += (value << 8); checksum_is += value; break;
			case (4): pm10_serial = value;         checksum_is += value; break;
			case (5): pm10_serial += (value << 8); checksum_is += value; break;
			case (6): SDS_ID = value;              checksum_is += value; break;
			case (7): SDS_ID += (value << 8);      checksum_is += value; break;
			case (8):
//					   DEBUG_PLN("Checksum is: "+String(checksum_is % 256)+" - should: "+String(value));
					  if (value == (checksum_is % 256)) { checksum_ok = 1; } else { len = -1; }; break;
			case (9): if (value != 171) { len = -1; }; break;
		}
		len++;
		if ((len == 10 && checksum_ok == 1) && (timer_SDS > SDS_WARMUP_TIME)) {
			if ((! isnan(pm10_serial)) && (! isnan(pm25_serial))) {
				sds_pm10_sum += pm10_serial;
				sds_pm25_sum += pm25_serial;
				sds_val_count++;
			}
			len = 0; checksum_ok = 0; pm10_serial = 0.0; pm25_serial = 0.0; checksum_is = 0;
		}
		// yield();                          // used for ESPxx series to ensure loop could be leaved for wifi things
	}

	// Data for SDS_READTIME time is read: now calculate the average and return value
	if (timer_SDS > (SDS_WARMUP_TIME + SDS_READ_TIME)) {
		// Calculate average
		DEBUG_PLN("Sum: " + String(sds_pm10_sum) + "  Cnt: " + String(sds_val_count));
		String sp1_av = Float2String(float(sds_pm10_sum)/(sds_val_count*10.0));
		String sp2_av = Float2String(float(sds_pm25_sum)/(sds_val_count*10.0));
    String SDS_ID_s = Float2String(SDS_ID);          // DEBUG_PLN(SDS_ID_s);
      int dezPoint = SDS_ID_s.indexOf('.');
      SDS_ID_s = SDS_ID_s.substring(0, dezPoint);
    DEBUG_PRT("Dev-ID: "+SDS_ID_s+"  "); DEBUG_PLN(SDS_ID, HEX);
		DEBUG_PLN("PM10:   "+sp1_av);
		DEBUG_PLN("PM2.5:  "+sp2_av);
		DEBUG_PLN("------");
		// result_SDS = Value2JsonMQTT("P1",sp1_av.c_str(),"P2",sp2_av.c_str(),NULL);
		// clear sums and count
		sds_pm10_sum = 0; sds_pm25_sum = 0; sds_val_count = 0;
		// and STOP SDS
		serialSDS.write(stop_SDS_cmd,sizeof(stop_SDS_cmd));	       // shut down by command
    delay(10);  digitalWrite(SDS_pwr, 0);                      // power off from supply
		is_SDS_running = false;
		DEBUG_PLN("SDS stopped, PwrModule disabled");

    // construct a result of PM10;PM2.5
    result_SDS = "";
    result_SDS += SDS_ID_s;                // xxxxx             (assumed 5 positions only)
    result_SDS += ";";
    result_SDS += sp1_av;                  //   x.x ... xx.x
    result_SDS += ";";
    result_SDS += sp2_av;                  //   x.x ... xx.x

    SDSread = true;
    DEBUG_PLN("result_SDS (routine): "+result_SDS);
	}
}
#endif


// -----------------------------------------------------------------------------------
#if S_DALLAS==1
//***************************************************************
// DALLAS-Sensor auslesen					                              *
//***************************************************************
// Dallas sensors (can be more than 1) have channel codes 3 and above!
void sensorDallas() {
	  result_DALLAS = "";
    DALLAS_FAIL = false;
   
	  uint8_t ind;
	  uint8_t erg[20];

    DEBUG_PRT("Search Dallas");
	  DeviceAddress tempDeviceAddress; 			// We'll use this variable to store a found device address
	  sensors.requestTemperatures();
	  for(int i=0; i<numberOfDevices; i++)
	  {
		// Search the wire for address
		if(sensors.getAddress(tempDeviceAddress, i)) {
			float tempC = sensors.getTempC(tempDeviceAddress);
			// Output the device ID
			if (debug>=1) {
				DEBUG_PRT(F("! DS18B20 dev ("));
				DEBUG_PRT(i);
			}
			int ival = (int) tempC;					// Make integer part
			int fval = (int) ((tempC - ival)*10);	// Fraction. Has same sign as integer part
			if (fval<0) fval = -fval;				// So if it is negative make fraction positive again.
			result_DALLAS = String("{\"T\":\"" + String(ival) + "." + String(fval)+"\"}");
			if (debug>=1) {
				DEBUG_PRT(") ");
				DEBUG_PLN(result_DALLAS);
			}
	  }
    else {
       blinkAlert();
       DALLAS_FAIL = true;
    }
	 //else ghost device! Check your power requirements and cabling
	}
}
#endif	  

// -----------------------------------------------------------------------------------
#if S_SHTxx == 1
//****************************************************************
// read SHTxx sensor values                                     *
//****************************************************************
void sensorSHT() {
  humi = myHumidity.readHumidity();
  temp = myHumidity.readTemperature();

  humi_s = Float2String(humi);
  temp_s = Float2String(temp);
  
  result_SHT  = "";
  result_SHT  = temp_s;            // -xx.x ... xx.x
  result_SHT += ";";
  result_SHT += humi_s;            //   x.x ... xx.x
  
  DEBUG_PRT("Temperature:");
  DEBUG_PRT(temp, 1);
  DEBUG_PRT("C  ");
  DEBUG_PRT(temp_s);
  DEBUG_PRT(" Humidity:");
  DEBUG_PRT(humi, 1); 
  DEBUG_PRT("%  ");
  DEBUG_PRT(humi_s);
  DEBUG_PLN();
}
#endif

// -----------------------------------------------------------------------------------
#if S_BME280 == 1
//****************************************************************
// read BME280 sensor values    only pressure will betransmitted *
//****************************************************************
void sensorBME() {
  bmeTemp  = bme.readTemperature();
  bmeHumi  = bme.readHumidity();
  bmePress = bme.readPressure() / 100.0F;

  bmeTemp_s  = Float2String(bmeTemp);
  bmeHumi_s  = Float2String(bmeHumi);
  bmePress_s = Float2String(bmePress);
  
  result_BME  = "";
  result_BME  = bmePress_s;          // xxx.x ... xxxx.x

    DEBUG_PRT("Temperature = ");
    DEBUG_PRT(bmeTemp);
    DEBUG_PRT(" *C  ");
    DEBUG_PRT(bmeTemp_s);
    DEBUG_PLN();
    DEBUG_PRT("Pressure = ");
    DEBUG_PRT(bmePress);
    DEBUG_PRT(" hPa  ");
    DEBUG_PRT(bmePress_s);
    DEBUG_PLN();
//    DEBUG_PRT("Approx. Altitude = ");
//    DEBUG_PRT(bme.readAltitude(SEALEVELPRESSURE_HPA));
//    DEBUG_PRT(" m");
//    DEBUG_PLN();
    DEBUG_PRT("Humidity = ");
    DEBUG_PRT(bmeHumi);
    DEBUG_PRT(" %  ");
    DEBUG_PRT(bmeHumi_s);
    DEBUG_PLN();
}
#endif


//---------------------------------------------------------
void led_on()  {
  digitalWrite(13, 1);
}

void led_off() {
  digitalWrite(13, 0);
}

//------------------------------------------------------------------------------------
void readSensors()  {
      #if S_DALLAS == 1
      // Read the sensor, store result in result_DALLAS
	       sensorDallas();
      #endif

      #if S_SHTxx == 1
      // Read the sensor, store result in result_SHT
         sensorSHT();
      #endif

      #if S_BME280 == 1
      // Read the sensor, store result in result_BME
         sensorBME();
      #endif

      #if Solar == 1
      // Read Solar Voltage
         Solar_Voltage();  
      #endif

      // Read the LiPo Battery Voltage
         Batt_Voltage();

     // original tosend     46037;14.7;9.2;36.7;24.3;982.2
     DEBUG_PRT("result_SDS: "); DEBUG_PLN(result_SDS);
     
	   tosend = "";

     tosend += result_SDS;
     tosend += ";";
     tosend += result_SHT;
     tosend += ";";
     tosend += result_BME;
     tosend += ";";
     tosend += voltageBatt_s;
     #if Solar == 1
      tosend += ";";
      tosend += voltageSolar_s;
     #endif

     DEBUG_PRT("tosend: ");
     DEBUG_PLN(tosend);

      SDSread    = false;
      //
      // return with completed message in tosend      
}

//--------------------------------------------------------------------------------
void Batt_Voltage() {                                   // A9 (pin23)
  // Measure voltage in "x.xx" volts
  sensorBattValue = adc->analogRead(readPin);           // read Battery
  delay(100);
  sensorBattValue = adc->analogRead(readPin);           // read Battery
  DEBUG_PRT(sensorBattValue);
  float voltageBatt = (sensorBattValue * 3.6/adc->getMaxValue(ADC_0));
  voltageBatt_s = Float2String(voltageBatt);
  DEBUG_PLN("  V_Batt:  "+voltageBatt_s);
}

#if Solar == 1
void Solar_Voltage() {                                  // A2 (pin16)
  // Measure voltage in "x.xx" volts
  sensorSolarValue = adc->analogRead(readPin2, ADC_1);  // read Solar
  delay(100);
  sensorSolarValue = adc->analogRead(readPin2, ADC_1);  // read Solar
  DEBUG_PRT(sensorSolarValue);
  float voltageSolar = (sensorSolarValue * 5.0/adc->getMaxValue(ADC_1));
  voltageSolar_s = Float2String(voltageSolar);
  DEBUG_PLN("  V_Solar: "+voltageSolar_s);
}
#endif



// -----------------------------------------------------------------------------------
// Set my_Debug 0 to supress Serial messages once the unit is running reliable
// 
void setup() {
// initialize the digital pin as an output.
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);                     // LED on
  pinMode(SDS_pwr, OUTPUT);
  digitalWrite(SDS_pwr, 1);

  pinMode(readPin,  INPUT);                     //A9  for Battery
  pinMode(readPin2, INPUT);                     //A2 for SolarPanel
  
  delay(2000);  DEBUG_BEGIN();                  // init with 57600
  delay(100);   DEBUG_PRT("\n\n\n");            //
  delay(100);   DEBUG_PRT("starting ...");      //
  delay(100);   DEBUG_PLN(version);             //
  
  delay(100);   LoRaSerial.begin(57600);        // high speed to LoRa module
  delay(100);   serialSDS.begin(9600);          // Teensy3.2: Rx1 = pin0 Tx1 = pin1
  delay(100);
  
  //reset rn2483
  pinMode(rst, OUTPUT);
  digitalWrite(rst, HIGH);
  digitalWrite(rst, LOW);
  delay(500);
  digitalWrite(rst, HIGH);

  //initialise the rn2483 module
  myLora.autobaud();

  //print out the HWEUI so that we can register it via ttnctl     // 0004A30B001AF61B
  DEBUG_PLN("When using OTAA, register this DevEUI: ");      delay(100);
  DEBUG_PLN(myLora.hweui());                 delay(100);
  DEBUG_PRT("RN2483 version number: ");        delay(100);
  DEBUG_PLN(myLora.sysver());                delay(100);
  DEBUG_PRT(" connecting ... ");
  ts1 = millis();
  //ABP: init(String AppEUI, String NwkSKey, String AppSKey, String addr);
  myLora.init(AppEUI, NwkSKey, AppSKey, addr);     // taken from the private h-file

  //OTAA: init(String AppEUI, String AppKey);
  //myLora.init("70B3D57ED00001A6", "A23C96EE13804963F8C2BD6285448198");

  ts2 = millis();
  DEBUG_PRT(" ... connected in ");
  DEBUG_PRT(ts2 - ts1);
  DEBUG_PRT(" millis");
  DEBUG_PLN();
  
  
#if S_DALLAS==1
	sensors.begin();
	numberOfDevices = sensors.getDeviceCount();
		DEBUG_PRT("DALLAS #:");
		DEBUG_PRT(numberOfDevices); 
		DEBUG_PLN(" ");
#endif

#if S_SHTxx == 1
  myHumidity.begin();
  DEBUG_PLN("SHTxx initialized ...");
#endif

#if S_BME280 == 1
  if (!bme.begin()) {
    DEBUG_PLN("Could not find a valid BME280 sensor, check wiring!");
    while (1)  { blinkAlert(); }
  }
  DEBUG_PLN("BME280 initialized ...");
#endif

  delay(500);

   DEBUG_PRT("DeviceAddr: ");
   DEBUG_PLN(addr);
   DEBUG_PLN(F("Starting with personalized device ..."));
   DEBUG_PRT(longsleep * 10/60);
   DEBUG_PLN(F(" minute interval"));

    adc->setReference(ADC_REF_3V3, ADC_0); // change all 3.3 to 1.2 if you change the reference to 1V2
    ///// ADC0 ////
    adc->setAveraging(4);                       // set number of averages
    adc->setResolution(12);                     // set bits of resolution
    adc->setConversionSpeed(ADC_HIGH_SPEED);    // change the conversion speed
    adc->setSamplingSpeed(ADC_HIGH_SPEED);      // change the sampling speed
    ////// ADC1 /////
    #if ADC_NUM_ADCS>1
    adc->setAveraging(32, ADC_1);               // set number of averages
    adc->setResolution(16, ADC_1);              // set bits of resolution
    adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_1);    // change the conversion speed
    adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_1);      // change the sampling speed
    #endif
    sensorBattValue  = adc->analogRead(readPin);           // read Battery
    sensorSolarValue = adc->analogRead(readPin2, ADC_1);   // read Solar

    /********************************************************
     Set Low Power Timer wake up in milliseconds.
     ********************************************************/
     timer.setTimer(mSecondsTimer);           // milliseconds  --> 10Seconds

  digitalWrite(led, LOW);                     // LED off
 
}


// -----------------------------------------------------------------------------------
// main loop
// Loop is simple: read sensor value and send it to the LoRaWAN network
//---------------------------------------------------------------------

void loop() {
	DEBUG_PLN("loop: Starting");
    //  int cnt_loop  = 0;
    //  int cnt_while = 0;
    //  int cnt_if    = 0;
    //  int cnt_read1 = 0;
    //  int cnt_read2 = 0;
    //  int cnt_sleep_long  = 0;
    //  int cnt_sleep_short = 0;
  
// The do_send function puts a message in the queue and then puts
// itself to sleep. When waking up, will again work on queue again.

    // DEBUG_PRT("loop "); DEBUG_PRT(cnt_loop); DEBUG_PLN(); cnt_loop++;
  
	while(1) {
		act_milli = millis();			                           // read system-tick

    // DEBUG_PRT("while "); DEBUG_PRT(cnt_while); DEBUG_PLN(); cnt_while++;

		if((act_milli - prev_milli) >= SDS_SAMPLE_TIME) {    // after SAMPLE_TIME (==0 1sec)
			prev_milli = act_milli;
			timer_SDS += 1;			    	                         // Count SDS-Timer
        // DEBUG_PRT("if "); DEBUG_PRT(cnt_if); DEBUG_PLN(); cnt_if++;
			sensorSDS();			      		                       // check (and read)  SDS011		   returns result_SDS as string
		}

    if ( SDSread == true )  {
       // DEBUG_PRT("read 1 "); DEBUG_PRT(cnt_read1); DEBUG_PLN(); cnt_read1++;
     readSensors();                                       // Put job in run queue(send mydata buffer)
       // DEBUG_PRT("read 2 "); DEBUG_PRT(cnt_read2); DEBUG_PLN(); cnt_read2++;

      led_on();
      ts1 = millis();
        myLora.txUncnf( tosend );                          // 
      ts2 = millis();
      DEBUG_PRT(" ... done in ");
      DEBUG_PRT(ts2 - ts1);
      DEBUG_PRT(" millis");
      DEBUG_PLN();
      // cnt_loop  = 0;
      // cnt_while = 0;
      // cnt_if    = 0;
      // cnt_read1 = 0;
      // cnt_read2 = 0;
      // cnt_sleep_long  = 0;
      // cnt_sleep_short = 0;      
      led_off();
      Serial.flush();
      delay(50);

    //------------------------------------------------------------
    // sleep longsleep (12) * Snooze   (12 * 10 Seconds)
    for ( int sleepcnt = 0; sleepcnt < longsleep; sleepcnt++ ) {
        // DEBUG_PRT("sleep long "); DEBUG_PRT(cnt_sleep_long); DEBUG_PLN(); cnt_sleep_long++;
        // delay(10000);
      who = Snooze.deepSleep( config_teensy32 );
    }
  
      // 
      // Now start SDS senor  and sleep again 10 Seconds
         digitalWrite(SDS_pwr, 1);                              // eneble pololu step-up module
         delay(50);
         DEBUG_PLN("SDS start");
         serialSDS.write(start_SDS_cmd,sizeof(start_SDS_cmd));  // not really needed. it should start with default config
         // DEBUG_PRT("sleep short "); DEBUG_PRT(cnt_sleep_short); DEBUG_PLN(); cnt_sleep_short++;

         // delay(10000);   
      who = Snooze.deepSleep( config_teensy32 );                // return module that woke processor
      who = Snooze.deepSleep( config_teensy32 );                // return module that woke processor


         DEBUG_PLN("SDS started");
         is_SDS_running = true;
         timer_SDS = 0;              // start timer
     
    }
		delay(100);
	}
	
}
//----------------------------------------------------------------------------------------------
/*


*/
