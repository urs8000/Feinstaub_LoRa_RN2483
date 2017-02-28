#Feinstaub-Sensor connected with LoRaWan by RN2483
### based on the ESP8266 version of the community Stuttgart (Open Data Stuttgart)
http://codefor.de/stuttgart/  build: http://luftdaten.info/feinstaubsensor-bauen/

### used hardware
microcontroller:	Teensy 3.2
Feinstaubsensor:	SDS011		http://inovafitness.com/en/Laser-PM2-5-Sensor-SDS011-35.html
Temp/Hum:			SHT21		connected via I2C
Pressure:			BMP280 		    "
LoRa Module:		RN2483		on breadboard
LiPo step-up		pololu		Pololu 5V Step-Up Voltage Regulator U1V11F5
LiPo Batteries		4 x 18650 parralel  (4 x 2500mA)

### power measurement
running:  130 - 170mA
sleeping:   4mA

