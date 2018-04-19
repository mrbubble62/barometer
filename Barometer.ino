

#define ARDUINO_ARCH_ESP32

#include <Adafruit_BME280.h>
#include <GxFont_GFX.h>
#include <GxEPD.h>
#include <GxGDEW0213Z16/GxGDEW0213Z16.cpp>  // 2.13" b/w/r
#include <Fonts/Org_01.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>
#include <CircularBuffer.h>
#include "NMEA2000_CAN.h"
#include "N2kMessages.h"
#include "font.h"

/*
* Connect the E-Paper display to the following pins:
*
* E-Paper display | ESP32
*    DIN      MOSI 23
*    CLK      SCK 18
*    CS		  5
*    DC       26
*    RST      27
*    BUSY     32
*/

// battery monitor ADC
#define ADC_PIN 35
static const float R1 = 81.79f; // resistor divider measured value, VCC
static const float R2 = 21.74f; // resistor divider measured value, GND
static const float VREF = 1.439f;

//e-paper display
GxIO_Class io1(SPI, 5, 26, 27); 
GxGDEW0213Z16 display1(io1, 27, 32);
#define RST_PIN 27
#define screenX 212
#define screenY 104
#define XscaleHours 48

// graph
double ox, opy, oty, ohy;
double y;
uint8_t x;  // ! uint16_t for screenX > 255
double height = 87;  //graph height

// Xscale hours

// pressure range for graph
#define plo 970 //millibars low
#define phi 1050 //millibars high

// temp range for graph
int8_t meanT=7; // center of temperature scale in tens of F
int tlo = meanT -20; //deg f low
int thi = meanT + 20; //deg f high

// humidity range for graph
#define hlo 20 //	20% RH
#define hhi 100 // 100% RH

//sensor data buffer
CircularBuffer<double, screenX> p;
CircularBuffer<double, screenX> t;
CircularBuffer<double, screenX> h;
bool empty = true;

typedef struct {
	uint8_t * buffer;
	size_t head;
	size_t tail;
	size_t size; //of the buffer
} circular_buf_t;

// Sensors
//BME280  bme;
Adafruit_BME280  bme;
/*
* Connect the BME280 to the following pins:
*
*    SCL     22
*    SDA     21
*/

double tC;  // Celcius
double hPa;  // millibars
double RH; // relative humidity
double Dp; // 3 hour pressure difference
float vBatt = 0;
//#define EEPROM_ADR_CONFIG 0 // eeprom address for config params
//#define P0 1005.3
#define stbyPin 14 // CAN tranceiver standby

//N2K
int SID = 1;
tN2kMsg N2kMsg;
int beginStatus;

// timing
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

long displayloop = 999;  // seconds since last display refresh
long longLoop = 999;	// seconds since last data buffer push
long longLoopPeriod = 815; //13
int DisplayRefreshTime=5;  // display refresh in minutes
bool OnBattery = false; // battery status

const unsigned long TransmitMessagesEnv[] PROGMEM = { 130310L, 130311L,0 }; //Atmospheric Pressure, Temperature
																			
// slim down the satartup as we are going to be doing this alot
void setup()
{
	// hard reset e-paper display
	pinMode(RST_PIN, OUTPUT);
	digitalWrite(RST_PIN, LOW);
	delay(20);
	digitalWrite(RST_PIN, HIGH);
	delay(20);
	display1.init(115200);

	//pinMode(LED_BUILTIN, OUTPUT);
	// battery voltage adc
	adcAttachPin(35);
	analogReadResolution(10);
	analogSetAttenuation(ADC_0db);			   
	OnBattery = GetBatteryStatus();

	//Blink(5, 300);
	Serial.begin(115200);
	delay(100);
	Serial.println("Environmental Sensor");
	delay(100);
	Serial.println("BME280 Start");
	if (!bme.begin(0x76)) {
		Serial.println(F("BME280 init failed!"));
		while (!bme.begin()) { Serial.println(F("BME280 init failed!")); delay(1000); }
	}
	else Serial.println(F("BME280 init success!"));
	// use "Weather Station" settings
	bme.setSampling(Adafruit_BME280::MODE_FORCED,
		Adafruit_BME280::SAMPLING_X1, // temperature
		Adafruit_BME280::SAMPLING_X1, // pressure
		Adafruit_BME280::SAMPLING_X1, // humidity
		Adafruit_BME280::FILTER_OFF);
	
	if (!OnBattery) {
		// turn on can tranciver
		pinMode(stbyPin, OUTPUT);
		digitalWrite(stbyPin, LOW);
		// setup NMEA 2000 bus
		NMEA2000.SetInstallationDescription1("Mr Bubble Barometer & Barograph");
		NMEA2000.SetProductInformation("01290518", // Manufacturer's Model serial code
			668, // Manufacturer's product code
			"Barometer",  // Manufacturer's Model ID
			"1.0.0.0 (2018-04-08)",  // Manufacturer's Software version code
			"1.0.0.0 (2018-04-08)", // Manufacturer's Model version
			1,	// load equivalency *50ma
			0xffff, // NMEA 2000 version - use default
			0xff // Certification level - use default	
		);
		NMEA2000.SetDeviceInformation(290517, // Unique number. Use e.g. Serial number.
			130, // Device function=Temperature See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
			85, // Device class=Sensor Communication Interface. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
			2040, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
			4
		);
		NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 22);
		NMEA2000.EnableForward(false);
		NMEA2000.ExtendTransmitMessages(TransmitMessagesEnv);
		NMEA2000.SetN2kCANMsgBufSize(5);
		NMEA2000.Open();
	}
	else {
		//place can tranciver in standby
		pinMode(stbyPin, OUTPUT);
		digitalWrite(stbyPin, HIGH);
		digitalWrite(GPIO_NUM_16, HIGH);
	
	}
	display1.setRotation(3);

	// pad the buffer a little to show its working
	NewData(); NewData(); NewData(); NewData(); NewData(); NewData(); NewData(); NewData(); NewData(); 
	NewData(); NewData(); NewData(); NewData(); NewData(); NewData(); NewData(); NewData(); NewData(); 

	Serial.println(F("Starting\n\n"));
}

bool SDEBUG = false; // debug print
bool SPRINT = false; // speed up for testing
long sleepseconds=10; // how long did I sleep for?


// on battery only need to up date on long loop
// possibly wake up BLE and transmit status
// on N2K bus power we can charge and send N2K messages every 1s
void loop() {
	char command = getCommand();
	switch (command)
	{
	case 'b':
		OnBattery = !OnBattery;
		Serial.print("OnBattery:"); Serial.println(OnBattery);
		break;
	case 'd': //toggle debug
		SDEBUG = !SDEBUG;
		Serial.print("SDEBUG:"); Serial.println(SDEBUG);
		break;
	case 's':
		SPRINT = !SPRINT;
		Serial.print("SPRINT:"); Serial.println(SPRINT);
		break;
	default:
		break;
	}

	//Detect Battery
	// ?
	if (!OnBattery) {
		// 1 second loop
		delt_t = millis() - count;
		if (delt_t > 1000) { // update once per second independent of read rate
			Now = micros();
			deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
			lastUpdate = Now;
			count = millis();
			ReadSensors();
			if (SDEBUG) printData();
			SetN2kEnvironmentalParameters(N2kMsg, SID, N2kts_OutsideTemperature, CToKelvin(tC), N2khs_InsideHumidity, RH, hPa * 100);
			NMEA2000.SendMsg(N2kMsg);
			displayloop++;
			longLoop++; //databuffer loop
			SID++; if (SID > 254) { SID = 1; }
		}
	}
	else {
		//if we are on battery 
		// sleep for 1-5 min 
		// wake up, fix clocks, read sensors, update display, and sleep.
		//FIX CLOCKS
		displayloop += sleepseconds;
		longLoop += sleepseconds;
	}

	//1m display update loop
	if (!OnBattery && displayloop > 60 * DisplayRefreshTime) { displayloop = 0; DisplyLoop(); }
	//5m display update loop
	if (OnBattery && displayloop >  60 * DisplayRefreshTime) { displayloop = 0; DisplyLoop(); }

	//accumulate graph data evey 13.58min, 815s/12h for two days display
	if (longLoop > longLoopPeriod) { longLoop = 0;  NewData();}
	if (SPRINT) {
		// 5 sec test
		if (longLoop > 5) { longLoop = 0;  NewData(); }
	}
	if (!OnBattery) {
		NMEA2000.ParseMessages();
	}
	else {
		Serial.print("SLEEP");
		esp_sleep_enable_timer_wakeup(10000000); //10 seconds
		int ret = esp_light_sleep_start();
		Serial.printf("light_sleep: %d\n", ret);
		Serial.print("WAKE");
	}
}

void PrintBuffer() {
	Serial.print("Buffer "); Serial.println(p.capacity() - p.available());
	for (x = 0; x < p.capacity() - p.available(); x++) {
		int  n = p[x]-1000;
		Serial.print(n); Serial.print(" ");
	}
	Serial.println();
}

// e-paper refresh loop
void DisplyLoop()
{
	//change the horizontal scale as data fills
	//if (DisplayRefreshTime < 5 && p.capacity() - p.available() > p.capacity() / 2) {
	//	DisplayRefreshTime = 5; //5 min
	//	Serial.println("Buffer half full");
	//}
	//if (p.isFull()) {
	//	if (DisplayRefreshTime != 15) {
	//		Serial.println("Buffer full");
	//		DisplayRefreshTime = 15; //15 min
	//	}
	//}
	DisplayRefreshTime = 1;
	// clear screen
	display1.fillScreen(GxEPD_WHITE);

	// display graph
	DisplayBarograph();
	// 3 hour trend icon and storm warning
	TrendIcon(Dp, 88, 43);
	// draw borttom text
	UpdateTextDisplay();
	// finally
	display1.update();
	display1.powerDown();
}


float ReadBatteryVoltage(){	
	return (R1 / R2) * VREF * float(analogRead(ADC_PIN)) / pow(2.0f, 10.0f);  // LiPo battery
}

bool GetBatteryStatus() {
	vBatt = ReadBatteryVoltage();
	Serial.print("Battery Voltage = "); Serial.print(ReadBatteryVoltage(), 3); Serial.println(" V");
	if (vBatt > 4.2) {
		return false;
	}
	if (vBatt > 3.6) {
		return false;
	}
	if (vBatt > 3.2) {
		return false;
	}
	if (vBatt < 3.2) {
		NewData();
		esp_deep_sleep(15 * 60 * 1000000); //sleep 15 min
	}
	return true;
}

void ReadSensors() {
	OnBattery = GetBatteryStatus();
	// Send Temperature & Pressure
	bme.takeForcedMeasurement();
	tC = bme.readTemperature();
	hPa = bme.readPressure() / 100; // convert Pa to mb
	RH = bme.readHumidity();
	double f = CtoF(tC);
	meanT = f / 10;
	if (f > (meanT * 10 + 10.0)) { meanT = f / 10 + 10; }
	if (f < (meanT * 10 - 10.0)) { meanT = f / 10 - 10; }
	tlo = meanT * 10 - 20;
	thi = meanT * 10 + 20;
	Dp = p[0] - p[(screenX / 48 * 3)];
}

void CheckI2CError() {
	bme.takeForcedMeasurement();
	if (bme.readTemperature() < -100) {
		Serial.println(F("Resetting BMP"));
		if (!bme.init()) {
			Serial.println(F("BMP init failed!"));
		}
		else Serial.println(F("BMP init success!"));
		ReadSensors();
		printData();
	}
}

// fill circular buffer
void NewData()
{
	CheckI2CError();
	ReadSensors();
	if (SDEBUG) { Serial.print("T "); Serial.print(tC); Serial.print(" meanT "); Serial.print(meanT * 10); Serial.print(" tlo "); Serial.print(tlo); Serial.print(" thi "); Serial.println(thi); }
	p.push(hPa);
	h.push(RH);
	t.push(tC);
}

void DisplayBarograph() {
	// horizontal lines
	display1.drawFastHLine(0, 21, screenX, GxEPD_BLACK); //(screenY - 17) / 4
	display1.drawFastHLine(0, 43, screenX, GxEPD_BLACK);// (screenY - 17) / 4 * 2
	display1.drawFastHLine(0, 65, screenX, GxEPD_BLACK); //(screenY - 17) / 4 * 3
	display1.drawFastHLine(0, screenY - 17, screenX, GxEPD_BLACK);
	// ~12h lines
	display1.drawFastVLine(screenX/4, 0, screenY - 17, GxEPD_BLACK);
	display1.drawFastVLine(screenX/2, 0, screenY - 17, GxEPD_BLACK);
	display1.drawFastVLine(screenX/4*3, 0, screenY - 17, GxEPD_BLACK);
	
	// axis labels
	display1.setTextColor(GxEPD_BLACK);
	// Pressure Y scale
	display1.setFont(&Org_01);
	display1.setCursor(4, screenY - 17 / 4 - 2);	display1.print("1030");
	display1.setCursor(4, 41);	display1.print("1010");
	display1.setCursor(4, 63);	display1.print("990");
	display1.setCursor(4, 85);	display1.print("970");
	// Temperature Y scale
	display1.setCursor(198, 19);	display1.print(thi);
	display1.setCursor(198, 41);	display1.print(meanT * 10);
	display1.setCursor(198, 63);	display1.print(meanT * 10 -10);
	display1.setCursor(198, 85);	display1.print(tlo);
	// Humidity Y scale
	display1.setFont(&Org_01);
	display1.setCursor(108, 19);	display1.print("80%");
	display1.setCursor(108, 41);	display1.print("60%");
	display1.setCursor(108, 63);	display1.print("40%");
	display1.setCursor(108, 85);	display1.print("20%");

	// fill from left until buffer full
	if (p.isFull()) {
		//draw data
		ox = screenX;
		//get left starting y 
		opy = (p[0] - plo) * height / (phi - plo);
		oty = (t[0] - tlo) * height / (thi - tlo);
		ohy = (h[0] - hlo) * height / (hhi - hlo);
		for (x = screenX; x > 0; x--) {		
			//temperature
			float f = CtoF(t[x]);
			y = height - (f - tlo) * height / (thi - tlo);
			display1.drawLine(ox, oty, x, y, GxEPD_BLACK);
			display1.drawLine(ox, oty + 1, x, y + 1, GxEPD_BLACK);
			display1.drawLine(ox, oty - 1, x, y - 1, GxEPD_BLACK);
			oty = y;
			//pressure
			y = height - (p[x] - plo) * height / (phi - plo);
			display1.drawLine(ox, opy, x, y, GxEPD_RED);
			display1.drawLine(ox, opy + 1, x, y + 1, GxEPD_RED);
			display1.drawLine(ox, opy - 1, x, y - 1, GxEPD_RED);
			opy = y;
			//humidity
			y = height - (h[x] - hlo) * height / (hhi - hlo);
			display1.drawLine(ox, ohy, x, y, GxEPD_BLACK);
			ohy = y;
			ox = x;
		}
	}
	else { //buffer not full draw from left side
		display1.setCursor(0, p[0]);
		ox = 0;
		opy = (p[ox] - plo) * height / (phi - plo);
		oty = (t[ox] - tlo) * height / (thi - tlo);
		ohy = (h[ox] - hlo) * height / (hhi - hlo);
		for (x = 0; x < p.capacity() - p.available(); x++) {
			//temperature
			float f = CtoF(t[x]);
			y = height - (f - tlo) * height / (thi - tlo);
			display1.drawLine(ox, oty, x, y, GxEPD_BLACK);
			display1.drawLine(ox, oty + 1, x, y + 1, GxEPD_BLACK);
			display1.drawLine(ox, oty - 1, x, y - 1, GxEPD_BLACK);
			oty = y;

			//pressure
			y = height - (p[x] - plo) * height / (phi - plo);
			display1.drawLine(ox, opy, x, y, GxEPD_RED);
			display1.drawLine(ox, opy + 1, x, y + 1, GxEPD_RED);
			display1.drawLine(ox, opy - 1, x, y - 1, GxEPD_RED);
			opy = y;

			//humidity
			y = height - (h[x] - hlo) * height / (hhi - hlo);
			display1.drawLine(ox, ohy, x, y, GxEPD_BLACK);
			ohy = y;
			ox = x;
		}
	}
}

//The trend shows the direction of change(higher, lower, steady) of the barometric pressure over the last three hours.
// Change in last 3 hrs
enum trend
{
	Steady = 0,
	FallingSlowly,
	RisingSlowly,
	Falling,
	Rising, 
	FallingQuickly,
	RisingQuickly,
	FallingVeryRapidly,
	RisingVeryRapidly
};

// Set trending thresholds
// Steady ........................up to 0.1 mb
// Falling / rising slowly.........0.1 - 1.5 mb
// Falling / rising...............1.6 - 3.5 = F6 / 7 ? -12hrs ?
// Falling / rising quickly.......3.6 - 6.0 = F6 / 7 / 8 ? -6hrs ?
// Falling / rising very rapidly..6 mb + = F7 / 8 / 9 ? -3hrs ?
uint8_t Trend(double Dp) {
	if (Dp >= 6 || Dp <= -6) {
		if (Dp > 0) return trend::RisingVeryRapidly;
		else return trend::FallingVeryRapidly;
	}
	if (Dp >= 3.6 || Dp <= -3.6) {
		if (Dp > 0) return trend::RisingQuickly;
		else return trend::FallingQuickly;
	}
	if (Dp >= 1.6 || Dp <= -1.6) {
		if (Dp > 0) return trend::Rising;
		else return trend::Falling;
	}
	if (Dp >= 0.1 || Dp <= -0.1) {
		if (Dp > 0) return trend::RisingSlowly;
		else return trend::FallingSlowly;
	}
	return trend::Steady;
}

void TrendIcon(double Dp, uint8_t X, uint8_t Y) {
	switch (Trend(Dp)) {
	case trend::Steady:
		Serial.println("Steady");
		break;
	case trend::RisingSlowly:
		display1.drawTriangle(X, Y - Dp - 1, X + 16, Y - Dp - 1, X + 8, Y - Dp - 9, GxEPD_BLACK);
		Serial.println("RisingSlowly");
		break;
	case trend::FallingSlowly:
		display1.drawTriangle(X, Y - Dp + 1, X + 16, Y - Dp + 1, X + 8, Y - Dp + 9, GxEPD_BLACK);
		Serial.println("FallingSlowly");
		break;
	case trend::Rising:
		display1.fillTriangle(X, Y - Dp - 1, X + 16, Y - Dp - 1, X + 8, Y - Dp - 9 , GxEPD_BLACK);
		display1.fillRect(X + 5, Y - Dp - 1, 7, Dp + 1, GxEPD_BLACK);
		Serial.println("Rising");
		break;
	case trend::Falling:
		display1.fillTriangle(X, Y - Dp + 1, X + 16, Y - Dp + 1, X + 8, Y - Dp + 9, GxEPD_BLACK);
		display1.fillRect(X + 5, Y + 2, 7, -Dp, GxEPD_BLACK);
		Serial.println("Falling");
		break;
	case trend::RisingQuickly:
		display1.fillTriangle(X, Y - Dp - 2, X + 16, Y - Dp - 2, X + 8, Y - Dp - 10, GxEPD_RED);
		display1.fillRect(X + 5, Y - Dp - 2, 7, Dp + 2, GxEPD_RED);
		Serial.println("RisingQuickly");
		break;
	case trend::FallingQuickly:
		display1.fillTriangle(X, Y - Dp + 2, X + 16, Y - Dp + 2, X + 8, Y- Dp + 10, GxEPD_RED);
		display1.fillRect(X + 5, Y - Dp + 2, 7, Dp + 1, GxEPD_RED);
		// display rate of change
		display1.setCursor(56, 38);
		display1.setFont(&Lato_Semibold_14);
		display1.print("Dp/Dt   "); display1.print(Dp, 2); display1.print(" in 3h");
		Serial.println("FallingQuickly");
		break;
	case trend::RisingVeryRapidly:
		display1.fillTriangle(X, Y - Dp - 6, X + 16, Y - Dp - 6, X + 8, Y - Dp - 14, GxEPD_RED);
		display1.fillRect(X + 5, Y - Dp - 6, 7, Dp+6, GxEPD_RED);
		Serial.println("RisingVeryRapidly");
		break;
	case trend::FallingVeryRapidly:
		display1.fillTriangle(X, Y - Dp + 6, X + 16, Y - Dp + 6, X + 8, Y - Dp + 14, GxEPD_RED);
		display1.fillRect(X + 5, Y - Dp + 6, 7, Dp - 4, GxEPD_RED);
		// display warning
		display1.setCursor(36, 18);
		display1.setFont(&Lato_Heavy_16);
		display1.setTextColor(GxEPD_RED);
		display1.print("STORM WARNING");
		// display rate of change
		display1.setCursor(56, 38);
		display1.setFont(&Lato_Semibold_14);
		display1.print("Dp/Dt   "); display1.print(Dp, 2); display1.print(" in 3h");
		Serial.println("FallingVeryRapidly");
		break;
	}
}

void UpdateTextDisplay() {
	display1.fillRect(0, 89, screenX, screenY - 89, GxEPD_WHITE);
	display1.setCursor(1, 102);
	display1.setFont(&Nimbus_Sans_L_Bold_Condensed_15);
	display1.setTextColor(GxEPD_RED);
	display1.setFont(&Lato_Semibold_13);
	display1.print(hPa, 1); display1.print("mb ");
	display1.setTextColor(GxEPD_BLACK);
	display1.print(CtoF(tC), 0); display1.print("*F");
	display1.setFont(&Nimbus_Sans_L_Regular_Condensed_14);
	display1.print(" ");
	display1.print(tC, 1); display1.print("*C ");
	display1.print(RH, 0); display1.print("% dp");
	display1.print(CtoF(dewpoint()), 0); display1.print("*F");

	display1.setCursor(4,8);
	display1.setFont(&Org_01);
	display1.setTextColor(GxEPD_BLACK);
	display1.print(vBatt, 2); display1.print("V");

}

//calculate Dew Point 
double dewpoint() {
	// (B1*(ln(RH/100) + (A1*t)/(B1 +t)))/(A1-ln(RH/100)-A1*t/(B1+t))
	const double b1 = 243.04;
	const double a1 = 17.625;
	double c = log(RH/100) + ((a1 * tC) / (b1 + tC));
	double d = (b1 * c) / (a1 - c);
	return d;
}

// convert Celcius to Farenheight
double CtoF(double t) {
	return (t * 9) / 5 + 32;
}

void printData() {
	Serial.print(F("Pressure: "));
	Serial.print(hPa, 1);
	Serial.print(" hPa ");

	Serial.print(F("Dp/Dt: "));
	Serial.print(Dp, 1);
	Serial.print("hPa ");

	Serial.print(F("Temp: "));
	Serial.print(tC, 2);
	Serial.print("*C ");
	Serial.print(CtoF(tC), 2);
	Serial.print("*F ");

	Serial.print(F("Humidity: "));
	Serial.print(RH, 1);
	Serial.print(" % ");

	Serial.print(F("dewpoint: "));
	Serial.print(dewpoint(), 1);
	Serial.print("*C");
	Serial.print(" Battery Voltage = "); Serial.print(vBatt, 3); Serial.println(" V");

}

char getCommand()
{
	char c = '\0';
	if (Serial.available())
	{
		c = Serial.read();
	}
	return c;
}


// LED blinker
// count flashes in duration ms
void Blink(int count, unsigned long duration)
{
	unsigned long d = duration / count;
	for (int counter = 0; counter < count; counter++) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(d / 2);
		digitalWrite(LED_BUILTIN, LOW);
		delay(d / 2);
	}
}

