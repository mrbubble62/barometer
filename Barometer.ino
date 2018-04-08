#define ARDUINO_ARCH_ESP32

#include "NMEA2000_CAN.h"
#include "N2kMessages.h"
#include <SparkFunBME280.h>

BME280  bmp;

#define EEPROM_ADR_CONFIG 0 // eeprom address for config params
#define P0 1005.3
bool SerialDebug = false;
#define stbyPin 14

int SID = 1;

tN2kMsg N2kMsg;
int beginStatus;
double T, P, H;

float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval


long slowloop = 0;
long veryslowloop = 0;
const unsigned long TransmitMessagesEnv[] PROGMEM = { 130310L, 130311L,0 }; //Atmospheric Pressure, Temperature

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(stbyPin, OUTPUT);
	digitalWrite(stbyPin, LOW);

	Blink(5, 300);
	Serial.begin(115200);
	delay(100);
	Serial.println("Barometer");
	delay(100);
	
	Serial.println("BMP Start");
	bmp.settings.commInterface = I2C_MODE;
	bmp.settings.I2CAddress = 0x76;
	bmp.settings.runMode = 3;
	bmp.settings.filter = 0;
	bmp.settings.tempOverSample = 1;
	bmp.settings.pressOverSample = 1;
	//bmp.settings.disableHumidity = true;
	if (!bmp.begin()) {
		Serial.println(F("BMP init failed!"));
		while (1);
	}
	else Serial.println(F("BMP init success!"));
	delay(50);

	NMEA2000.SetInstallationDescription1("Mr Bubble Barometer");
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
	delay(100);
	Serial.println(F("Starting\n\n"));
}
bool SDEBUG = false; // debug print for spreadsheet
bool SPRINT = false; // send test output

void loop() {
	char command = getCommand();
	switch (command)
	{
	case 'h': //toggle debug
		SDEBUG = !SDEBUG;
		delay(100);
		break;
	case 's':
		SPRINT = !SPRINT;
		break;

	default:
		break;
	}

	// display at 0.25s rate independent of data rates
	delt_t = millis() - count;
	if (delt_t > 250) { // update once per half-second independent of read rate
		Now = micros();
		deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
		lastUpdate = Now;
		count = millis();

		slowloop++;
	}
	//1 s
	if (slowloop > 4) { slowloop = 0; SlowLoop(); }
	NMEA2000.ParseMessages();
}


void SlowLoop()
{
	SID++; if (SID > 254) { SID = 1; }
	printData();
	// Send Temperature & Pressure
	T = bmp.readTempC();
	P = bmp.readFloatPressure();
	H = bmp.readFloatHumidity();
	SetN2kEnvironmentalParameters(N2kMsg, SID, N2kts_OutsideTemperature, CToKelvin(T), N2khs_InsideHumidity, H, P);
	NMEA2000.SendMsg(N2kMsg);
}


void printData() {
	Serial.println("");
	Serial.print(F("Pressure: "));
	Serial.print(P / 100, 1);
	Serial.println(" P");

	Serial.print(F("Temp: "));
	Serial.print(T, 2);
	Serial.println(" *C");
	
	Serial.print(F("Humidity: "));
	Serial.print(H, 1);
	Serial.println(" %");

	if (SerialDebug) {
		Serial.print("rate = "); Serial.print((float)sumCount / sum, 2); Serial.println(" Hz");
	}
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

