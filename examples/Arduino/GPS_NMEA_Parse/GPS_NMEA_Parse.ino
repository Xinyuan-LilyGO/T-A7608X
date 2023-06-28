/**
 * @file      GPS_NMEA_Parse.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-06-28
 *
 */

#define UART_BAUD           115200
#define MODEM_DTR_PIN             25
#define PIN_TX              26
#define PIN_RX              27
#define MODEM_PWR_PIN             4
#define BAT_ADC             35
#define BOARD_POWER_ON              12
#define PIN_RI              33
#define MODEM_RST_PIN               5
#define SD_MISO             2
#define SD_MOSI             15
#define SD_SCLK             14
#define SD_CS               13

#define TINY_GSM_MODEM_SIM7600  //A7608's AT instruction is compatible with SIM7600
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>
#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// The TinyGPSPlus object
TinyGPSPlus gps;


void displayInfo();

void setup()
{
    Serial.begin(115200);
    SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);

    // Board Power control pin
    // When using battery power, this pin must be set to HIGH
    pinMode(BOARD_POWER_ON, OUTPUT);
    digitalWrite(BOARD_POWER_ON, HIGH);

    //Modem wakeup pin
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, LOW);

    //modem reset pin
    pinMode(MODEM_RST_PIN, OUTPUT);
    digitalWrite(MODEM_RST_PIN, HIGH);

    //modem power key pin
    pinMode(MODEM_PWR_PIN, OUTPUT);
    digitalWrite(MODEM_PWR_PIN, LOW);
    delay(100);
    digitalWrite(MODEM_PWR_PIN, HIGH);
    delay(1000);    //Ton = 1000ms ,Min = 500ms, Max 2000ms
    digitalWrite(MODEM_PWR_PIN, LOW);


    Serial.print("Modem starting...");
    int retry = 0;
    while (!modem.testAT(1000)) {
        Serial.println(".");
        if (retry++ > 10) {
            digitalWrite(MODEM_PWR_PIN, LOW);
            delay(100);
            digitalWrite(MODEM_PWR_PIN, HIGH);
            delay(1000);    //Ton = 1000ms ,Min = 500ms, Max 2000ms
            digitalWrite(MODEM_PWR_PIN, LOW);
            retry = 0;
        }
    }
    Serial.println();

    delay(200);

    //START Compatible with previous versions of devices, this command is invalid in the new version, please ignore
    modem.sendAT("+CGDRT=5,1");
    modem.waitResponse(10000L);

    modem.sendAT("+CGSETV=5,0");
    modem.waitResponse(10000L);
    //END  Compatible with previous versions of devices, this command is invalid in the new version, please ignore

    //START In the new version, AUX power supply needs to be turned on, but this command is invalid in the old version
    modem.sendAT("+CVAUXS=1");
    modem.waitResponse(10000L);
    //END In the new version, AUX power supply needs to be turned on, but this command is invalid in the old version

    Serial.print("GPS starting...");
    modem.sendAT("+CGNSSPWR=1");
    while (modem.waitResponse(10000UL, "+CGNSSPWR: READY!") != 1) {
        Serial.print(".");
    }
    Serial.println("GPS Ready!");

    //Configure the baud rate of UART3 and GPS module
    modem.sendAT("+CGNSSIPR=115200");
    modem.waitResponse(1000L);

    //Configure GNSS support mode : BD + GPS
    modem.sendAT("+CGNSSMODE=3");
    modem.waitResponse(1000L);
    // Configure NMEA sentence type
    modem.sendAT("+CGNSSNMEA=1,1,1,1,1,1,0,0");
    modem.waitResponse(1000L);

    // Set NMEA output rate : 1HZ
    modem.sendAT("+CGPSNMEARATE=1");
    modem.waitResponse(1000L);

    // Send data received from UART3 to NMEA port
    modem.sendAT("+CGNSSTST=1");
    modem.waitResponse(1000L);

    // Select the output port for NMEA sentence
    modem.sendAT("+CGNSSPORTSWITCH=0,1");
    modem.waitResponse(1000L);

    Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
    Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
    Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));

}

void loop()
{
    // Simple show loaction
    // displayInfo()

    // Show full information
    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

    printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
    printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    printInt(gps.location.age(), gps.location.isValid(), 5);
    printDateTime(gps.date, gps.time);
    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
    printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
    printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

    unsigned long distanceKmToLondon =
        (unsigned long)TinyGPSPlus::distanceBetween(
            gps.location.lat(),
            gps.location.lng(),
            LONDON_LAT,
            LONDON_LON) / 1000;
    printInt(distanceKmToLondon, gps.location.isValid(), 9);

    double courseToLondon =
        TinyGPSPlus::courseTo(
            gps.location.lat(),
            gps.location.lng(),
            LONDON_LAT,
            LONDON_LON);

    printFloat(courseToLondon, gps.location.isValid(), 7, 2);

    const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

    printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

    printInt(gps.charsProcessed(), true, 6);
    printInt(gps.sentencesWithFix(), true, 10);
    printInt(gps.failedChecksum(), true, 9);
    Serial.println();

    smartDelay(1000);
}

void displayInfo()
{
    Serial.print(F("Location: "));
    if (gps.location.isValid()) {
        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.print(gps.location.lng(), 6);
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.print(F("  Date/Time: "));
    if (gps.date.isValid()) {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.print(F(" "));
    if (gps.time.isValid()) {
        if (gps.time.hour() < 10) Serial.print(F("0"));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        if (gps.time.minute() < 10) Serial.print(F("0"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        if (gps.time.second() < 10) Serial.print(F("0"));
        Serial.print(gps.time.second());
        Serial.print(F("."));
        if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());
    } else {
        Serial.print(F("INVALID"));
    }

    Serial.println();
}


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do {
        while (SerialAT.available())
            gps.encode(SerialAT.read());
    } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
    if (!valid) {
        while (len-- > 1)
            Serial.print('*');
        Serial.print(' ');
    } else {
        Serial.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1); // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
        for (int i = flen; i < len; ++i)
            Serial.print(' ');
    }
    smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
    char sz[32] = "*****************";
    if (valid)
        sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i = strlen(sz); i < len; ++i)
        sz[i] = ' ';
    if (len > 0)
        sz[len - 1] = ' ';
    Serial.print(sz);
    smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
    if (!d.isValid()) {
        Serial.print(F("********** "));
    } else {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
        Serial.print(sz);
    }

    if (!t.isValid()) {
        Serial.print(F("******** "));
    } else {
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
        Serial.print(sz);
    }

    printInt(d.age(), d.isValid(), 5);
    smartDelay(0);
}

static void printStr(const char *str, int len)
{
    int slen = strlen(str);
    for (int i = 0; i < len; ++i)
        Serial.print(i < slen ? str[i] : ' ');
    smartDelay(0);
}
