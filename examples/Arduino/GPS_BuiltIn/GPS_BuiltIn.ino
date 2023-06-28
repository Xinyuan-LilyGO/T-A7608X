/**
 * @file      GPS_BuiltIn.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-06-28
 *
 */

#define UART_BAUD                   115200
#define MODEM_DTR_PIN               25
#define MODEM_TXD_PIN               26
#define MODEM_RXD_PIN               27
#define MODEM_PWR_PIN               4
#define BAT_ADC                     35
#define BOARD_POWER_ON              12
#define MODEM_RI_PIN                33
#define MODEM_RST_PIN               5
#define SD_MISO                     2
#define SD_MOSI                     15
#define SD_SCLK                     14
#define SD_CS                       13

#define TINY_GSM_MODEM_SIM7600  //A7608 Compatible with SIM7600 AT instructions
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

#define SerialAT Serial1
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial


// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS


#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif


bool streamGetLength(char *buf, int8_t numChars,
                     const uint32_t timeout_ms = 1000L)
{
    if (!buf) {
        return false;
    }

    int8_t   numCharsReady = -1;
    uint32_t startMillis   = millis();
    while (millis() - startMillis < timeout_ms &&
            (numCharsReady = modem.stream.available()) < numChars) {
        TINY_GSM_YIELD();
    }

    if (numCharsReady >= numChars) {
        modem.stream.readBytes(buf, numChars);
        return true;
    }

    return false;
}

int16_t streamGetIntLength(int8_t         numChars,
                           const uint32_t timeout_ms = 1000L)
{
    char buf[numChars + 1];
    if (streamGetLength(buf, numChars, timeout_ms)) {
        buf[numChars] = '\0';
        return atoi(buf);
    }

    return -9999;
}

uint64_t streamGetLongLongBefore(char lastChar)
{
    char   buf[12];
    size_t bytesRead = modem.stream.readBytesUntil(
                           lastChar, buf, static_cast<size_t>(12));
    // if we read 12 or more bytes, it's an overflow
    if (bytesRead && bytesRead < 12) {
        buf[bytesRead] = '\0';
        uint64_t  res  = atoll(buf);
        return res;
    }
    return 0;
}

int streamGetIntBefore(char lastChar)
{
    char   buf[7];
    size_t bytesRead = modem.stream.readBytesUntil(
                           lastChar, buf, static_cast<size_t>(7));
    // if we read 7 or more bytes, it's an overflow
    if (bytesRead && bytesRead < 7) {
        buf[bytesRead] = '\0';
        int res    = atoi(buf);
        return res;
    }

    return -9999;
}

float streamGetFloatLength(int8_t         numChars,
                           const uint32_t timeout_ms = 1000L)
{
    char buf[numChars + 1];
    if (streamGetLength(buf, numChars, timeout_ms)) {
        buf[numChars] = '\0';
        return atof(buf);
    }

    return -9999.0F;
}

float streamGetFloatBefore(char lastChar)
{
    char   buf[16];
    size_t bytesRead = modem.stream.readBytesUntil(
                           lastChar, buf, static_cast<size_t>(16));
    // if we read 16 or more bytes, it's an overflow
    if (bytesRead && bytesRead < 16) {
        buf[bytesRead] = '\0';
        float res      = atof(buf);
        return res;
    }

    return -9999.0F;
}

bool streamSkipUntil(const char c, const uint32_t timeout_ms = 1000L)
{
    uint32_t startMillis = millis();
    while (millis() - startMillis < timeout_ms) {
        while (millis() - startMillis < timeout_ms &&
                !modem.stream.available()) {
            TINY_GSM_YIELD();
        }
        if (modem.stream.read() == c) {
            return true;
        }
    }
    return false;
}

void getGNSS()
{
    modem.sendAT(GF("+CGNSSINFO"));
    if (modem.waitResponse(GF(GSM_NL "+CGNSSINFO: ")) != 1) {
        return ;
    }

    uint8_t fixMode = streamGetIntBefore(',');  // mode 2=2D Fix or 3=3DFix
    // TODO(?) Can 1 be returned
    if (fixMode == 1 || fixMode == 2 || fixMode == 3) {
        // init variables
        float ilat = 0;
        char  north;
        float ilon = 0;
        char  east;
        float ispeed       = 0;
        float ialt         = 0;
        int   ivsat        = 0;
        int   iusat        = 0;
        float iaccuracy    = 0;
        int   iyear        = 0;
        int   imonth       = 0;
        int   iday         = 0;
        int   ihour        = 0;
        int   imin         = 0;
        float secondWithSS = 0;

        streamSkipUntil(',');               // GPS satellite valid numbers
        streamSkipUntil(',');               // GLONASS satellite valid numbers
        //! This bit in the A7608 manual should represent BEIDOU satellite valid numbers,
        //! but it does not.
        // streamSkipUntil(',');               // BEIDOU satellite valid numbers
        ilat  = streamGetFloatBefore(',');  // Latitude in ddmm.mmmmmm
        north = modem.stream.read();              // N/S Indicator, N=north or S=south
        streamSkipUntil(',');
        ilon = streamGetFloatBefore(',');  // Longitude in ddmm.mmmmmm
        east = modem.stream.read();              // E/W Indicator, E=east or W=west
        streamSkipUntil(',');

        // Date. Output format is ddmmyy
        iday   = streamGetIntLength(2);    // Two digit day
        imonth = streamGetIntLength(2);    // Two digit month
        iyear  = streamGetIntBefore(',');  // Two digit year

        // UTC Time. Output format is hhmmss.s
        ihour = streamGetIntLength(2);  // Two digit hour
        imin  = streamGetIntLength(2);  // Two digit minute
        secondWithSS = streamGetFloatBefore(',');  // 4 digit second with subseconds

        ialt   = streamGetFloatBefore(',');  // MSL Altitude. Unit is meters
        ispeed = streamGetFloatBefore(',');  // Speed Over Ground. Unit is knots.
        streamSkipUntil(',');                // Course Over Ground. Degrees.
        streamSkipUntil(',');  // After set, will report GPS every x seconds
        iaccuracy = streamGetFloatBefore(',');  // Position Dilution Of Precision
        streamSkipUntil(',');   // Horizontal Dilution Of Precision
        streamSkipUntil(',');   // Vertical Dilution Of Precision
        streamSkipUntil('\n');  // TODO(?) is one more field reported??


        DBG("Latitude:", ilat, "\tLongitude:", ilon);
        DBG("Speed:", ispeed, "\tAltitude:", ialt);
        DBG("Visible Satellites:", ivsat, "\tUsed Satellites:", iusat);
        DBG("Accuracy:", iaccuracy);
        DBG("Year:", iyear, "\tMonth:", imonth, "\tDay:", iday);
        DBG("Hour:", ihour, "\tMinute:", imin, "\tSecond:", static_cast<int>(secondWithSS));

        modem.waitResponse();
        // Sometimes, although fix is displayed,
        // the value of longitude and latitude 0 will be set as invalid
        if (ilat == 0 || ilon == 0) {
            return ;
        }
        return ;
    }
    modem.waitResponse();
}


void setup()
{
    Serial.begin(115200);
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RXD_PIN, MODEM_TXD_PIN);

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
        while (SerialAT.available()) {
            Serial.write(SerialAT.read());
        }
        while (Serial.available()) {
            SerialAT.write(Serial.read());
        }
    }
    Serial.println("GPS Ready!");

}

void loop()
{
    getGNSS();
    delay(1000);
}

