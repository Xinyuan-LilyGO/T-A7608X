#define TINY_GSM_MODEM_SIM7600  //A7608 Compatible with SIM7600 AT instructions
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// See all AT commands, if wanted
#define DUMP_AT_COMMANDS

/*
   Tests enabled
*/
#define TINY_GSM_TEST_GPRS    true
#define TINY_GSM_TEST_GPS     true
#define TINY_GSM_POWERDOWN    true

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]  = "YOUR-APN";     //SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";

// Server details to test TCP/SSL
const char server[] = "vsh.pp.ua";
const char resource[] = "/TinyGSM/logo.txt";


#include <TinyGsmClient.h>
#include <SPI.h>
#include <SD.h>
#include <Ticker.h>

#ifdef DUMP_AT_COMMANDS  // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  60          // Time ESP32 will go to sleep (in seconds)

#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_TX      26
#define PIN_RX      27
#define PWR_PIN     4
#define BAT_ADC     35
#define BAT_EN      12
#define  PIN_RI     33
#define  PIN_DTR    25
#define  RESET      5

#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13





void setup()
{
    // Set console baud rate
    SerialMon.begin(UART_BAUD);
    delay(10);

    SerialMon.println("setup...");

// BAT EN
    pinMode(BAT_EN, OUTPUT);
    digitalWrite(BAT_EN, HIGH);

//A7608 Reset
    pinMode(RESET, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(100);
    digitalWrite(RESET, HIGH);
    delay(1000);
    digitalWrite(RESET, LOW);

//A7608 Power on
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(100);
    digitalWrite(PWR_PIN, HIGH);
    delay(1000);
    digitalWrite(PWR_PIN, LOW);


    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS)) {
        SerialMon.println("SDCard MOUNT FAIL");
    } else {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        String str = "SDCard Size: " + String(cardSize) + "MB";
        SerialMon.println(str);
    }

    SerialMon.println("\nWait...");

    delay(1000);

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.println("Initializing modem...");
    if (!modem.init()) {
        SerialMon.println("Failed to restart modem, attempting to continue without restarting");
    }




    modem.sendAT(GF("+CGDRT=5,1"));//GPIO5 output
    if (modem.waitResponse(10000L) != 1) {
        return ;
    }

    modem.sendAT(GF("+CGSETV=5,1"));// Power off the GNSS_1V8
    if (modem.waitResponse(10000L) != 1) {
        return ;
    }

}

void loop()
{



    /*
         while (1) {
             while (SerialMon.available()) {
                 SerialAT.write(SerialMon.read());
             }
             while (SerialAT.available()) {
                 SerialMon.write(SerialAT.read());
             }
         }
    */

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.println("Initializing modem...");
    if (!modem.init()) {
        SerialMon.println("Failed to restart modem, attempting to continue without restarting");
    }

    String name = modem.getModemName();
    delay(500);
    SerialMon.println("Modem Name: " + name);

    String modemInfo = modem.getModemInfo();
    delay(500);



#if TINY_GSM_TEST_GPRS
    // Unlock your SIM card with a PIN if needed
    if ( GSM_PIN && modem.getSimStatus() != 3 ) {
        modem.simUnlock(GSM_PIN);
    }
#endif

#if TINY_GSM_TEST_GPRS && defined TINY_GSM_MODEM_XBEE
    // The XBee must run the gprsConnect function BEFORE waiting for network!
    modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

    DBG("Waiting for network...");
    if (!modem.waitForNetwork()) {
        delay(10000);
        return;
    }

    if (modem.isNetworkConnected()) {
        DBG("Network connected");
    }


#if TINY_GSM_TEST_GPRS
    DBG("Connecting to", apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        delay(10000);
        return;
    }

    bool res = modem.isGprsConnected();
    DBG("GPRS status:", res ? "connected" : "not connected");

    String ccid = modem.getSimCCID();
    DBG("CCID:", ccid);

    String imei = modem.getIMEI();
    DBG("IMEI:", imei);

    String cop = modem.getOperator();
    DBG("Operator:", cop);

    IPAddress local = modem.localIP();
    DBG("Local IP:", local);

    int csq = modem.getSignalQuality();
    DBG("Signal quality:", csq);

#endif


#if TINY_GSM_TEST_GPRS
    modem.gprsDisconnect();
    if (!modem.isGprsConnected()) {
        DBG("GPRS disconnected");
    } else {
        DBG("GPRS disconnect: Failed.");
    }
#endif

#if TINY_GSM_TEST_GPS

    modem.sendAT(GF("+CGSETV=5,0"));// on the GNSS_1V8
    if (modem.waitResponse(10000L) != 1) {
        while (1) {
            DBG("+CGDRT=5,0 FALL");
            delay(1000);
        }
    }

    modem.enableGPS();

    modem.sendAT("+CGNSSPWR=1");
    if (modem.waitResponse(10000L) != 1) {
        while (1) {
            delay(1000);
            DBG("EnableGPS  false");
        }

    }

    float lat,  lon;
    while (1) {
        if (modem.getGPS(&lat, &lon)) {
            SerialMon.printf("lat:%f lon:%f\n", lat, lon);

            break;
        } else {
            SerialMon.print("getGPS ");
            SerialMon.println(millis());
        }
        delay(2000);
    }
    modem.disableGPS();
#endif


#if TINY_GSM_POWERDOWN
    // Try to power-off (modem may decide to restart automatically)
    // To turn off modem completely, please use Reset/Enable pins
    modem.poweroff();
    DBG("Poweroff.");
#endif

    // Test is complete Set it to sleep mode
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    delay(200);
    esp_deep_sleep_start();

    // Do nothing forevermore
    while (true) {
        modem.maintain();
    }
}
