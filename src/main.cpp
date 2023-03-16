// #include <Arduino.h>
// // untuk pergantian device - ganti ID>localAddress>NextAddress
#include <SPI.h>
#include <LoRa.h>
#include <JY901.h>
#include <axp20x.h>
#include <TinyGPS++.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <ArduinoEigenDense.h>
#include "sensor/GPS.h"
#include <iomanip>
#include "math/Optimizer.h"
#include <iostream>
#include <fstream>
#include <cassert>
#include <string>
// #include "test/utils/DataFormat.h"
#include "test/TestLocation.h"
#include "test/TestCalibration.h"
#include "config/Config.h"
#include "location/Location.h"
#include "models/XgboostDetector.h"
#include "math/Quaternions.h"
#include <chrono>
#include "test/TestXgboostDetector.h"

using namespace Eigen;
using namespace std;

Location location;

TaskHandle_t Task1;

#define SS 18   // LoRa radio chip select
#define RST 14  // LoRa radio reset
#define DIO0 26 // change for your board; must be a hardware interrupt pin
#define SCK 5
#define MISO 19
#define MOSI 27
#define BUTTONYELLOW 14
#define BUTTONGREEN 25

const uint8_t i2c_sda = 21;
const uint8_t i2c_scl = 22;
const uint8_t slave_address = AXP192_SLAVE_ADDRESS;

String outgoing; // outgoing message
String ID = "MKRRMP011222";
String ROMUSAGE = "244";
String PayLoad, Latitude, Longitude, Altitude, SOG, COG, VBAT, STAT;
String AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, roll, pitch, yaw;
String RSI = "NAN";
String SNR = "NAN";

byte msgCount = 0;       // count of outgoing messages
byte NextAddress = 0x13; // address of this device

long lastSendTime = 0; // last send time

int localAddress = 1; // address of this device
int interval = 50;    // interval between sends
int counter;
int destination = 0; // destination to send to
int BTNG, BTNY;
bool state, state2;

TinyGPSPlus gps;
AXP20X_Class axp;
HardwareSerial GPS2(1);
Adafruit_SSD1306 display(128, 64, &Wire);

void sendMessage(String outgoing);
void sendMessageNext(String outgoing);
void onReceive(int packetSize);
void Task1code(void *parameter);

void setup()
{
    Serial.begin(9600);

    xTaskCreatePinnedToCore(
        Task1code, /* Function to implement the task */
        "Task1",   /* Name of the task */
        10000,     /* Stack size in words */
        NULL,      /* Task input parameter */
        0,         /* Priority of the task */
        &Task1,    /* Task handle. */
        0);

    SPI.begin(SCK, MISO, MOSI, SS);
    LoRa.setPins(SS, RST, DIO0);
    if (!LoRa.begin(915E6))
    { // initialize ratio at 915 MHz
        Serial.println("LoRa init failed. Check your connections.");
    }

    GPS2.begin(9600, SERIAL_8N1, 34, 12);

    Wire.begin(i2c_sda, i2c_scl);

    JY901.startIIC();

    int ret = axp.begin(Wire, slave_address);
    if (ret)
    {
        Serial.println("Ooops, AXP202/AXP192 power chip detected ... Check your wiring!");
    }
    axp.adc1Enable(AXP202_VBUS_VOL_ADC1 |
                       AXP202_VBUS_CUR_ADC1 |
                       AXP202_BATT_CUR_ADC1 |
                       AXP202_BATT_VOL_ADC1,
                   true);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    { // Address 0x3D for 128x64
        Serial.println(F("SSD1306 allocation failed 1"));
        if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D))
        {
            Serial.println(F("SSD1306 allocation failed 2"));
        }
    }
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    int i = 0;
    while (i < 3)
    {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("\n   START");
        display.println(String() + "     " + (i + 1));
        Serial.println(i);
        display.display();
        delay(1000);
        i++;
    }
    pinMode(BUTTONYELLOW, INPUT_PULLUP);
    pinMode(BUTTONGREEN, INPUT_PULLUP);
}

void loop()
{
    BTNY = digitalRead(BUTTONYELLOW);
    BTNG = digitalRead(BUTTONGREEN);
    if (BTNY == 0)
    {
        ESP.restart();
    }
    if (millis() - lastSendTime > interval)
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        if (axp.isBatteryConnect())
        {
            VBAT = axp.getBattVoltage();
            state2 = false;
        }
        else
        {
            VBAT = "NAN";
            state2 = true;
        }
        if (gps.location.isValid())
        {
            Latitude = String(location.GetGNSSINS().lat, 9);
            Longitude = String(location.GetGNSSINS().lng, 9);
            SOG = String(gps.speed.kmph());
            COG = String(gps.course.deg());
        }
        else
        {
            Latitude = "NAN";
            Longitude = "NAN";
            SOG = "NAN";
            COG = "NAN";
        }
        Altitude = JY901.getAltitude() / 100.0;
        AccX = JY901.getAccX();
        AccY = JY901.getAccY();
        AccZ = JY901.getAccZ();
        GyroX = JY901.getGyroX();
        GyroY = JY901.getGyroY();
        GyroZ = JY901.getGyroZ();
        MagX = JY901.getMagX();
        MagY = JY901.getMagY();
        MagZ = JY901.getMagZ();
        roll = JY901.getRoll();
        pitch = JY901.getPitch();
        yaw = JY901.getYaw();
        PayLoad = String() + ID + "," + Latitude + "," + Longitude + "," + Altitude + "," + SOG + "," + COG + "," + AccX + "," + AccY + "," + AccZ + "," + GyroX + "," + GyroY + "," + GyroZ + "," + MagX + "," + MagY + "," + MagZ + "," + roll + "," + pitch + "," + yaw + "," + VBAT + "," + RSI + "," + SNR + ",*";
        display.println(String() + "   " + ID);
        display.println(String() + "LAT : " + Latitude);
        display.println(String() + "LON : " + Longitude);
        if (state == false)
        {
            display.println(String() + "STAT: NOT CONNECT");
            display.println(String() + "RSSI: ");
        }
        else
        {
            display.println(String() + "STAT: " + STAT);
            display.println(String() + "RSSI: " + RSI);
        }
        if (state2 == false)
        {
            display.println(String() + "VBAT: " + VBAT + "mV");
        }
        else
        {
            display.println(String() + "VBAT: NO BATTERY");
        }
        //    sendMessage(PayLoad);
        //    Serial.println("Sending " + PayLoad);
        //    Serial.println(String()+ "FREE RAM : " + ESP.getFreeHeap() + "\tRAM SIZE : " +  ESP.getHeapSize() + "\t USED RAM : " + (ESP.getMaxAllocHeap()/1000));
        display.println(String() + "   USED       USED");
        display.println(String() + "RAM:" + (ESP.getMaxAllocHeap() / 1000) + "kb  ROM:" + ROMUSAGE + "kb");
        display.display();
        lastSendTime = millis(); // timestamp the message
        interval = 1000;
    }
    else
    {
        while (GPS2.available())
        {
            gps.encode(GPS2.read());
        }
    }
    if (millis() - lastSendTime > 1000)
    {
        counter++;
        if (counter >= 5)
        {
            state = false;
        }
        lastSendTime = millis(); // timestamp the message
    }
    onReceive(LoRa.parsePacket());
}

void Task1code(void *parameter)
{
    Serial.printf("test");
    Vector3d gyro_data_v(JY901.getGyroX(), JY901.getGyroY(), JY901.getGyroZ());
    Vector3d mag_data_v(JY901.getMagX(), JY901.getMagY(), JY901.getMagZ());
    Vector3d acc_data_v(JY901.getAccX(), JY901.getAccX(), JY901.getAccX());
    VectorXd gps_data_v(7);
    gps_data_v << gps.location.lng(), gps.location.lat(), JY901.getAltitude(), 0.0, gps.speed.value(), 0.0, millis();
    Vector3d g_data_v(0.094139, 0.107857, 9.808955);
    Vector3d ornt_data_v(JY901.getRoll(), JY901.getPitch(), JY901.getRoll());
    Vector3d road_data(0.0, 0.0, 0);
    location.PredictCurrentPosition(gyro_data_v, acc_data_v, mag_data_v, gps_data_v, g_data_v, ornt_data_v, road_data);
    for (;;)
    {
        Serial.print("Current predict result: lng ");
        Serial.println(location.GetGNSSINS().lng);
        Serial.print(", lat ");
        Serial.println(location.GetGNSSINS().lat);
    }
}

void sendMessage(String outgoing)
{
    LoRa.beginPacket();            // start packet
    LoRa.write(destination);       // add destination address
    LoRa.write(localAddress);      // add sender address
    LoRa.write(msgCount);          // add message ID
    LoRa.write(outgoing.length()); // add payload length
    LoRa.print(outgoing);          // add payload
    LoRa.endPacket();              // finish packet and send it
    msgCount++;                    // increment message ID
}
void sendMessageNext(String outgoing)
{
    LoRa.beginPacket();            // start packet
    LoRa.write(NextAddress);       // add destination address
    LoRa.write(localAddress);      // add sender address
    LoRa.write(msgCount);          // add message ID
    LoRa.write(outgoing.length()); // add payload length
    LoRa.print(outgoing);          // add payload
    LoRa.endPacket();              // finish packet and send it
    msgCount++;                    // increment message ID
}

void onReceive(int packetSize)
{
    if (packetSize == 0)
        return; // if there's no packet, return

    // read packet header bytes:
    int recipient = LoRa.read();       // recipient address
    byte sender = LoRa.read();         // sender address
    byte incomingMsgId = LoRa.read();  // incoming msg ID
    byte incomingLength = LoRa.read(); // incoming msg length

    String incoming = "";

    // if the recipient isn't this device or broadcast,
    if (recipient != localAddress && recipient != 0x10)
    {
        return; // skip rest of function
    }
    else if (recipient == localAddress)
    {
        STAT = String() + "CONNECTED";
        sendMessage(PayLoad);
        counter = 0;
        state = true;
    }
    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }

    if (incomingLength != incoming.length())
    { // check length for error
        Serial.println("error: message length does not match length");
        return; // skip rest of function
    }

    //  Serial.println(incoming);
    RSI = String(LoRa.packetRssi());
    SNR = String(LoRa.packetSnr());
}
