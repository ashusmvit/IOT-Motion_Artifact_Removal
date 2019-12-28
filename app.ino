#include <SparkFunLSM6DS3.h>



// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// Please use an Arduino IDE 1.6.8 or greater

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>

#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"




#include <AzureIoTHub.h>
#include <AzureIoTProtocol_MQTT.h>
#include <AzureIoTUtility.h>

#include "config.h"

static bool messagePending = false;
static bool messageSending = true;

static char *connectionString="HostName=iot-244p-demo6.azure-devices.net;DeviceId=samaks-sparkfun-devthing;SharedAccessKey=Y+97txEwPD5/yfCtwxo0tcTage3fKzFuyl4/BejrNP4=";
static char *ssid="ashu";
static char *pass="1234567890";
static int interval = INTERVAL;

// -- constants for project
#define GAIN   5.249213146e+00

float xv[9], yv[9], xv_x[9], yv_x[9],xv_y[9], yv_y[9],xv_z[9], yv_z[9];

MAX30105 particleSensor;
LSM6DS3 myIMU; //Default constructor is I2C, addr 0x6B

float beta = 2E-10;    //rate of convergence
const int N = 4;     // # of weights (coefﬁcients)
 
float w_x[N];         //buffer weights of adapt ﬁlter
float delay_x[N];   //input buffer to adapt ﬁlter
float w_y[N];         //buffer weights of adapt ﬁlter
float delay_y[N];   //input buffer to adapt ﬁlter
float w_z[N];         //buffer weights of adapt ﬁlter
float delay_z[N];   //input buffer to adapt ﬁlter
float buffer_ir[100];
float buffer_ret[100];
int buffer_index = 0;
int queue_count = 0;
// -- end-------
void blinkLED()
{
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
}

void initWifi()
{
    // Attempt to connect to Wifi network:
    Serial.printf("Attempting to connect to SSID: %s.\r\n", ssid);

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED)
    {
        // Get Mac Address and show it.
        // WiFi.macAddress(mac) save the mac address into a six length array, but the endian may be different. The huzzah board should
        // start from mac[0] to mac[5], but some other kinds of board run in the oppsite direction.
        uint8_t mac[6];
        WiFi.macAddress(mac);
        Serial.printf("You device with MAC address %02x:%02x:%02x:%02x:%02x:%02x connects to %s failed! Waiting 10 seconds to retry.\r\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], ssid);
        WiFi.begin(ssid, pass);
        delay(10000);
    }
    Serial.printf("Connected to wifi %s.\r\n", ssid);
}

void initTime()
{
    time_t epochTime;
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    while (true)
    {
        epochTime = time(NULL);

        if (epochTime == 0)
        {
            Serial.println("Fetching NTP epoch time failed! Waiting 2 seconds to retry.");
            delay(2000);
        }
        else
        {
            Serial.printf("Fetched NTP epoch time is: %lu.\r\n", epochTime);
            break;
        }
    }
}


float BandPassFilter_irValue(float data) {
for(short i = 0; i < 8; i++) {
xv[i] = xv[i+1];
yv[i] = yv[i+1];
}

    xv[8] = data/GAIN;
    yv[8] = (xv[0] + xv[8]) - 4 * (xv[2] + xv[6]) + 6 * xv[4]
            + ( -0.0377338824 * yv[0]) + ( -0.0593309765 * yv[1])
            + (  0.1181030334 * yv[2]) + (  0.5649892679 * yv[3])
            + ( -0.6871632434 * yv[4]) + ( -0.2166971678 * yv[5])
            + ( -0.6622650678 * yv[6]) + (  1.9712252877 * yv[7]);
     return yv[8];
 }

float BandPassFilter_xAxis(float data) {
for(short i = 0; i < 8; i++) {
xv_x[i] = xv_x[i+1];
yv_x[i] = yv_x[i+1];
}

    xv_x[8] = data/GAIN;
    yv_x[8] = (xv_x[0] + xv_x[8]) - 4 * (xv_x[2] + xv_x[6]) + 6 * xv_x[4]
            + ( -0.0377338824 * yv_x[0]) + ( -0.0593309765 * yv_x[1])
            + (  0.1181030334 * yv_x[2]) + (  0.5649892679 * yv_x[3])
            + ( -0.6871632434 * yv_x[4]) + ( -0.2166971678 * yv_x[5])
            + ( -0.6622650678 * yv_x[6]) + (  1.9712252877 * yv_x[7]);
     return yv_x[8];
 }

 float BandPassFilter_yAxis(float data) {
for(short i = 0; i < 8; i++) {
xv_y[i] = xv_y[i+1];
yv_y[i] = yv_y[i+1];
}

    xv_y[8] = data/GAIN;
    yv_y[8] = (xv_y[0] + xv_y[8]) - 4 * (xv_y[2] + xv_y[6]) + 6 * xv_y[4]
            + ( -0.0377338824 * yv_y[0]) + ( -0.0593309765 * yv_y[1])
            + (  0.1181030334 * yv_y[2]) + (  0.5649892679 * yv_y[3])
            + ( -0.6871632434 * yv_y[4]) + ( -0.2166971678 * yv_y[5])
            + ( -0.6622650678 * yv_y[6]) + (  1.9712252877 * yv_y[7]);
     return yv_y[8];
 }

 float BandPassFilter_zAxis(float data) {
for(short i = 0; i < 8; i++) {
xv_z[i] = xv_z[i+1];
yv_z[i] = yv_z[i+1];
}

    xv_z[8] = data/GAIN;
    yv_z[8] = (xv_z[0] + xv_z[8]) - 4 * (xv_z[2] + xv_z[6]) + 6 * xv_z[4]
            + ( -0.0377338824 * yv_z[0]) + ( -0.0593309765 * yv_z[1])
            + (  0.1181030334 * yv_z[2]) + (  0.5649892679 * yv_z[3])
            + ( -0.6871632434 * yv_z[4]) + ( -0.2166971678 * yv_z[5])
            + ( -0.6622650678 * yv_z[6]) + (  1.9712252877 * yv_z[7]);
     return yv_z[8]; 
}

float LMS_x(float x,float ir)
{
  delay_x[0] = x;
  float yn = 0;                      //initial output of adapt ﬁlter
  for(int i=0;i<N;i++)
   yn += (w_x[0] * delay_x[0]); //output of adaptive ﬁlter
 
  float E = ir - yn;       //”error” signal=(d+n)-yn
 
  for (int i = N-1; i >= 0; i--)  //to update weights and delays
  {
      w_x[i] = w_x[i] + beta*E*delay_x[i]; //update weights
      if(i != 0)
        delay_x[i] = delay_x[i-1];       //update delay samples
  }
  return E;
}

float LMS_y(float y,float ir)
{
  delay_y[0] = y;
  float yn = 0;                      //initial output of adapt ﬁlter
  for(int i=0;i<N;i++)
   yn += (w_y[0] * delay_y[0]); //output of adaptive ﬁlter
 
  float E = ir - yn;       //”error” signal=(d+n)-yn
 
  for (int i = N-1; i >= 0; i--)  //to update weights and delays
  {
      w_y[i] = w_y[i] + beta*E*delay_y[i]; //update weights
      if(i != 0)
        delay_y[i] = delay_y[i-1];       //update delay samples
  }

  return E;
}

float LMS_z(float z,float ir)
{
  delay_z[0] = z;
  float yn = 0;                      //initial output of adapt ﬁlter
  for(int i=0;i<N;i++)
   yn += (w_z[0] * delay_z[0]); //output of adaptive ﬁlter
 
  float E = ir - yn;       //”error” signal=(d+n)-yn
 
  for (int i = N-1; i >= 0; i--)  //to update weights and delays
  {
      w_z[i] = w_z[i] + beta*E*delay_z[i]; //update weights
      if(i != 0)
        delay_z[i] = delay_z[i-1];       //update delay samples
  }

  return E;
}

float LMS(float irValue,float x, float y,float z)
{
  float E;
  E = LMS_x(x,irValue);
  E = LMS_x(y,E);
  E = LMS_z(z,E);

   return E;
}

void init_LMS() {

  short T = 0;
  for (T = 0; T < N; T++)
  {
    w_x[T] = 0;       //init buffer for weights
    delay_x[T] = 0; //init buffer for delay samples
    w_y[T] = 0;       //init buffer for weights
    delay_y[T] = 0; //init buffer for delay samples
    w_z[T] = 0;       //init buffer for weights
    delay_z[T] = 0; //init buffer for delay samples
  }
  return;
}

float mini(float a,float b){
  if(a<b){
    return a;
  } else {
    return b;
  }
}

double mini(double a,double b){
  if(a<b){
    return a;
  } else {
    return b;
  }
}

float maxx(float a,float b){
  if(a>b){
    return a;
  } else {
    return b;
  }
}

double maxx(double a,double b){
  if(a>b){
    return a;
  } else {
    return b;
  }
}

static IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle;
float minRet = 2000000;
float maxRet = 0;
float minIr = 2000000;
float maxIr = 0;
float irRangDelta = maxIr - minIr;
float retRangeDelta = maxRet - minRet;
long avg = 0;
const int SECONDS = 1000;
long windowLength = 15*SECONDS;
void calibrateSensorData() {
  Serial.println("in calib");
  boolean bufferCapture = true;
  long startTime = millis();
  long c = 0;
  long csum = 0;
  int i = 0;
  while(i<500){
    Serial.println("calibrating......");
    float ret;
    long irValue = particleSensor.getIR();
    float ans = BandPassFilter_irValue(irValue);
    float acc_x = (float)myIMU.readFloatAccelX();
    float acc_y = (float)myIMU.readFloatAccelY();
    float acc_z = (float)myIMU.readFloatAccelZ();
    
    if (irValue > 105000 && irValue < 120000) {
      if(c<10){
        csum += irValue;
        c++;
        Serial.print("csum=");
        Serial.print(csum);
        Serial.print("c=");
        Serial.print(c);
        Serial.println();
        
      } else{
        if(avg > 0) {
          avg = long((avg + long(csum/10))/2);
          Serial.print("avg=");
          Serial.print(avg);
          
        } else {
          avg = long(csum/10);
          Serial.print("first_avg=");
          Serial.print(avg);
        }
        Serial.println();
        c = 0;
        csum = 0;
      }
      ret = LMS(ans,acc_x,acc_y,acc_z);
      minRet = mini(minRet,ret);
      maxRet = maxx(maxRet, ret);
      minIr = mini(minIr,irValue);
      maxIr = maxx(maxIr,irValue);
    } else {
      Serial.println("Please keep your finger on the device to aid calibration");
    }
    i++;
  }
  Serial.println("outside calib");
  irRangDelta = maxIr - minIr;
  retRangeDelta = maxRet - minRet;
}
void setup()
{
    initSerial();
    delay(2000);
    //readCredentials();
    Serial.println("In setup");
    initWifi();
    initTime();
    
    /*
    * Break changes in version 1.0.34: AzureIoTHub library removed AzureIoTClient class.
    * So we remove the code below to avoid compile error.
    */
    // initIoThubClient();
    buffer_index = 0;

    myIMU.begin();
  if( myIMU.beginCore() != 0 )
  {
    Serial.print("Error at beginCore().\n");
  }
  else
  {
    Serial.print("\nbeginCore() passed.\n");
  }

    iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(connectionString, MQTT_Protocol);
    if (iotHubClientHandle == NULL)
    {
        Serial.println("Failed on IoTHubClient_CreateFromConnectionString.");
        while (1);
    }

    IoTHubClient_LL_SetMessageCallback(iotHubClientHandle, receiveMessageCallback, NULL);
    IoTHubClient_LL_SetDeviceMethodCallback(iotHubClientHandle, deviceMethodCallback, NULL);
    IoTHubClient_LL_SetDeviceTwinCallback(iotHubClientHandle, twinCallback, NULL);

  // Done

 //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  init_LMS();
  long irValue = particleSensor.getIR();
  while(irValue < 60000) {
      Serial.println("please keep your finger");
  }
  calibrateSensorData();
}

static int messageCount = 1;
void loop() {
  long irValue = particleSensor.getIR();
  Serial.println(irValue);
  float ret;
  // below code is to calculate IR value and return BPM
  float ans = BandPassFilter_irValue(irValue);
  float acc_x = (float)myIMU.readFloatAccelX();
  float acc_y = (float)myIMU.readFloatAccelY();
  float acc_z = (float)myIMU.readFloatAccelZ();
  if (irValue > 60000) {
    ret = LMS(ans,acc_x,acc_y,acc_z);
    if(buffer_index >= 10) buffer_index = 0;
    buffer_ir[buffer_index] = irValue;
    float scaled_ret = ret+avg;
    Serial.print("scaled_ret=");
    Serial.print(scaled_ret);
    Serial.print("old_ret=");
    Serial.print(ret+123000);
    Serial.println();
    buffer_ret[buffer_index] = scaled_ret;
    buffer_index++;
    queue_count++;
  }    
  
  if (!messagePending && messageSending && queue_count >= 0)
    {
       //Serial.println(queue_count);
        char messagePayload[MESSAGE_MAX_LEN];
        bool temperatureAlert = readMessage(messageCount, messagePayload,buffer_ir[messageCount%10],buffer_ret[messageCount%10]);
        //bool temperatureAlert = readMessage(messageCount, messagePayload,irValue,ret);
        sendMessage(iotHubClientHandle, messagePayload, temperatureAlert);
        messageCount++;
        queue_count--;
    }
    IoTHubClient_LL_DoWork(iotHubClientHandle);
}
