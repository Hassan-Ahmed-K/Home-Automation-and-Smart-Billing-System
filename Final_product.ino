

// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID "TMPL6kEnA--dF"
#define BLYNK_DEVICE_NAME "Home Automation System"
#define BLYNK_AUTH_TOKEN "mREF5XiBeKDeaK5JFuqjiFclz82SPPXg"
#define BLYNK_TEMPLATE_NAME "Home Automation System"
#define BLYNK_PRINT Serial

#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
// LiquidCrystal_I2C lcd(0x27, 20, 4);

#include "EmonLib.h"
#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include "time.h"
#include "sntp.h"

BlynkTimer timer;
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Zzz";
char pass[] = "123456780";
// char ssid[] = "Galaxy A03s3605";
// char pass[] = "hassan12";
 
float kWh = 0;


const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 18000;
const int   daylightOffset_sec = 3600;

const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";

bool dayPassed = false;
int day_count = 1;
float fixedUnitsLimit =0.2;
float unitsLimit = 0.2;
time_t referenceTime = 0;
const int addrDay = 8;




unsigned long lastmillis = millis();

#define button1_pin 18
#define button2_pin 5
#define button3_pin 33
#define button4_pin 32

#define relay1_pin 23
#define relay2_pin 22
#define relay3_pin 21
#define relay4_pin 19

#define currCaliobrstionPin 34
#define voltageCaliobrstionPin 35

EnergyMonitor emon;
// #define vCalibration 83.3
// #define currCalibration 0.50

#define vCalibration 41.5
#define currCalibration 0.15

int relay1_state = digitalRead(button1_pin);
int relay2_state = digitalRead(button2_pin);
int relay3_state = digitalRead(button3_pin);
int relay4_state = digitalRead(button4_pin);
float price = 0;
float gross_tax = 0;
const int addrKWh = 12;
int printer =1;


//Change the virtual pins according the rooms
#define button1_vpin    V1
#define button2_vpin    V2
#define button3_vpin    V3 
#define button4_vpin    V4
// #define current_vpin    V5
// #define voltage_vpin    V6
// #define power_vpin      V7
#define kwh_vpin        V5


//------------------------------------------------------------------------------
// This function is called every time the device is connected to the Blynk.Cloud
// Request the latest state from the server
BLYNK_CONNECTED() {
  Blynk.syncVirtual(button1_vpin);
  Blynk.syncVirtual(button2_vpin);
  Blynk.syncVirtual(button3_vpin);
  Blynk.syncVirtual(button4_vpin);
}

//--------------------------------------------------------------------------
// This function is called every time the Virtual Pin state change
//i.e when web push switch from Blynk App or Web Dashboard
BLYNK_WRITE(button1_vpin) {
  delay(200);
  if(digitalRead(button1_pin) != LOW){
    relay1_state = param.asInt();
  digitalWrite(relay1_pin, relay1_state);
  }
  else{
    relay1_state =1;
    Blynk.virtualWrite(button1_vpin, relay1_state);
  digitalWrite(relay1_pin, relay1_state);
  delay(200);
  }
}
//--------------------------------------------------------------------------
BLYNK_WRITE(button2_vpin) {
  delay(200);
  if(digitalRead(button2_pin) != LOW){
    relay2_state = param.asInt();
  digitalWrite(relay2_pin, relay2_state);}
  else{
    relay2_state =1;
    Blynk.virtualWrite(button2_vpin, relay2_state);
  digitalWrite(relay2_pin, relay2_state);
  delay(200);
  }
}
//--------------------------------------------------------------------------
BLYNK_WRITE(button3_vpin) {
  delay(200);
  if(digitalRead(button3_pin) != LOW){
    relay3_state = param.asInt();
  digitalWrite(relay3_pin, relay3_state);}
  else{
    relay3_state =1;
    Blynk.virtualWrite(button3_vpin, relay3_state);
  digitalWrite(relay3_pin, relay3_state);
  delay(200);
  }
}
//--------------------------------------------------------------------------
BLYNK_WRITE(button4_vpin) {
  delay(200);
  if(digitalRead(button4_pin) != LOW){
    relay4_state = param.asInt();
  digitalWrite(relay4_pin, relay4_state);}
  else{
    relay4_state =1;
    Blynk.virtualWrite(button4_vpin, relay4_state);
  digitalWrite(relay4_pin, relay4_state);
  delay(200);
  }
}
//--------------------------------------------------------------------------

const int rs = 13, en = 12, d4 = 14, d5 = 27, d6 = 26, d7 = 25, ct=2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);



void setup()
{
  // Debug console
  Serial.begin(115200);

  //--------------------------------------------------------------------
  pinMode(button1_pin, INPUT_PULLUP);
  pinMode(button2_pin, INPUT_PULLUP);
  pinMode(button3_pin, INPUT_PULLUP);
  pinMode(button4_pin, INPUT_PULLUP);
  //--------------------------------------------------------------------
  pinMode(relay1_pin, OUTPUT);
  pinMode(relay2_pin, OUTPUT);
  pinMode(relay3_pin, OUTPUT);
  pinMode(relay4_pin, OUTPUT);
  //--------------------------------------------------------------------
  //During Starting all Relays should TURN OFF
  digitalWrite(relay1_pin, relay1_state);
  digitalWrite(relay2_pin, relay2_state);
  digitalWrite(relay3_pin, relay3_state);
  digitalWrite(relay4_pin, relay4_state);
  //--------------------------------------------------------------------
  Blynk.begin(auth, ssid, pass);
  // You can also specify server:
  //Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  //Blynk.begin(auth, ssid, pass, IPAddress(192,168,1,100), 8080);
  //--------------------------------------------------------------------
  Blynk.virtualWrite(button1_vpin, relay1_state);
  Blynk.virtualWrite(button2_vpin, relay2_state);
  Blynk.virtualWrite(button3_vpin, relay3_state);
  Blynk.virtualWrite(button4_vpin, relay4_state);

  //billing system
  emon.voltage(voltageCaliobrstionPin, vCalibration, 1.7);  // Voltage: input pin, calibration, phase_shift
  emon.current(currCaliobrstionPin, currCalibration);    // Current: input pin, calibration.
  
  timer.setInterval(5000L, myTimerEvent);

  readEnergyDataFromEEPROM();
  readDateFromEEPROM();

  unitsLimit = fixedUnitsLimit *day_count;

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Home Automation And");
  lcd.setCursor(0, 1);
  lcd.print("Billing System");
  delay(3000);
  lcd.clear();
  // analogWrite(ct,50);


  sntp_set_time_sync_notification_cb(timeAvailable);

  sntp_servermode_dhcp(1);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);


  // Set the reference time to the current time
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    referenceTime = mktime(&timeinfo);
  }




}
  //--------------------------------------------------------------------
void myTimerEvent()
{
  emon.calcVI(20, 2000);
  kWh = kWh + emon.apparentPower * (millis() - lastmillis) / 3600000000.0;
  price = unit_price_calculation(kWh);
  // saveEnergyDataToEEPROM();
  yield();
  saveEnergyDataToEEPROM();
  Serial.print("Vrms: ");
  Serial.print(emon.Vrms, 2);
  Serial.print("V");
  // EEPROM.put(0, emon.Vrms);
  // delay(100);
 
  Serial.print("\tIrms: ");
  Serial.print(emon.Irms, 4);
  Serial.print("A");
  // EEPROM.put(4, emon.Irms);
  // delay(100);
 
  Serial.print("\tPower: ");
  Serial.print(emon.apparentPower, 4);
  Serial.print("W");
  // EEPROM.put(8, emon.apparentPower);
  // delay(100);
 
  Serial.print("\tkWh: ");
  Serial.print(kWh, 5);
  // Serial.println("kWh");
  // EEPROM.put(12, kWh);

  Serial.print("\tPrice: ");
  Serial.print(price, 6);
  Serial.println("PKR");
  delay(100);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
 if(printer==1){
  
  lcd.clear();
  lcd.home();
  lcd.print("Vrms:");
  lcd.print(emon.Vrms, 2);
  lcd.print("V");
  lcd.setCursor(0, 1);
  lcd.print("Irms:");
  lcd.print(emon.Irms, 4);
  lcd.print("A");
  printer++;
 }
// delay(5000);
else if(printer==2){
  lcd.clear();
  lcd.home();
  lcd.setCursor(0, 0);
  lcd.print("Power:");
  lcd.print(emon.apparentPower, 4);
  lcd.print("W");
  lcd.setCursor(0, 1);
  lcd.print("kWh:");
  lcd.print(kWh, 4);
  lcd.print("W");
  printer++;
}
  // delay(5000);
  else if(printer==3){
  lcd.clear();
  lcd.home();
  lcd.setCursor(0, 0);
  lcd.print("Units:");
  lcd.print(kWh, 4);
  lcd.print("W");
  lcd.setCursor(0, 1);
  lcd.print("Price:");
  lcd.print(price, 2);
  lcd.print("PKR");
  printer=1;
  }
  // delay(5000);

 
  lastmillis = millis();
 
  // Blynk.virtualWrite(voltage_vpin, emon.Vrms);
  // Blynk.virtualWrite(current_vpin, emon.Irms);
  // Blynk.virtualWrite(power_vpin, emon.apparentPower);
  Blynk.virtualWrite(kwh_vpin, kWh);
}

void loop()
{

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    time_t currentTime = mktime(&timeinfo);

    // Calculate the difference in seconds between the current time and the reference time
    int secondsPassed = difftime(currentTime, referenceTime);

    // Check if one day has passed
    if (secondsPassed >= 24*24*60) { // 24 hours in seconds
      Serial.println("One day has passed!");
      dayPassed = true;
      day_count +=1;
      if (day_count > 30){
        day_count = 1;
        kWh=0;
      }
      unitsLimit = fixedUnitsLimit *day_count;

      Serial.print("day_count");
      Serial.println(day_count);
      Serial.print("unitsLimit");
      Serial.println(unitsLimit);
      // Update the reference time to the current time for the next check
      referenceTime = currentTime;
      saveDateToEEPROM();
    } 
    // printLocalTime();
  } else {
    Serial.println("Failed to obtain time");
  }

  Blynk.run();
  timer.run();
  listen_push_buttons();
  
}

//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
void listen_push_buttons(){

     if(kWh<unitsLimit){
      if(digitalRead(button1_pin) == LOW ) {
      Serial.println(digitalRead(button1_pin));
      delay(200);
      control_relay(1);
      Blynk.virtualWrite(button1_vpin, relay1_state); //update button state
      delay(200);
    }
    //--------------------------------------------------------------------------
     if (digitalRead(button2_pin) == LOW ){
      Serial.println(digitalRead(button2_pin));
      delay(200);
      control_relay(2);
      Blynk.virtualWrite(button2_vpin, relay2_state); //update button state
      delay(200);
    }
    //--------------------------------------------------------------------------
    if (digitalRead(button3_pin) == LOW ){
      
      delay(200);
      control_relay(3);
      Blynk.virtualWrite(button3_vpin, relay3_state); //update button state
      delay(200);
    }
    //--------------------------------------------------------------------------
     if (digitalRead(button4_pin) == LOW ){
      delay(200);
      control_relay(4);
      Blynk.virtualWrite(button4_vpin, relay4_state); //update button state
      delay(200);
    }
     }else{
      delay(200);

      digitalWrite(relay1_pin, 0);
      Blynk.virtualWrite(button1_vpin, 1); 
      delay(200);
      digitalWrite(relay2_pin, 0);
      Blynk.virtualWrite(button2_vpin, 1); 
      delay(200);
      unitsLimit+=fixedUnitsLimit;
      lcd.begin(16, 2);
  lcd.setCursor(0, 0);
      lcd.clear();
      lcd.home();
      lcd.setCursor(0, 0);
      lcd.print("Daily Limit");
      lcd.setCursor(0, 3);
      lcd.print("Exceed");
      delay(5000);
      lcd.clear();
      lcd.home();
      lcd.setCursor(0, 0);
      lcd.print("New Limit:");
      lcd.setCursor(0, 1);
      lcd.print(unitsLimit,5);
      lcd.print(" Units");
      delay(5000);

      // digitalWrite(relay3_pin, 0);
      // Blynk.virtualWrite(button3_vpin, 1); 
      // delay(200)
      // digitalWrite(relay4_pin, 0);
      // Blynk.virtualWrite(button4_vpin, 1); 
      // delay(200)
     }
    //--------------------------------------------------------------------------
}

void control_relay(int relay){
  //------------------------------------------------

  if(relay == 1 ){

    relay1_state = 1;
    delay(50);
    digitalWrite(relay1_pin, relay1_state);
    Serial.print("Relay1 State = ");
    Serial.println(relay1_state);
    
  }
  //------------------------------------------------
  else if(relay == 2){
    relay2_state = 1;
    delay(50);
    digitalWrite(relay2_pin, relay2_state);
    Serial.print("Relay2 State = ");
    Serial.println(relay2_state);
    
  }
  //------------------------------------------------
  else if(relay == 3){
    relay3_state = 1;
    delay(50);
    digitalWrite(relay3_pin, relay3_state);
    
  }
  //------------------------------------------------
  else if(relay == 4){
    relay4_state = 1;
    delay(50);
    digitalWrite(relay4_pin, relay4_state);
  }}


float unit_price_calculation(float kwh ){
  if(kwh<=100){
    price = kwh * 5.79;
  }
  else if(kwh < 100 && kwh <= 200){
    price = kwh * 8.11;
  }
  else if(kwh < 201 && kwh <= 300){
    price = kwh * 10.2;
  }
  else if(kwh < 300 && kwh <= 700){
    price = kwh * 17.6;
  }
  else{
    price = kwh * 20.6;
  }

  if(price < 25000){
    gross_tax = price * 0.075;
  }

  price = price + gross_tax;

  return price;
}


void readEnergyDataFromEEPROM()
{
  // Read the stored kWh value from EEPROM
  EEPROM.get(addrKWh, kWh);
 
  // Check if the read value is a valid float. If not, initialize it to zero
  if (isnan(kWh))
  {
    kWh = 0.0;
    saveEnergyDataToEEPROM(); // Save initialized value to EEPROM
  }
}
 
 
void saveEnergyDataToEEPROM()
{
  // Write the current kWh value to EEPROM
  EEPROM.put(addrKWh, kWh);
 
  // Commit changes to EEPROM
  EEPROM.commit();
}


void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void timeAvailable(struct timeval *t)
{
  Serial.println("Got time adjustment from NTP!");
  printLocalTime();
}

void readDateFromEEPROM()
{
  EEPROM.get(addrDay, day_count);
  if (isnan(day_count))
  {
    day_count = 1;
    saveDateToEEPROM(); // Save initialized value to EEPROM
  }
}
 
void saveDateToEEPROM()
{
  EEPROM.put(addrDay, day_count);
  EEPROM.commit();
}