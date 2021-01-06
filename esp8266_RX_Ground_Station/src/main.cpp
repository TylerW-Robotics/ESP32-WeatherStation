/*
References:
https://github.com/NishantSahay7/Upload-Sensor-Data-to-Google-Sheets-from-NodeMCU/blob/master/arduino_code.txt

*/
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>

// Setup Google Sheets Code
String readString;
const char* ssid = "********";
const char* password = "**********";

const char* host = "script.google.com";
const int httpsPort = 443;

WiFiClientSecure client;
const char* fingerprint = "46 B2 C3 44 9C 59 09 8B 01 B6 F8 BD 4C FB 00 74 91 2F EF F6";
String GAS_ID = "------------------------------";  // Replace by your GAS service id

//define the pins used by the transceiver module
#define ss 15
#define rst 5
#define dio0 16

//Found Values
double lat = 0.0000000;
double lng = 0.0000000;

double temp = 0.00;
double pressure = 0.00;
double humidity = 0.00;

double accelX = 0.00;
double accelY = 0.00;
double accelZ = 0.00;

//Time settings
unsigned int time_interval = 600000; // 10 min = 600000 ms
unsigned long current_time = 0;
bool first_data_ = 0;

void sendData(double lat_, double lng_, double temp){
  client.setInsecure();
  Serial.print("connecting to ");
  Serial.println(host);
  if (!client.connect(host, httpsPort)) {
    Serial.println("connection failed");
    return;
  }

  if (client.verify(fingerprint, host)) {
  Serial.println("certificate matches");
  } else {
  Serial.println("certificate doesn't match");
  }
  String string_lat     =  String(lat_, 5);
  String string_lng     =  String(lng_, 5);
  String string_temp     =  String(temp, 2);
  String url = "/macros/s/" + GAS_ID + "/exec?lat=" + string_lat + "&lng=" + string_lng + "&temp=" + string_temp; //https://script.google.com/macros/s/yourcodehere/exec?A=1&temperature=2&humidity=3
  Serial.print("requesting URL: ");
  Serial.println(url);

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
         "Host: " + host + "\r\n" +
         "User-Agent: BuildFailureDetectorESP8266\r\n" +
         "Connection: close\r\n\r\n");

  Serial.println("request sent");
    while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("headers received");
      break;
    }
  }
  String line = client.readStringUntil('\n');
  if (line.startsWith("{\"state\":\"success\"")) {
    Serial.println("esp8266/Arduino CI successfull!");
  } else {
    Serial.println("esp8266/Arduino CI has failed");
  }
  Serial.println("reply was:");
  Serial.println("==========");
  Serial.println(line);
  Serial.println("==========");
  Serial.println("closing connection");
}

double find_num(char *value, int *count){
  char buff[16];
  bool s_ = 1;
  int c_old = *count;
  int c_new = 0;
  while (s_ == 1){
    if ((value[c_old] == '\n') || (value[c_old] == '\0')){
      s_ = 0;
      buff[c_new] = '\0';
    } else {
      buff[c_new] = value[c_old];
    }
    c_new++;
    c_old++;
  }
  *count = c_old - 1;
  return atof(buff);
}

void printValues(){
  Serial.print("\nTest:\nLat: "); Serial.print(lat, 8);
  Serial.print("\nLng: "); Serial.print(lng, 8);
  Serial.print("\nTemp: "); Serial.print(temp, 2);
  Serial.print("\nPressure: "); Serial.print(pressure, 2);
  Serial.print("\nHumidity: "); Serial.print(humidity, 2);
  Serial.print("\nAcceleration-X: "); Serial.print(accelX, 2);
  Serial.print("\nAcceleration-Y: "); Serial.print(accelY, 2);
  Serial.print("\nAcceleration-Z: "); Serial.println(accelZ, 2);
}

void collect_values(char *message){
  bool status = 1;
  int cnt = 0;
  while (status == 1){
    //Serial.print("message val: "); Serial.println(message[cnt]);
    if(message[cnt] == '$'){
      // GPS code values
      if ((message[cnt + 1] == 'L') && (message[cnt + 2] == 'a') && (message[cnt + 3] == 't') && (message[cnt + 4] == ':')){
        cnt = cnt + 5;
        lat = find_num(message, &cnt);
      }
      else if ((message[cnt + 1] == 'L') && (message[cnt + 2] == 'n') && (message[cnt + 3] == 'g') && (message[cnt + 4] == ':')){
        cnt = cnt + 5;
        lng = find_num(message, &cnt);
      }
      // BME280 code values
      else if ((message[cnt + 1] == 'T') && (message[cnt + 2] == 'm') && (message[cnt + 3] == 'p') && (message[cnt + 4] == ':')){
        cnt = cnt + 5;
        temp = find_num(message, &cnt);        
      } 
      else if ((message[cnt + 1] == 'P') && (message[cnt + 2] == 'r') && (message[cnt + 3] == 's') && (message[cnt + 4] == ':')){
        cnt = cnt + 5;
        pressure = find_num(message, &cnt);        
      } 
      else if ((message[cnt + 1] == 'H') && (message[cnt + 2] == 'u') && (message[cnt + 3] == 'm') && (message[cnt + 4] == ':')){
        cnt = cnt + 5;
        humidity = find_num(message, &cnt);        
      } 
      // Acceleration values
      else if ((message[cnt + 1] == 'X') && (message[cnt + 2] == 'a') && (message[cnt + 3] == 'c') && (message[cnt + 4] == ':')){
        cnt = cnt + 5;
        accelX = find_num(message, &cnt);        
      } 
      else if ((message[cnt + 1] == 'Y') && (message[cnt + 2] == 'a') && (message[cnt + 3] == 'c') && (message[cnt + 4] == ':')){
        cnt = cnt + 5;
        accelY = find_num(message, &cnt);        
      } 
      else if ((message[cnt + 1] == 'Z') && (message[cnt + 2] == 'a') && (message[cnt + 3] == 'c') && (message[cnt + 4] == ':')){
        cnt = cnt + 5;
        accelZ = find_num(message, &cnt);        
      } 
      //If the sequence is wrong, end the search
      else {
        //Serial.println("Not Found\nError is the message");
        status = 0;
      }  
    }
    if ((message[cnt] == '\0')){
      //Serial.println("END");
      status = 0;
    }
    cnt++;
    //Serial.println(cnt);
    
  }

  printValues(); //debug for view values in the terminal  

}

void setup() {
  Serial.begin(9600);//initialize Serial Monitor
  //while (!Serial);
  Serial.println("LoRa Receiver");

  WiFi.mode(WIFI_STA);        //Setup Wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  LoRa.setPins(ss, rst, dio0);//setup LoRa transceiver module
  while (!LoRa.begin(915E6)) { //433E6 for Asia, 866E6 for Europe, 915E6 for North America
    Serial.println(".");
    delay(500);
  }

  // Change sync word (0xF3) to match the receiver
  LoRa.setSyncWord(0xF3);// The sync word assures you don't get LoRa messages from other LoRa transceivers// ranges from 0-0xFF
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  char message_data[120];
  bool check_ = 0;
  if (packetSize) {
    // received a packet
    //Serial.print("Received packet '\n\n");

    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      //Serial.print(LoRaData); 
      LoRaData.toCharArray(message_data, 120);

      
      collect_values(message_data);
      check_ = 1;
      

    }
    // print RSSI of packet
    //Serial.print("\n\n' with RSSI ");
    //Serial.println(LoRa.packetRssi());
  }

  if (((check_ == 1) && ((unsigned long)(millis() - current_time) > time_interval)) || (first_data_ == 0 && check_ == 1)) {
    current_time = millis();
    sendData(lat, lng, temp);
    first_data_ = 1;

  }
}