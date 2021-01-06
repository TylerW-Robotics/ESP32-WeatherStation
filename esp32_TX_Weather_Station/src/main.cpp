#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Adafruit_BME280.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_GFX.h> /*Also needs the Adafruit BusIO installed if not done yet */
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

//define the pins used by the transceiver module
#define ss 5
#define rst 14
#define dio0 2

// BME280 & LSM9DS1 Setup
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;                        // i2c sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();  // i2c sensor
float temp; //C*
float pressure;//hPa
float altitude;//m
float humidity;//%
sensors_event_t accel, mag, gyro, t_;

// Setup end

// OLED Setup
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Create display Object
void oled_display();
//OLED end

// GPS set up
int RXPin = 16;
int TXPin = 17;
int GPSBaud = 9600;                         //Default baud of NEO-6M is 9600
TinyGPSPlus gps;                            // Create a TinyGPS++ object
SoftwareSerial gpsSerial(RXPin, TXPin);     // Create a software serial port called "gpsSerial"
// Setup end

// Time Handling
unsigned long oled_current_time = 0;
uint16_t oled_time_interval = 1000;

unsigned long current_time = 0;

void init_OLED(){
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.display();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

}

void oled_display_GPS(){
  uint8_t y_ = 20;

  display.clearDisplay();
  
  display.setCursor(2, 2);
  display.println("GPS:");

  display.drawRect(0, 0, 128, 16, WHITE);
  display.drawRect(0, 17, 128, (SCREEN_HEIGHT - 17), WHITE);

  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    double alt = gps.altitude.meters();

    display.setCursor(2, y_);
    display.println("Lat:");

    if (lat < 0) {
      display.setCursor((50), y_);
    } else {
      display.setCursor((56), y_);
    }
    display.println(gps.location.lat(), 6);
    
    display.setCursor(2, (y_ + 10));
    display.println("Long:");
    if (lng < 0) {
      display.setCursor((50), (y_ + 10));
    } else {
      display.setCursor((56), (y_ + 10));
    }
    display.println(gps.location.lng(), 6);
    
    display.setCursor(2, (y_ + 20));
    display.print("Alt (m):");
    if (alt < 0) {
      display.setCursor((50), (y_ + 20));
    } else {
      display.setCursor((56), (y_ + 20));
    }
    display.println(gps.altitude.meters());
  } else {
    display.setCursor(10, y_);
    display.println("Not Available");
  }

  display.display();
}

void radio_send_message(String str){
  
  LoRa.beginPacket();
//  LoRa.print("hello ");
  LoRa.print(str);
  LoRa.endPacket();
}

void collect_values(){
  temp = bme.readTemperature(); //C*
  pressure = bme.readPressure();//hPa
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);//m
  humidity = bme.readHumidity();//%
  lsm.getEvent(&accel, &mag, &gyro, &t_);
}


void read_values(){
  Serial.println("GPS values:");
  Serial.print("Lat: "); Serial.println(gps.location.lat(), 7);
  Serial.print("Lng: "); Serial.println(gps.location.lng(), 7);

  Serial.println("BME280 values:");
  Serial.print("Temp(C*) = "); Serial.println(temp);
  Serial.print("Pressure = "); Serial.println(pressure);
  Serial.print("Altitude = "); Serial.println(altitude);
  Serial.print("Humidity = "); Serial.println(humidity);

  Serial.println("LSM9DS1 values:");
  Serial.print("accel: (X,Y,Z) {" );
  Serial.print(accel.acceleration.x); Serial.print(", ");
  Serial.print(accel.acceleration.y); Serial.print(", ");
  Serial.print(accel.acceleration.z); Serial.print("}\n");

  Serial.print("Mag: (X,Y,Z) {" );
  Serial.print(mag.magnetic.x); Serial.print(", ");
  Serial.print(mag.magnetic.y); Serial.print(", ");
  Serial.print(mag.magnetic.z); Serial.print("}\n");

  Serial.print("Gyro: (X,Y,Z) {" );
  Serial.print(gyro.gyro.x); Serial.print(", ");
  Serial.print(gyro.gyro.y); Serial.print(", ");
  Serial.print(gyro.gyro.z); Serial.print("}\n");
  
}

void setup() {
  //initialize Serial Monitor
  Serial.begin(9600);
  Serial.print("\nStarting Setup");

  Wire.begin();
  Serial.print(".");

  if (!bme.begin(0x76)){
    Serial.println("BME280 Sensor is missing!");
  }
  Serial.print(".");

  init_OLED();
  Serial.print(".");

  gpsSerial.begin(9600); // Setup GPS serial communication

  LoRa.setPins(ss, rst, dio0);  //setup LoRa transceiver module  

  while (!LoRa.begin(915E6)) {  //433E6 for Asia  //866E6 for Europe  //915E6 for North America
    Serial.println(".");
    delay(500);
  }
  Serial.print(".");

  if(!lsm.begin()){
    Serial.println("LSM9DS1 Sensor is missing!");
    while(1);
  }
  // Set ranges
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);

  Serial.print(".");

   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  
  Serial.println("\n\nSetup Complete!");
}

void loop() {

  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
    //Serial.write(gpsSerial.read());
  }

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected");
  }

  if((unsigned long)(millis() - oled_current_time) > oled_time_interval){ // allows arduino delay with other functions running in the background
    //Serial.println((unsigned long)(millis() - oled_current_time));
    oled_current_time = millis();
  
    oled_display_GPS();
    collect_values();
    read_values();
  }

  if((unsigned long)(millis() - current_time) > 10000){ // allows arduino delay with other functions running in the background
    current_time = millis();

    
    String str_message_GPS = String("$Lat:" + String(gps.location.lat(), 7) + "\n$Lng:" + String(gps.location.lng(), 7) + "\n");
    String str_message_BME = String("$Tmp:" + String(temp) + "\n$Prs:" + String(pressure) + "\n$Hum:" + String(humidity) + "\n");
    String str_message_LSM9DS1 = String("$Xac:" + String(accel.acceleration.x) + "\n$Yac:" + String(accel.acceleration.y) + "\n$Zac:" + String(accel.acceleration.z) + "\n");
    String str_message = String(str_message_GPS + str_message_BME + str_message_LSM9DS1 + "\0");
    radio_send_message(str_message);
    
  }

}
/*
  Serial.print("accel: (X,Y,Z) {" );
  Serial.print(accel.acceleration.x); Serial.print(", ");
  Serial.print(accel.acceleration.y); Serial.print(", ");
  Serial.print(accel.acceleration.z); Serial.print("}\n");
  */