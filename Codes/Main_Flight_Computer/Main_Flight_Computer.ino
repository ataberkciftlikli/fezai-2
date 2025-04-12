#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define rf Serial3

SoftwareSerial GPS(0, 1); // RX, TX
Adafruit_BMP280 bmp; // I2C BMP280
Adafruit_MPU6050 mpu; // I2C MPU6050

int led_green = 27; // Led 1  -> Green Status Led
int led_red = 28; // Led 2 -> Red Status Led
int buzzer = 29; // Buzzer
int relay_one = 25; // Recovery 1
int relay_two = 26; // Recovery 2

boolean apogee = false;
float altidute_setup;
float altidute;
float altidute_old;
float velocity;

float security_altidute = 1; //DEĞİŞECEK!! STANDART 500
float altidute_secondary = 10; //DEĞİŞECEK!! STANDART 500
float last_altidute = 2; //DEĞİŞECEK!! STANDART 25

int state = 0;
String state_string;
boolean system_state = true;

String gps_clock = "";    // hhmmss.00
String gps_latitude = "";   // DDMM.MMMM Kuzey/Güney N/S - Enlem
String gps_longitude = "";  // DDMM.MMMM Doğu/Batı E/W - Boylam
String gps_altidute = "";  // metre

void setup() {
  Serial.begin(9600);
  GPS.begin(9600);
  rf.begin(9600);
  
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  altidute_setup=bmp.readAltitude(1013.25);


  pinMode(led_green,OUTPUT);
  pinMode(led_red,OUTPUT);
  pinMode(buzzer,OUTPUT);
  pinMode(relay_one,OUTPUT);
  pinMode(relay_two,OUTPUT);


  Serial.println("All sensors were calibrated successfully!");
  digitalWrite(led_green,HIGH);
  delay(100);
}

void loop() {

  altidute_old=altidute;

  altidute = bmp.readAltitude(1013.25)-altidute_setup;

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if(altidute>=security_altidute && state==0) {
    state=1;
    digitalWrite(buzzer,HIGH);
    digitalWrite(led_green,LOW);
    digitalWrite(led_red,HIGH);
    delay(250);
    digitalWrite(buzzer,LOW);
    digitalWrite(led_green,HIGH);
    digitalWrite(led_red,LOW);
  }

  if (state==1 && altidute_old-altidute>=1) {
    digitalWrite(relay_one,HIGH);
    digitalWrite(buzzer,HIGH);
    digitalWrite(led_green,LOW);
    digitalWrite(led_red,HIGH);
    delay(1200); //DİRENÇ TELİ SÜRESİ
    digitalWrite(relay_one,LOW);
    digitalWrite(buzzer,LOW);
    digitalWrite(led_green,HIGH);
    digitalWrite(led_red,LOW);
    apogee=true;
    state=2;
}

if(state==2 && altidute<=altidute_secondary) {
    digitalWrite(relay_two,HIGH);
    digitalWrite(buzzer,HIGH);
    digitalWrite(led_green,LOW);
    digitalWrite(led_red,HIGH);
    delay(2000); //LINEER SÜRESİ
    digitalWrite(relay_one,LOW);
    digitalWrite(buzzer,LOW);
    digitalWrite(led_green,HIGH);
    digitalWrite(led_red,LOW);
    state=3;
}

if(state==3 && altidute<=last_altidute && 0<=altidute_old-altidute<=1) {
    state=4;
}


  switch (state) {
  case 0:
    state_string="Yerde"; // Waiting for flight
    break;
  case 1:
    state_string="Uçuş"; // Flight started
    break;
  case 2:
    state_string="Tepe noktasına ulaşıldı! Faydalı yük bıraklıdı. İkinci Paraşüt açıldı."; // Apogee, Payload, Recovery 1 -> Secondary Parachute
    break;
  case 3:
    state_string="Ana paraşüt açıldı!"; // 500 meters, Recovery 2 -> Main Parachute
    break;
  case 4:
    state_string="İniş Tamamlandı!"; // Flight completed
    digitalWrite(led_green,LOW);
    digitalWrite(buzzer,HIGH);
    delay(800);
    digitalWrite(led_green,HIGH);
    digitalWrite(buzzer,LOW);
    delay(500);
    digitalWrite(buzzer,HIGH);
    break;
}
  
  ListenGPS();
  Serial.println("<------ VERİ ------>");
  Serial.print("Saat:");
  Serial.println(gps_clock);
  Serial.print("Enlem: ");
  Serial.println(gps_latitude);
  Serial.print("Boylam: ");
  Serial.println(gps_longitude);
  Serial.print("GPS İrtifa:");
  Serial.println(gps_altidute);
  Serial.println("");
  
  Serial.print("BMP İrtifa: ");
  Serial.println(altidute);
  Serial.print("Rakım: ");
  Serial.println(altidute_setup);
  Serial.print("BMP Sıcaklık: ");
  Serial.println(bmp.readTemperature());
  Serial.println("");

  /* Print out the values */
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print(g.gyro.z);
  Serial.println("");

  Serial.print("MPU Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.print("Durum: ");
  Serial.println(state_string);

  Serial.print("Sistem Durumu: ");
  Serial.println(system_state);

  Serial.println("");
  rf.println("<------ VERI ------>");
  rf.print("Saat:");
  rf.println(gps_clock);
  rf.print("Enlem: ");
  rf.println(gps_latitude);
  rf.print("Boylam: ");
  rf.println(gps_longitude);
  rf.print("GPS Irtifa:");
  rf.println(gps_altidute);
  rf.println("");
  
  rf.print("BMP Irtifa: ");
  rf.println(altidute);
  rf.print("Rakim: ");
  rf.println(altidute_setup);
  rf.print("BMP Sicaklik: ");
  rf.println(bmp.readTemperature());
  rf.println("");

  /* Print out the values */
  rf.print(a.acceleration.x);
  rf.print(",");
  rf.print(a.acceleration.y);
  rf.print(",");
  rf.print(a.acceleration.z);
  rf.print(", ");
  rf.print(g.gyro.x);
  rf.print(",");
  rf.print(g.gyro.y);
  rf.print(",");
  rf.print(g.gyro.z);
  rf.println("");

  rf.print("MPU Temperature: ");
  rf.print(temp.temperature);
  rf.println(" degC");

  rf.print("Durum: ");
  rf.println(state_string);

  rf.print("Sistem Durumu: ");
  rf.println(system_state);
  delay(1000);



}

void ListenGPS() {

  if ( GPS.find("$GPGGA,") ) {

    gps_clock = GPS.readStringUntil(',');
    gps_latitude = GPS.readStringUntil(',');
    gps_latitude.concat(GPS.readStringUntil(','));
    gps_longitude = GPS.readStringUntil(',');
    gps_longitude.concat(GPS.readStringUntil(','));

    for ( int i = 0; i < 3; i++ ) {
      GPS.readStringUntil(',');
    }

    gps_altidute = GPS.readStringUntil(',');
    gps_altidute.concat(GPS.readStringUntil(','));

    GPS.readStringUntil('\r');
  }
} 
