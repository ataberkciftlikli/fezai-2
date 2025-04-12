#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SFE_BMP180.h> 
#include "MPU9250.h"

MPU9250 mpu;


SoftwareSerial GPS(0, 1); // RX, TX
SFE_BMP180 pressure;  // I2C BMP280

int led_green = 27; // Led 1  -> Green Status Led
int led_red = 28; // Led 2 -> Red Status Led
int buzzer = 29; // Buzzer
int relay_one = 25; // Recovery 1
int relay_two = 26; // Recovery 2

boolean apogee = false;
float altidute;
float altidute_old;
float velocity;

float security_altidute = 1; //DEĞİŞECEK!! STANDART 500
float altidute_secondary = 10; //DEĞİŞECEK!! STANDART 500
float last_altidute = 2; //DEĞİŞECEK!! STANDART 25

double baseline; // baseline pressure

int state = 0;
String state_string;
boolean system_state = false;

String gps_clock = "";    // hhmmss.00
String gps_latitude = "";   // DDMM.MMMM Kuzey/Güney N/S - Enlem
String gps_longitude = "";  // DDMM.MMMM Doğu/Batı E/W - Boylam
String gps_altidute = "";  // metre

void setup() {
  Serial.begin(9600);
  GPS.begin(9600);

  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail (disconnected?)\n\n");
    while(1); // Pause forever.
  }
  

  if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    
   baseline = getPressure();
  
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb");  


  pinMode(led_green,OUTPUT);
  pinMode(led_red,OUTPUT);
  pinMode(buzzer,OUTPUT);
  //pinMode(relay_one,OUTPUT);
  //pinMode(relay_two,OUTPUT);


  Serial.println("All sensors were calibrated successfully!");
  digitalWrite(led_green,HIGH);
  
  delay(100);
}

void loop() {

 
  altidute = pressure.altitude(getPressure(),baseline);
  mpu.update();

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

  if (state==1 && mpu.getPitch()>35) {
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
  //Serial.print("Saat:");
  //Serial.println(gps_clock);
  Serial.println("Enlem: 4099.7555");
  //Serial.println(gps_latitude);
  Serial.print("Boylam: 2906.4266");
  //Serial.print("GPS İrtifa:");
  //Serial.println(gps_altidute);
  Serial.println("");
  
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
  
  Serial.print("BMP İrtifa: ");
  Serial.println(altidute);
  Serial.print("Rakım (basınç): ");
  Serial.println(baseline);
  Serial.println("");

  Serial.print("Durum: ");
  Serial.println(state_string);

  Serial.print("Sistem Durumu: ");
  Serial.println(system_state);

  Serial.println("");
  
  //delay(1000);



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

double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
