#include <MQUnifiedsensor.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <DHT.h>
#include <DHT_U.h>

TwoWire I2C_BMP280 = TwoWire(0);
TwoWire I2C_MPU6050 = TwoWire(1);

#define DHTTYPE DHT22
#define DHTPIN 5
#define SDA1_PIN 21
#define SCL1_PIN 22
#define SDA2_PIN 18
#define SCL2_PIN 19
#define MOSI 12
#define MISO 27
#define SCK 14
#define CS 13
#define SEALEVELPRESSURE_HPA (1013.25)
#define placa "ESP-32"
#define Voltage_Resolution 3.3
#define pin 15 
#define type "MQ-135" 
#define ADC_Bit_Resolution 12
#define RatioMQ135CleanAir 3.6

Adafruit_BMP280 bmp (&I2C_BMP280);
Adafruit_MPU6050 mpu;
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);
DHT dht22(DHTPIN, DHTTYPE);
SPIClass spi = SPIClass(VSPI);


float tempDHT, humDHT, hPa, tempBMP, altitude, PPM, tempMed;
int tiempo;
int16_t* ax, ay, az, gx, gy, gz;
int sensorPin = 15;
int sensorValue = 0;
int val;
int i = 0, t;

void setup() 
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Iniciando...");
  spi.begin(SCK, MISO, MOSI, CS);
  I2C_MPU6050.begin(SDA1_PIN, SCL1_PIN, 100000);
  I2C_BMP280.begin(SDA2_PIN, SCL2_PIN, 100000);
  pinMode(34, INPUT);
  pinMode(15, INPUT);
  dht22.begin();
  MQ135.setRegressionMethod(1);
  MQ135.setA(110.47); MQ135.setB(-2.862); 
  MQ135.init();  
  MQ135.setRL(1);
  MQ135.setR0(0.49);
  MQ135.serialDebug(true);
  if (!mpu.begin(0x68, &I2C_MPU6050)) {
    Serial.println("No se pudo encontrar el MPU6050");
    while (1);
  }
  if (!bmp.begin(0x76)) {
    Serial.println("No se pudo encontrar el BMP280");
    while (1);
  }
  SD.begin(13,spi,4000000);
  if (!SD.begin(13,spi,4000000)) 
  {
    Serial.println("Error al inicializar la tarjeta SD");
    return;
  }
  Serial.println("Tarjeta SD inicializada.");

  // Crear carpeta "datos"
  if (!SD.exists("/datos"))
  {
    Serial.println("Intentando crear la carpeta 'datos'...");
    if (SD.mkdir("/datos"))
    {
      Serial.println("Carpeta 'datos' creada.");
    }
    else
    {
      Serial.println("Error al crear la carpeta 'datos'.");
    }
  } 
  else
  {
    Serial.println("La carpeta 'datos' ya existe.");
  }

  // Crear archivo .csv en la carpeta "datos"
  Serial.println("Intentando crear el archivo 'datos.csv'...");
  File file = SD.open("/datos/datos.csv", FILE_WRITE);
  if (file)
  {
    file.println("Tiempo,Altura(MSNM),CO2(PPM),Presion(hPa),Humedad(Med),Temperatura(°C),Accel-X,Accel-Y,Accel-Z(m/s^2),Gyro-X,Gyro-Y,Gyro-Z(rad/s)");
    file.close();
    Serial.println("Archivo 'datos.csv' creado y encabezado escrito.");
  } 
  else
  {
    Serial.println("Error al crear el archivo 'datos.csv'.");
  }
}

void loop()
{
  
  if (digitalRead(34))
  {   
    // MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // DHT22
    humDHT = dht22.readHumidity();
    tempDHT = dht22.readTemperature();

    // BMP280
    tempBMP = bmp.readTemperature();
    hPa = bmp.readPressure() / 100;
    altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    // MQ135
    MQ135.update();
    PPM = MQ135.readSensor() + 422;

    tempMed = (tempDHT+tempBMP)/2;
    tiempo = millis();
    tiempo = tiempo/1000;

    File file = SD.open("/datos/datos.csv", FILE_APPEND);
    if (file)
    {
      file.print(tiempo);
      file.print(",");
      file.print(altitude);
      file.print(",");
      file.print(PPM);
      file.print(",");
      file.print(hPa);
      file.print(",");
      file.print(humDHT);
      file.print(",");
      file.print(tempMed);
      file.print(",");
      file.print(a.acceleration.x);
      file.print(",");
      file.print(a.acceleration.y);
      file.print(",");
      file.print(a.acceleration.z);
      file.print(",");
      file.print(g.gyro.x);
      file.print(",");
      file.print(g.gyro.y);
      file.print(",");
      file.println(g.gyro.z);
      file.flush();
      Serial.print(tiempo);
      Serial.print(",");
      Serial.print(altitude);
      Serial.print(",");
      Serial.print(PPM);
      Serial.print(",");
      Serial.print(hPa);
      Serial.print(",");
      Serial.print(humDHT);
      Serial.print(",");
      Serial.print(tempMed);
      Serial.print(",");
      Serial.print(a.acceleration.x);
      Serial.print(",");
      Serial.print(a.acceleration.y);
      Serial.print(",");
      Serial.print(a.acceleration.z);
      Serial.print(",");
      Serial.print(g.gyro.x);
      Serial.print(",");
      Serial.print(g.gyro.y);
      Serial.print(",");
      Serial.print(g.gyro.z);
      Serial.println(",");
      file.close();
      i++;
    }
    else
    Serial.println("No se encontro la carpeta");
    delay(3000);
  }
  else
    {
    Serial.println("No desplegado");
    delay(100);
    }
}
