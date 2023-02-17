#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define RELAY_PIN D7
#define BUTTON_RIGHT_PIN D6
#define BUTTON_LEFT_PIN D5

#define DISPLAY_I2C_ADDRESS 0x3c
#define BMP280_I2C_ADDRESS 0x76
#define DHT2X_I2C_ADDRESS 0x38
Adafruit_BMP280 bme; // I2C

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void i2cScann()
{
  byte Error, address;
  int DeviceCount;
  Serial.println("Scanning I2C devices...");
  DeviceCount = 0;

  Wire.begin();
  while (true)
  {
    for (address = 1; address < 127; address++)
    {
      Wire.beginTransmission(address);
      Error = Wire.endTransmission();
      if (Error == 0)
      {
        Serial.print("I2C device found at address 0x");
        if (address < 16)
        {
          Serial.print("0");
        }
        Serial.println(address, HEX);
        DeviceCount++;
      }
      else if (Error == 4)
      {
        Serial.print("Unknown Error at address 0x");
        if (address < 16)
        {
          Serial.print("0");
        }
        Serial.println(address, HEX);
      }
    }
    if (DeviceCount == 0)
    {
      Serial.println("No I2C devices found!");
    }
    else
    {
      Serial.println("Success!\n");
    }
    delay(5000);
  }
}


void setup()
{
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);
  digitalWrite(RELAY_PIN, LOW);

  Serial.begin(115200);
  i2cScann();


if(!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(100); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.display();
  display.setTextSize(1.8);
  display.setTextColor(WHITE);




  Serial.println(F("BMP280 test"));

  if (!bme.begin(BMP280_I2C_ADDRESS))
  {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1)
      ;
  }
}

void loop()
{
  display.setCursor(0,0);
  display.clearDisplay();
  
  Serial.print("Temperature = "); Serial.print(bme.readTemperature()); Serial.println(" *C");
  display.print("Temperature: "); display.print(bme.readTemperature()); display.println(" *C");

  Serial.print("Pressure = "); Serial.print(bme.readPressure() / 100.0F); Serial.println(" hPa");
  display.print("Pressure: "); display.print(bme.readPressure() / 100.0F); display.println(" hPa");

  // Serial.print("Humidity = "); Serial.print(bme.readHumidity()); Serial.println(" %");
  // display.print("Humidity: "); display.print(bme.readHumidity()); display.println(" %");

  Serial.println();
  display.display();
  delay(1000);
}