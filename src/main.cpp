#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>

#define RELAY_PIN D6
#define BUTTON_RIGHT_PIN D8
#define BUTTON_LEFT_PIN D7

#define DISPLAY_I2C_ADDRESS 0x3c
#define BMP280_I2C_ADDRESS 0x76
#define AHT_I2C_ADDRESS 0x38

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_BMP280 bmp; // I2C
Adafruit_AHTX0 aht;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool initDisplay()
{
    if (!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_I2C_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        // for(;;); // Don't proceed, loop forever
        return false;
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
    return true;
}

bool initBmp()
{
    if (!bmp.begin(BMP280_I2C_ADDRESS))
    {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
        return false;
    }
    return true;
}

bool initAht()
{
    if (!aht.begin())
    {
        Serial.println("Could not find AHT? Check wiring");
        return false;
    }
    Serial.println("AHT10 or AHT20 found");
    return true;
}

void setup()
{
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);
    digitalWrite(RELAY_PIN, LOW);

    Serial.begin(115200);

    Serial.println(F("BMP280 test"));

    initDisplay();
    initBmp();
    initAht();
}

void loop()
{
    display.setCursor(0, 0);
    display.clearDisplay();

    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    display.print("Temperature: ");
    display.print(bmp.readTemperature());
    display.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure() / 100.0F);
    Serial.println(" hPa");
    display.print("Pressure: ");
    display.print(bmp.readPressure() / 100.0F);
    display.println(" hPa");

sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

    Serial.println();
    display.display();
    delay(1000);
}