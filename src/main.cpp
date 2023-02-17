#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <DHT.h>

#define RELAY_PIN D6
#define BUTTON_RIGHT_PIN D8
#define BUTTON_LEFT_PIN D7
#define DHT22_PIN D5 // Digital pin connected to the DHT sensor

#define DHTTYPE DHT22 // DHT 22  (AM2302), AM2321

#define DISPLAY_I2C_ADDRESS 0x3c
#define BMP280_I2C_ADDRESS 0x76
#define AHT_I2C_ADDRESS 0x38

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)

struct DHT22_readings
{
    float humidity;
    float temperature;
    float heatIndex;
} dht22_readings;

struct DHT11_readings
{
    float humidity;
    float temperature;
    float heatIndex;
} dht11_readings;

struct AHT21_readings
{
    float humidity;
    float temperature;
    float heatIndex;
} aht21_readings;

struct BMP280_readings
{
    float pressure;
    float temperature;
    float altitude;
} bmp280_readings;

DHT dht22(DHT22_PIN, DHTTYPE);
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

bool initDht()
{
    dht22.begin();
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
    initDht();
}

float convertCtoFglobal(float c) { return c * 1.8 + 32; }

float convertFtoCglobal(float f) { return (f - 32) * 0.55555; }

float computeHeatIndexGlobal(float temperature, float percentHumidity,
                             bool isFahrenheit)
{
    float hi;

    if (!isFahrenheit)
        temperature = convertCtoFglobal(temperature);

    hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) +
                (percentHumidity * 0.094));

    if (hi > 79)
    {
        hi = -42.379 + 2.04901523 * temperature + 10.14333127 * percentHumidity +
             -0.22475541 * temperature * percentHumidity +
             -0.00683783 * pow(temperature, 2) +
             -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature * pow(percentHumidity, 2) +
             -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

        if ((percentHumidity < 13) && (temperature >= 80.0) &&
            (temperature <= 112.0))
            hi -= ((13.0 - percentHumidity) * 0.25) *
                  sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

        else if ((percentHumidity > 85.0) && (temperature >= 80.0) &&
                 (temperature <= 87.0))
            hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
    }

    return isFahrenheit ? hi : convertFtoCglobal(hi);
}

bool dht22UpdateData()
{
    float h = dht22.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht22.readTemperature();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    // float f = dht22.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    // if (isnan(h) || isnan(t) || isnan(f))
    if (isnan(h) || isnan(t))
    {
        Serial.println(F("Failed to read from DHT sensor!"));
        return false;
    }

    // Compute heat index in Fahrenheit (the default)
    // float hif = dht22.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = computeHeatIndexGlobal(t, h, false);

    dht22_readings.temperature = t;
    dht22_readings.humidity = h;
    dht22_readings.heatIndex = hic;

    // Serial.print(F("Humidity: "));
    // Serial.print(h);
    // Serial.print(F("%  Temperature: "));
    // Serial.print(t);
    // Serial.print(F("°C "));
    // Serial.print(f);
    // Serial.print(F("°F  Heat index: "));
    // Serial.print(hic);
    // Serial.print(F("°C "));
    // Serial.print(hif);
    // Serial.println(F("°F"));

    return true;
}

bool bmpUpdateData()
{
    float t = bmp.readTemperature();
    float p = bmp.readPressure() / 100.0F;
    float alt = bmp.readAltitude();

    bmp280_readings.temperature = t;
    bmp280_readings.pressure = p;
    bmp280_readings.altitude = alt;
    // Serial.print("Temperature = ");
    // Serial.print(bmp.readTemperature());
    // Serial.println(" *C");
    // display.print("Temperature: ");
    // display.print(bmp.readTemperature());
    // display.println(" *C");

    // Serial.print("Pressure = ");
    // Serial.print(bmp.readPressure() / 100.0F);
    // Serial.println(" hPa");
    // display.print("Pressure: ");
    // display.print(bmp.readPressure() / 100.0F);
    // display.println(" hPa");

    return true;
}

bool ahtUpdateData()
{
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp); // populate temp and humidity objects with fresh data
    float hic = computeHeatIndexGlobal(temp.temperature, humidity.relative_humidity, false);
    
    aht21_readings.temperature = temp.temperature;
    aht21_readings.humidity = humidity.relative_humidity;
    aht21_readings.heatIndex = hic;
    
    // Serial.print("Temperature: ");
    // Serial.print(temp.temperature);
    // Serial.println(" degrees C");
    // Serial.print("Humidity: ");
    // Serial.print(humidity.relative_humidity);
    // Serial.println("% rH");
    return true;
}

void loop()
{
    display.setCursor(0, 0);
    display.clearDisplay();

    bmpUpdateData();
    dht22UpdateData();
    ahtUpdateData();

    Serial.println();
    display.display();
    delay(1000);
}