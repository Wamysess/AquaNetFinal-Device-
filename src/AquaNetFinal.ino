#include "arduino_secrets.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

#define ONE_WIRE_BUS 18         // Pin connected to DS18B20 sensor
#define TURBIDITY_SENSOR_PIN A0 // Analog pin connected to turbidity sensor
#define MQ135_SENSOR_PIN A6     // Analog pin connected to MQ-135 sensor
#define PH_SENSOR_PIN A7        // Analog pin connected to pH sensor (use A2 for D2)
#define SCREEN_WIDTH 128        // OLED display width, in pixels
#define SCREEN_HEIGHT 64        // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C     // Address of the OLED display

#define OLED_RESET -1           // Reset pin # (or -1 if sharing Arduino reset pin)
#define LED_PIN_WIFI 23         // LED pin for indicating WiFi connection

// Initialize OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Set up WiFi indicator LED pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

#define RL 47
#define b_ppm 1.5441
#define m_ppm -0.263

// Use the calibrated Ro value here
float Ro = 10.23; // Replace this value with the calibrated Ro

const char* ssid     = "private";
const char* password = "PLDCWIFIZsa9%";

const char* serverName = "https://aquanet.site/esp32data.php";

String apiKeyValue = "tPmAT5Ab3j7F9";

int pH_Pin = 35; // Analog pin connected to pH sensor
float Voltage;
float pH_Value;

float lastTemperature = 0;
String lastTurbidity = "";
float lastPH = 0;
float lastNH3 = 0;

// Calibration values
float calibph7 = 2.50; // Voltage at pH 7 calibration point
float calibph4 = 3.11; // Voltage at pH 4 calibration point
float m, b; // Slope and intercept for calibration

void setup() {
    Serial.begin(9600);
    pinMode(LED_PIN_WIFI, OUTPUT);

    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }

    // Clear the buffer
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);

    // Initialize DS18B20 temperature sensor
    sensors.begin();

    // Calculate slope and intercept for pH calibration
    m = (4.0 - 7.0) / (calibph4 - calibph7);
    b = 7.0 - m * calibph7;

    connectToWiFi(); // Attempt to connect to WiFi network
}

void loop() {
    // Check WiFi connection status
    if (WiFi.status() != WL_CONNECTED) {
        digitalWrite(LED_PIN_WIFI, LOW); // Turn off WiFi indicator LED
        connectToWiFi(); // Attempt to reconnect to WiFi
    } else {
        digitalWrite(LED_PIN_WIFI, HIGH); // Turn on WiFi indicator LED

        // Read temperature from DS18B20 sensor
        sensors.requestTemperatures(); // Request temperature readings
        float temperatureC = sensors.getTempCByIndex(0); // Get temperature in Celsius

        // Read turbidity from analog sensor
        int turbidityValue = analogRead(TURBIDITY_SENSOR_PIN);
        String turbidityLevel = getTurbidityLevel(turbidityValue);

        // Stabilize pH value
        pH_Value = stabilizePH();

        // Calculate PPM
        float ppm = calculateNH3();

        // Display sensor data on OLED
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Temp: ");
        display.print(temperatureC);
        display.println(" C");
        display.print("Clarity: ");
        display.println(turbidityLevel);
        // Display pH value
        display.print("pH Value: ");
        display.println(pH_Value, 2); // Display pH value with two decimal places
        // Display PPM value
        display.print("NH3 ppm: ");
        display.println(ppm);

        display.display();

        // Check if data has changed
        if (temperatureC != lastTemperature || turbidityLevel != lastTurbidity || pH_Value != lastPH || ppm != lastNH3) {
            // Update last readings
            lastTemperature = temperatureC;
            lastTurbidity = turbidityLevel;
            lastPH = pH_Value;
            lastNH3 = ppm;

            // Send sensor data to server
            sendSensorData(temperatureC, turbidityLevel, pH_Value, ppm); 
        }
    }
}

void connectToWiFi() {
    Serial.println("Connecting to WiFi...");
    digitalWrite(LED_PIN_WIFI, LOW); // Turn off WiFi indicator LED
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 5) { // Try to connect for up to 5 attempts
        delay(1000);
        Serial.println("Connecting to WiFi...");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected to WiFi");
        digitalWrite(LED_PIN_WIFI, HIGH); // Turn on WiFi indicator LED
    } else {
        Serial.println("Failed to connect to WiFi");
        Serial.println(WiFi.status());
        // You can add additional actions here if needed, such as resetting the ESP32 or trying a different network
    }
}

void sendSensorData(float temperature, String turbidity, float pH, float nh3) {
    WiFiClient client; // Use WiFiClient instead of WiFiClientSecure
    HTTPClient http;
    
    // Your Domain name with URL path or IP address with path
    http.begin(client, serverName);
    
    // Specify content-type header
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    
    // Prepare your HTTP POST request data
    String httpRequestData = "api_key=" + apiKeyValue 
                          + "&temperature=" + String(temperature)
                          + "&turbidity=" + turbidity
                          + "&phlevel=" + String(pH)
                          + "&nh3=" + String(nh3);
    Serial.print("HTTP Request Data: ");
    Serial.println(httpRequestData);

    // Send HTTP POST request
    int httpResponseCode = http.POST(httpRequestData);
     
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Temp: ");
    display.print(temperature);
    display.println(" C");
    display.print("Clarity: ");
    display.println(turbidity);
    display.print("pH Value: ");
    display.println(pH, 2); // Display pH value with two decimal places
    display.print("NH3 ppm: ");
    display.println(nh3);

    // Display status message
    display.setCursor(0, 54);
    display.setTextColor(SSD1306_WHITE);
    if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        display.println("Data Sent!");
    } else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
        display.println("Failed to send data");
    }
    display.display();

    // Free resources
    http.end();
    delay(1000); // Wait for 1 second before taking the next reading
}

String getTurbidityLevel(int value) {
    if (value == 0) {
        return "Very Muddy";
    } else if (value < 500) {
        return "Muddy";
    } else if (value < 1000) {
        return "Slightly Muddy";
    } else if (value < 1500) {
        return "Moderate";
    } else if (value < 2090) {
        return "Clear";   
    } else {
        return "Very Clear";
    }
}

float stabilizePH() {
    // Stabilize pH value
    const int numReadings = 20; // Number of readings to average
    float sumVoltage = 0.0;
    for (int i = 0; i < numReadings; i++) {
        int rawValue = analogRead(pH_Pin); // Read raw value from sensor
        sumVoltage += rawValue * (3.3 / 4095.0); // Convert raw value to voltage
        delay(10); // Delay between readings
    }
    Voltage = sumVoltage / numReadings; // Calculate average voltage

    // Apply calibration to calculate pH value
    return m * Voltage + b;
}

float calculateNH3() {
    float VRL; // Voltage drop across the MQ sensor
    float Rs; // Sensor resistance at gas concentration 
    float ratio; // Define variable for ratio
    
    // Read MQ-135 sensor value
    int mq135Value = analogRead(MQ135_SENSOR_PIN);

    // Convert sensor value to voltage
    VRL = mq135Value * (3.3 / 4095.0); // 3.3V is the operating voltage of ESP32
    
    // Calculate sensor resistance
    Rs = ((3.3 * RL) / VRL) - RL;
    
    // Calculate ratio Rs/Ro
    ratio = Rs / Ro;

    // Calculate ammonia concentration in ppm
    return pow(10, ((log10(ratio) - b_ppm) / m_ppm));
}
