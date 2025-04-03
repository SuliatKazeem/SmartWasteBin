#define BLYNK_TEMPLATE_ID "TMPL5FCg1G_fK"
#define BLYNK_TEMPLATE_NAME "Smart Bin Template"
#define BLYNK_DEVICE_NAME "Smart Bin"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32_SSL.h>
#include <ESP32Servo.h>
#include <HCSR04.h>
#include <DFRobot_HX711.h>
#include <PubSubClient.h>


char ssid[] = "************";
char pass[] = "************";
char auth[] = "*********************************";

#define MQTT_BROKER "test.mosquitto.org"
#define MQTT_PORT (1883)

WiFiClient wifiClient;
PubSubClient client(wifiClient);
boolean touchFlag = false;

const int trigPin1 = 5;  // Waste level
const int echoPin1 = 18;
const int trigPin2 = 23;  // Intent detection
const int echoPin2 = 22;

// Pin for the servo motor
Servo myServo;
int pos = 0;  // variable to store the servo position
int servoPin = 14;

// Pins for HX711 strain gauge
const int LOADCELL_DOUT_PIN = 17;
const int LOADCELL_SCK_PIN = 21;

// Create ultrasonic and strain gauge objects
UltraSonicDistanceSensor wasteSensor(trigPin1, echoPin1);
UltraSonicDistanceSensor intentSensor(trigPin2, echoPin2);

/*!
 * @fn DFRobot_HX711
 * @brief Constructor 
 * @param pin_din  Analog data pin
 * @param pin_slk  Analog data pin
 */
DFRobot_HX711 MyScale(17, 21);

// Variables
unsigned long startTime = 0;
bool isIntentDetected = false;
float wasteDistance = 0.0;
float weight = 0.0;
const float weightEmpty = 50.0; 
const float weightHalfFull = 500.0; 
const float weightFull = 1000.0; 

// Virtual pins for blynk
const int VPIN_BIN_FULLNESS = V1;
const int VPIN_WEIGHT = V2;
const int VPIN_HEIGHT = V3;
const int VPIN_BIN_STATUS = V4;
const int VPIN_SYSTEM_STATUS = V5;

#define MQTT_TOPIC_WASTELEVEL "uok/iot/sk993/smartwastebin/wastelevel"
#define MQTT_TOPIC_WEIGHT "uok/iot/sk993/smartwastebin/weight"

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started...");

  Blynk.begin(auth, ssid, pass);
  Serial.println("Connected to WiFi & Blynk");

  WiFi.begin(ssid, pass);
  client.setServer(MQTT_BROKER, MQTT_PORT);
  Serial.println("Connected to MQTT");
  
  myServo.setPeriodHertz(50);           
  myServo.attach(servoPin, 600, 2400);
  closeLid();

  Serial.print(MyScale.readWeight(), 1);
  Serial.println(" g");
  delay(500);

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void loop() {
  Blynk.run();
  mqttLoop();

  checkIntentAndOpenLid();
  measureAndReport();
  client.loop();
  delay(100);
}

void mqttLoop() {
  if (!client.connected()) {
    while (!client.connected()) {
      Serial.print("Attempting MQTT connection...");
      if (client.connect("ESP32Client")) {

        client.publish(MQTT_TOPIC_WASTELEVEL, String(wasteDistance).c_str());
        client.publish(MQTT_TOPIC_WEIGHT, String(weight).c_str());

      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        delay(5000);
      }
    }
  }

  client.loop();
}

void checkIntentAndOpenLid() {
  float intentDistance = intentSensor.measureDistanceCm();
  if (intentDistance < 20 && intentDistance >= 0) {
    if (!isIntentDetected) {
      isIntentDetected = true;
      startTime = millis();
    } else if (millis() - startTime >= 5000) {
      Serial.println("Bin Open");
      openLid();  // Open the lid

      Blynk.virtualWrite(VPIN_BIN_STATUS, 1);  // Update Blynk - bin open
      delay(5000);  // Keep lid open for 5secs

      closeLid();  // Close the lid
      Serial.println("Bin Close");

      Blynk.virtualWrite(VPIN_BIN_STATUS, 0);  // Update Blynk - bin closed
      isIntentDetected = false;
    }
  } else {
    isIntentDetected = false;
  }
}

void measureAndReport() {
  wasteDistance = wasteSensor.measureDistanceCm();
  weight = -MyScale.readWeight(); // calibration (-1)
  Serial.print("Waste Level: ");
  Serial.print(wasteDistance);
  Serial.println(" cm");
  Serial.print("Weight: ");
  Serial.print(weight);
  Serial.println("g");

  // Check for waste level and weight
  if (wasteDistance <= 5 && weight >= weightFull) {
    Serial.println("Bin Full");
    Blynk.virtualWrite(VPIN_BIN_FULLNESS, 100);
  } else if (wasteDistance <= 5 && weight >= weightHalfFull) {
    Serial.println("Bin Nearly Full");
    Blynk.virtualWrite(VPIN_BIN_FULLNESS, 75);
  } else if ((wasteDistance > 5 && wasteDistance <= 15) && weight >= weightFull) {
    Serial.println("Bin Half Full & Heavy");
    Blynk.virtualWrite(VPIN_BIN_FULLNESS, 75);
  } else if ((wasteDistance > 5 && wasteDistance <= 15) && weight >= weightHalfFull) {
    Serial.println("Bin Half Full");
    Blynk.virtualWrite(VPIN_BIN_FULLNESS, 50);
  } else if ((wasteDistance > 15 && wasteDistance <= 25) && weight >= weightEmpty && weight < weightHalfFull) {
    Serial.println("Object Expansion Likely");
    Blynk.virtualWrite(VPIN_BIN_FULLNESS, 25);
  } else {
    Serial.println("Bin Empty or Nearly Empty");
    Blynk.virtualWrite(VPIN_BIN_FULLNESS, 0);
  }

  Blynk.virtualWrite(VPIN_HEIGHT, wasteDistance);
  Blynk.virtualWrite(VPIN_WEIGHT, weight);
}

void openLid() {
  for (pos = 0; pos <= 180; pos += 1) {  // from 0 to 180 
    myServo.write(pos);  
    delay(15);            // waits 15 ms 
  }
}

void closeLid() {
  for (pos = 180; pos >= 0; pos -= 1) {  // from 180 to 0
    myServo.write(pos); 
    delay(15);                          
}
}
