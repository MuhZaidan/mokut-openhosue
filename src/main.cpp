#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>

// ---------------------------- SERVER --------------------------------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
String route1_g = "null", route2_g = "null", route3_g = "null";
// ---------------------------- MODULE -------------------------------------
// Buzzer 
#define PINBUZZER       32

// Ultrasonic (1)
#define PINECHO_US      35    // Input Only
#define PINTRIG_US      33

// Motorshield (2)
#define PINMOTOR_A      16
#define PINMOTOR_INP1   17
#define PINMOTOR_INP2   5 
#define PINMOTOR_B      21
#define PINMOTOR_INP3   18
#define PINMOTOR_INP4   19

// Linesensor (3)
#define PINLINESENSOR_A 25
#define PINLINESENSOR_B 26

// Color Sensor (4)
#define PINCOLOR_S0     15 // Kabel Warna abu
#define PINCOLOR_S1     27 // Kabel Warna oren
#define PINCOLOR_S2     12 // Kabel Warna coklat
#define PINCOLOR_S3     13 // Kabel Warna putih
#define PINCOLOR_OUT    14 // Kabel Warna kuning

// --------------------------- Initialize Function ------------------------------------------
void motorControl(int mode);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void pinInitialize();
void taskManager();
void gotoRouteDecision(int route);
String processor(const String& var);

// ------------------------- Real Time Variable ----------------------------------------------
long duration, distance;

volatile int redFrequency, 
            greenFrequency, 
            blueFrequency, 
            red, 
            green, 
            blue,
            lineSensorA,
            lineSensorB, count;
    
bool route1Status, route2Status, route3Status;
bool isRouteNumberOne = true;


TaskHandle_t handleUltrasonic = NULL;
TaskHandle_t handleLineSensor = NULL;
TaskHandle_t handleColorSensor = NULL;

// -------------------------- Real Time Task -------------------------------------------------
void taskUltrasonic(void *params){
  for(;;) {
    digitalWrite(PINTRIG_US, LOW);
    delayMicroseconds(2);
    digitalWrite(PINTRIG_US, HIGH);
    delayMicroseconds(10);
    digitalWrite(PINTRIG_US, LOW);

    duration = pulseIn(PINECHO_US, HIGH);
    distance = duration * 0.034 / 2; // Centimeters
    Serial.printf("Distance : %d\n", distance);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskLineSensor(void *params) {
  for(;;){
    lineSensorA = digitalRead(PINLINESENSOR_A);
    lineSensorB = digitalRead(PINLINESENSOR_B);

    Serial.printf("Line Sensor A : %d\n", lineSensorA);
    Serial.printf("Line Sensor B : %d\n", lineSensorB);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskColorSensor(void *params) {
  for(;;){
    // Red
    digitalWrite(PINCOLOR_S2,LOW);
    digitalWrite(PINCOLOR_S3,LOW);
    redFrequency = pulseIn(PINCOLOR_OUT, LOW);
    red = map(redFrequency, 50, 57, 255, 0);
    Serial.print(" R = ");
    Serial.println(redFrequency);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Green
    digitalWrite(PINCOLOR_S2,HIGH);
    digitalWrite(PINCOLOR_S3,HIGH);
    greenFrequency = pulseIn(PINCOLOR_OUT, LOW);
    green = map(greenFrequency, 80, 85, 255, 0);
    Serial.print(" G = ");
    Serial.println(greenFrequency);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // Blue
    digitalWrite(PINCOLOR_S2,LOW);
    digitalWrite(PINCOLOR_S3,HIGH);
    blueFrequency = pulseIn(PINCOLOR_OUT, LOW);
    blue = map(blueFrequency, 70, 76, 255, 0);
    Serial.print(" B = ");
    Serial.println(blueFrequency);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if(red > green && red > blue) {
      Serial.println("RED");
    }
    else if(green > red && green > blue) {
      Serial.println("GREEN");
    }
    else if(blue > red && blue > green) {
      Serial.println("BLUE");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Device Started/Restarted...");
  if(!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  WiFi.mode(WIFI_MODE_AP);
  WiFi.softAP("MoKut-ESP32", "123456789");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: ");
  Serial.println(IP);

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", "text/css");
  });

  server.on("/main.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/main.js", "text/javascript");
  });

  pinInitialize();
  taskManager();
  isRouteNumberOne = true;
  server.begin();
}

void loop() {
  // if(route1_g != "null" && route2_g != "null" && route3_g != "null") {
  //   if(red > green && red > blue) {
  //     motorControl(0);
  //     delay(1000);
  //   } else if(green > red && green > blue) {
  //     motorControl(1);
  //     delay(1000);
  //   } else if(blue > red && blue > green) {
  //     motorControl(2);
  //     delay(1000);
  //   }
  // }

  if(distance <= 20){
    tone(PINBUZZER, 999);
    motorControl(0);
  } else {
    noTone(PINBUZZER);
    // if(route1_g == "Route 1") gotoRouteDecision(1);
    if(route1_g == "Route 2") gotoRouteDecision(2);
    // else if(route1_g == "Route 3") gotoRouteDecision(3);
    // motorControl(1);
  }
  delay(200);
}

void gotoRouteDecision(int route) {
  bool redStatus = red > green && red > blue ? true : false;
  bool greenStatus = green > red && green > blue ? true : false;
  bool blueStatus = blue > red && blue > green ? true : false;

  if(route == 1) {
    if(greenStatus) {
      if(lineSensorA == LOW && lineSensorB == LOW) {
        motorControl(1);
      } else if(lineSensorA == HIGH && lineSensorB == HIGH) {
        motorControl(2);
      }
    }
    if(redStatus) {
      if(lineSensorA == LOW && lineSensorB == LOW) {
        motorControl(1);
      } else if(lineSensorA == HIGH && lineSensorB == HIGH) {
        motorControl(0);
      } else if(lineSensorA == HIGH && lineSensorB == LOW) {
        motorControl(3);
      }
    }
  } else if(route == 2) {
    if(greenStatus) {
      if(lineSensorA == LOW && lineSensorB == LOW) {
        motorControl(1);
        count = 1;
      } else if(lineSensorA == HIGH && lineSensorB == HIGH && count == 1) {
        motorControl(1);
        count = 2;
      } else if(lineSensorA == HIGH && lineSensorB == HIGH && count == 2) {
        motorControl(0);
        count = 0;
      }
    }
  } else if(route == 3) {
    if(greenStatus) {
      if(lineSensorA == LOW && lineSensorB == LOW) {
        motorControl(1);
      } else if(lineSensorA == HIGH && lineSensorB == HIGH) {
        motorControl(3);
      }
    }
    if(blueStatus) {
      if(lineSensorA == LOW && lineSensorB == LOW) {
        motorControl(1);
      } else if(lineSensorA == HIGH && lineSensorB == HIGH) {
        motorControl(0);
      } else if(lineSensorA == LOW && lineSensorB == HIGH) {
        motorControl(2);
      }
    }
  }
  return;
}

void motorControl(int mode) {
  Serial.println("Motor Control....");

  if(mode == 0){                      // STOP
    analogWrite(PINMOTOR_A, 0);
    analogWrite(PINMOTOR_B, 0);
    digitalWrite(PINMOTOR_INP1, LOW);
    digitalWrite(PINMOTOR_INP2, LOW);
    digitalWrite(PINMOTOR_INP3, LOW);
    digitalWrite(PINMOTOR_INP4, LOW);
  }
  else if(mode == 1){                 // FORWARD
    analogWrite(PINMOTOR_A, 150);
    analogWrite(PINMOTOR_B, 150);
    digitalWrite(PINMOTOR_INP1, HIGH);
    digitalWrite(PINMOTOR_INP2, LOW);
    digitalWrite(PINMOTOR_INP3, HIGH);
    digitalWrite(PINMOTOR_INP4, LOW);
  }
  else if(mode == 2){               // TURN LEFT
    analogWrite(PINMOTOR_A, 150);
    analogWrite(PINMOTOR_B, 0);
    digitalWrite(PINMOTOR_INP1, HIGH);
    digitalWrite(PINMOTOR_INP2, LOW);
    digitalWrite(PINMOTOR_INP3, LOW);
    digitalWrite(PINMOTOR_INP4, LOW);
  }
  else if(mode == 3){               // TURN RIGHT
    analogWrite(PINMOTOR_A, 0);
    analogWrite(PINMOTOR_B, 150);
    digitalWrite(PINMOTOR_INP1, LOW);
    digitalWrite(PINMOTOR_INP2, LOW);
    digitalWrite(PINMOTOR_INP3, LOW);
    digitalWrite(PINMOTOR_INP4, HIGH);
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if(info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    String msg = "";
    for(size_t i = 0; i < len; i++) {
      msg += (char)data[i];
    }
    Serial.println("JSON Received: " + msg);
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, msg);
    if(error) {
      Serial.println("Fail to Parsing JSON");
      return;
    }

    String route1 = doc["route1"];
    String route2 = doc["route2"];
    String route3 = doc["route3"];
    if(route1 == "null" && route2 == "null" && route3 == "null") {
      route1Status, route2Status, route3Status = false;
    }

    route1_g = route1;
    route2_g = route2;
    route3_g = route3;
    route1Status = true;
    route2Status = true;
    route3Status = true;

    Serial.println("Route 1 : " + route1_g);
    Serial.println("Route 2 : " + route2_g);
    Serial.println("Route 3 : " + route3_g);
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}

String processor(const String& var){
  Serial.println(var);
  return String();
}

void pinInitialize() {
  Serial.println("GPIO PIN Has Initialized...");

  // Motorshield
  pinMode(PINMOTOR_A, OUTPUT);
  pinMode(PINMOTOR_INP1, OUTPUT);
  pinMode(PINMOTOR_INP2, OUTPUT);
  pinMode(PINMOTOR_B, OUTPUT);
  pinMode(PINMOTOR_INP3, OUTPUT);
  pinMode(PINMOTOR_INP4, OUTPUT);

  digitalWrite(PINMOTOR_INP1, LOW);
  digitalWrite(PINMOTOR_INP2, LOW);
  digitalWrite(PINMOTOR_INP3, LOW);
  digitalWrite(PINMOTOR_INP4, LOW);

  // Buzzer
  pinMode(PINBUZZER, OUTPUT);

  // Ultrasonic (US)
  pinMode(PINTRIG_US, OUTPUT);
  pinMode(PINECHO_US, INPUT);

  // Linesensor
  pinMode(PINLINESENSOR_A, INPUT);
  pinMode(PINLINESENSOR_B, INPUT);

  // Color Sensor
  pinMode(PINCOLOR_S0, OUTPUT);
  pinMode(PINCOLOR_S1, OUTPUT);
  pinMode(PINCOLOR_S2, OUTPUT);
  pinMode(PINCOLOR_S3, OUTPUT);
  pinMode(PINCOLOR_OUT, INPUT);
  
  digitalWrite(PINCOLOR_S0, HIGH);
  digitalWrite(PINCOLOR_S1, LOW);
  return;
}

void taskManager() {
  Serial.println("RealTime Task Manager has Running...");

  xTaskCreate(
    taskUltrasonic, "Task Ultrasonic", 2048, NULL, 1, &handleUltrasonic
  );
  xTaskCreate(
    taskLineSensor, "Task LineSensor", 2048, NULL, 2, &handleLineSensor
  );
  xTaskCreate(
    taskColorSensor, "Task ColorSensor", 4096, NULL, 2, &handleColorSensor
  );
}