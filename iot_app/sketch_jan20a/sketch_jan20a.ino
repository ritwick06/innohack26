/*
 * Enhanced ESP32 MQTT Bridge for Water Surface Cleaning Robot
 * Fixed version with proper error handling
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi Credentials - CHANGE THESE FOR YOUR NETWORK
const char* ssid = "Nokia G42 5G";
const char* password = "ritwick69420";

// MQTT Broker settings
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* topic_prefix = "robot/water";

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

// Data structures
struct SensorData {
  // Battery data
  float batteryLevel;
  bool solarCharging;
  float voltage;
  float temperature;
  float chargeCurrent;
  
  // GPS data
  float latitude;
  float longitude;
  float altitude;
  float heading;
  
  // Status data
  String mode;
  float lidar;
  float speed;
  float motorTemp;
  
  // Sensor readings
  float waterTemp;
  float humidity;
  float pressure;
  int rpmLeft;
  int rpmRight;
  String waterQuality;
  
  // Mission progress
  float areaCovered;
  float trashCollected;
  int cellsCovered;
  int totalCells;
  float coveragePercent;
  
  // Environmental data
  float windSpeed;
  float waterCurrent;
  float waveHeight;
  String trashDensity;
};

struct PathPlanning {
  float areaLat[20];
  float areaLng[20];
  int areaPoints;
  bool areaSet;
  float cellSize; // meters
  float path[100][2]; // Path points
  int pathPoints;
  int currentPathIndex;
  bool missionActive;
  unsigned long missionStartTime;
};

SensorData sensorData;
PathPlanning pathPlan;
unsigned long lastPublish = 0;
const long publishInterval = 2000; // Publish every 2 seconds
unsigned long lastEnvironmentUpdate = 0;
const long environmentInterval = 5000; // Update environment every 5 seconds
unsigned long lastBatteryUpdate = 0;
const long batteryInterval = 10000; // Update battery every 10 seconds
unsigned long lastSerialRead = 0;
const long serialReadInterval = 100; // Read serial every 100ms

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Enhanced Water Robot ESP32 MQTT Bridge ===");
  Serial.println("Starting...");
  
  // Initialize data structures
  initializeSensorData();
  initializePathPlanning();
  
  // Connect to WiFi
  setupWiFi();
  
  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  
  Serial.println("Setup complete!");
  Serial.println("Starting main loop...");
}

void loop() {
  // Maintain MQTT connection
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  
  // Check for data from MATLAB via Serial with timing
  unsigned long now = millis();
  if (now - lastSerialRead > serialReadInterval) {
    lastSerialRead = now;
    readMATLABData();
  }
  
  // Publish data at regular intervals
  if (now - lastPublish > publishInterval) {
    lastPublish = now;
    publishAllData();
  }
  
  // Update environment simulation periodically
  if (now - lastEnvironmentUpdate > environmentInterval) {
    lastEnvironmentUpdate = now;
    updateEnvironmentSimulation();
  }
  
  // Update battery simulation
  if (now - lastBatteryUpdate > batteryInterval) {
    lastBatteryUpdate = now;
    updateBatterySimulation();
  }
  
  // Update mission progress if active
  if (pathPlan.missionActive) {
    updateMissionProgress();
  }
  
  delay(10); // Small delay to prevent watchdog timer issues
}

void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("WiFi connection failed!");
    // Try to reconnect after delay
    delay(5000);
    ESP.restart();
  }
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Create a unique client ID
    String clientId = "ESP32WaterRobot-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("connected!");
      
      // Subscribe to topics
      String cmdTopic = String(topic_prefix) + "/commands";
      client.subscribe(cmdTopic.c_str());
      
      String planTopic = String(topic_prefix) + "/planning";
      client.subscribe(planTopic.c_str());
      
      Serial.println("Subscribed to commands and planning topics");
      
      // Publish connection status
      publishAlert("ESP32 connected", "success");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  Serial.print("Message received on topic: ");
  Serial.println(topicStr);
  
  // Convert payload to string
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';
  
  Serial.print("Message: ");
  Serial.println(message);
  
  // Check if it's a planning message
  if (topicStr.endsWith("/planning")) {
    handlePlanningMessage(message);
    return;
  }
  
  // Check if it's a command message
  if (topicStr.endsWith("/commands")) {
    handleCommandMessage(message);
    return;
  }
}

void handlePlanningMessage(String message) {
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("Failed to parse planning message: ");
    Serial.println(error.c_str());
    return;
  }
  
  String type = doc["type"];
  
  if (type == "OPERATION_AREA") {
    JsonArray coords = doc["coordinates"];
    pathPlan.areaPoints = 0;
    
    // Store area coordinates
    for (JsonVariant point : coords) {
      if (pathPlan.areaPoints < 20) {
        pathPlan.areaLat[pathPlan.areaPoints] = point["lat"];
        pathPlan.areaLng[pathPlan.areaPoints] = point["lng"];
        pathPlan.areaPoints++;
      }
    }
    
    pathPlan.areaSet = true;
    
    // Generate boustrophedon path
    generateBoustrophedonPath();
    
    Serial.println("Operation area received and path generated");
    Serial.print("Area points: ");
    Serial.println(pathPlan.areaPoints);
    Serial.print("Path points: ");
    Serial.println(pathPlan.pathPoints);
    
    // Send acknowledgement to MATLAB
    sendAreaToMATLAB();
    
    // Publish alert
    publishAlert("Operation area set and path planned", "success");
  }
}

void handleCommandMessage(String message) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("Failed to parse command message: ");
    Serial.println(error.c_str());
    return;
  }
  
  String command = doc["command"];
  Serial.print("Command: ");
  Serial.println(command);
  
  // Handle commands
  if (command == "START_MISSION") {
    if (pathPlan.areaSet) {
      pathPlan.missionActive = true;
      pathPlan.missionStartTime = millis();
      pathPlan.currentPathIndex = 0;
      sensorData.mode = "COVERING";
      
      // Update total cells
      sensorData.totalCells = pathPlan.pathPoints;
      sensorData.cellsCovered = 0;
      sensorData.coveragePercent = 0;
      
      Serial.println("Mission started!");
      publishAlert("Mission started successfully", "success");
      
      // Send command to MATLAB
      Serial.println("CMD:START_MISSION");
    } else {
      publishAlert("No operation area set. Please draw area first.", "warning");
    }
    
  } else if (command == "STOP") {
    pathPlan.missionActive = false;
    sensorData.mode = "IDLE";
    Serial.println("CMD:STOP");
    publishAlert("Emergency stop activated", "error");
    
  } else if (command == "DOCK") {
    pathPlan.missionActive = false;
    sensorData.mode = "DOCKING";
    Serial.println("CMD:DOCK");
    publishAlert("Returning to dock", "info");
    
  } else if (command == "RESET") {
    initializeSensorData();
    initializePathPlanning();
    Serial.println("CMD:RESET");
    publishAlert("System reset complete", "success");
    
  } else if (command == "PAUSE") {
    if (sensorData.mode == "COVERING") {
      sensorData.mode = "IDLE";
      publishAlert("Mission paused", "warning");
    } else if (sensorData.mode == "IDLE" && pathPlan.missionActive) {
      sensorData.mode = "COVERING";
      publishAlert("Mission resumed", "success");
    }
    Serial.print("CMD:PAUSE:");
    Serial.println(sensorData.mode);
    
  } else if (command == "MANUAL_CONTROL") {
    String direction = doc["direction"];
    Serial.print("CMD:MANUAL:");
    Serial.println(direction);
    publishAlert("Manual control: " + direction, "info");
  }
}

void readMATLABData() {
  while (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    if (data.length() == 0) continue;
    
    // Expected format from MATLAB: JSON data
    if (data.startsWith("{")) {
      StaticJsonDocument<1024> doc;
      DeserializationError error = deserializeJson(doc, data);
      
      if (error) {
        Serial.print("Failed to parse MATLAB data: ");
        Serial.println(error.c_str());
        Serial.print("Data: ");
        Serial.println(data);
        return;
      }
      
      // Parse battery data
      if (doc.containsKey("battery")) sensorData.batteryLevel = doc["battery"];
      if (doc.containsKey("voltage")) sensorData.voltage = doc["voltage"];
      if (doc.containsKey("temp")) sensorData.temperature = doc["temp"];
      if (doc.containsKey("solar")) sensorData.solarCharging = doc["solar"];
      if (doc.containsKey("chargeCurrent")) sensorData.chargeCurrent = doc["chargeCurrent"];
      
      // Parse GPS data
      if (doc.containsKey("lat")) sensorData.latitude = doc["lat"];
      if (doc.containsKey("lng")) sensorData.longitude = doc["lng"];
      if (doc.containsKey("alt")) sensorData.altitude = doc["alt"];
      if (doc.containsKey("heading")) sensorData.heading = doc["heading"];
      
      // Parse status
      if (doc.containsKey("mode")) sensorData.mode = doc["mode"].as<String>();
      if (doc.containsKey("lidar")) sensorData.lidar = doc["lidar"];
      if (doc.containsKey("speed")) sensorData.speed = doc["speed"];
      if (doc.containsKey("motorTemp")) sensorData.motorTemp = doc["motorTemp"];
      
      // Parse sensors
      if (doc.containsKey("waterTemp")) sensorData.waterTemp = doc["waterTemp"];
      if (doc.containsKey("humidity")) sensorData.humidity = doc["humidity"];
      if (doc.containsKey("pressure")) sensorData.pressure = doc["pressure"];
      if (doc.containsKey("rpmLeft")) sensorData.rpmLeft = doc["rpmLeft"];
      if (doc.containsKey("rpmRight")) sensorData.rpmRight = doc["rpmRight"];
      if (doc.containsKey("waterQuality")) sensorData.waterQuality = doc["waterQuality"].as<String>();
      
      // Parse progress
      if (doc.containsKey("area")) sensorData.areaCovered = doc["area"];
      if (doc.containsKey("trash")) sensorData.trashCollected = doc["trash"];
      if (doc.containsKey("cellsCovered")) sensorData.cellsCovered = doc["cellsCovered"];
      if (doc.containsKey("totalCells")) sensorData.totalCells = doc["totalCells"];
      if (doc.containsKey("coveragePercent")) sensorData.coveragePercent = doc["coveragePercent"];
      
      // Parse environment
      if (doc.containsKey("environment")) {
        JsonObject env = doc["environment"];
        if (env.containsKey("windSpeed")) sensorData.windSpeed = env["windSpeed"];
        if (env.containsKey("waterCurrent")) sensorData.waterCurrent = env["waterCurrent"];
        if (env.containsKey("waveHeight")) sensorData.waveHeight = env["waveHeight"];
        if (env.containsKey("trashDensity")) sensorData.trashDensity = env["trashDensity"].as<String>();
      }
      
      // Debug: Print received data
      if (millis() % 10000 < 100) { // Every 10 seconds
        Serial.print("Data from MATLAB - Mode: ");
        Serial.print(sensorData.mode);
        Serial.print(", Lat: ");
        Serial.print(sensorData.latitude, 6);
        Serial.print(", Lng: ");
        Serial.print(sensorData.longitude, 6);
        Serial.print(", Cells: ");
        Serial.print(sensorData.cellsCovered);
        Serial.print("/");
        Serial.println(sensorData.totalCells);
      }
    } else if (data.startsWith("CMD:")) {
      // Command from MATLAB (for debugging)
      Serial.println("Received from MATLAB: " + data);
    }
  }
}

void sendAreaToMATLAB() {
  StaticJsonDocument<1024> doc;
  doc["type"] = "AREA_CONFIRMATION";
  
  JsonArray areaArray = doc.createNestedArray("coordinates");
  for (int i = 0; i < pathPlan.areaPoints; i++) {
    JsonObject point = areaArray.createNestedObject();
    point["lat"] = pathPlan.areaLat[i];
    point["lng"] = pathPlan.areaLng[i];
  }
  
  doc["totalCells"] = pathPlan.pathPoints;
  doc["cellSize"] = pathPlan.cellSize;
  
  String output;
  serializeJson(doc, output);
  
  Serial.println(output);
}

void generateBoustrophedonPath() {
  if (!pathPlan.areaSet || pathPlan.areaPoints < 3) return;
  
  // Simple boustrophedon path generation
  // Find bounding box
  float minLat = pathPlan.areaLat[0];
  float maxLat = pathPlan.areaLat[0];
  float minLng = pathPlan.areaLng[0];
  float maxLng = pathPlan.areaLng[0];
  
  for (int i = 1; i < pathPlan.areaPoints; i++) {
    if (pathPlan.areaLat[i] < minLat) minLat = pathPlan.areaLat[i];
    if (pathPlan.areaLat[i] > maxLat) maxLat = pathPlan.areaLat[i];
    if (pathPlan.areaLng[i] < minLng) minLng = pathPlan.areaLng[i];
    if (pathPlan.areaLng[i] > maxLng) maxLng = pathPlan.areaLng[i];
  }
  
  // Generate grid points
  float latStep = 0.000045; // ~5 meters in latitude
  float lngStep = 0.000045; // ~5 meters in longitude
  
  pathPlan.pathPoints = 0;
  bool forward = true;
  
  for (float lat = minLat + latStep/2; lat < maxLat; lat += latStep) {
    if (forward) {
      for (float lng = minLng + lngStep/2; lng < maxLng; lng += lngStep) {
        if (isPointInPolygon(lat, lng) && pathPlan.pathPoints < 100) {
          pathPlan.path[pathPlan.pathPoints][0] = lat;
          pathPlan.path[pathPlan.pathPoints][1] = lng;
          pathPlan.pathPoints++;
        }
      }
    } else {
      for (float lng = maxLng - lngStep/2; lng > minLng; lng -= lngStep) {
        if (isPointInPolygon(lat, lng) && pathPlan.pathPoints < 100) {
          pathPlan.path[pathPlan.pathPoints][0] = lat;
          pathPlan.path[pathPlan.pathPoints][1] = lng;
          pathPlan.pathPoints++;
        }
      }
    }
    forward = !forward;
  }
  
  pathPlan.cellSize = 5.0; // meters
  sensorData.totalCells = pathPlan.pathPoints;
}

bool isPointInPolygon(float lat, float lng) {
  // Simple point-in-polygon test (ray casting)
  bool inside = false;
  
  for (int i = 0, j = pathPlan.areaPoints - 1; i < pathPlan.areaPoints; j = i++) {
    if (((pathPlan.areaLat[i] > lat) != (pathPlan.areaLat[j] > lat)) &&
        (lng < (pathPlan.areaLng[j] - pathPlan.areaLng[i]) * (lat - pathPlan.areaLat[i]) / 
         (pathPlan.areaLat[j] - pathPlan.areaLat[i]) + pathPlan.areaLng[i])) {
      inside = !inside;
    }
  }
  
  return inside;
}

void updateMissionProgress() {
  if (!pathPlan.missionActive || pathPlan.currentPathIndex >= pathPlan.pathPoints) {
    if (pathPlan.missionActive && pathPlan.currentPathIndex >= pathPlan.pathPoints) {
      // Mission complete
      pathPlan.missionActive = false;
      sensorData.mode = "DOCKING";
      publishAlert("Mission complete! Returning to dock.", "success");
    }
    return;
  }
  
  // Move to next cell periodically
  static unsigned long lastMove = 0;
  if (millis() - lastMove > 3000) { // Move every 3 seconds
    lastMove = millis();
    
    // Update current position to next path point
    if (pathPlan.currentPathIndex < pathPlan.pathPoints) {
      sensorData.latitude = pathPlan.path[pathPlan.currentPathIndex][0];
      sensorData.longitude = pathPlan.path[pathPlan.currentPathIndex][1];
      pathPlan.currentPathIndex++;
      sensorData.cellsCovered = pathPlan.currentPathIndex;
      sensorData.coveragePercent = (sensorData.cellsCovered * 100.0) / sensorData.totalCells;
      
      // Update area covered
      sensorData.areaCovered += 25.0; // 5x5 meter cell
      
      // Random trash collection
      if (random(100) < 30) { // 30% chance per cell
        sensorData.trashCollected += random(5, 20) / 10.0; // 0.5-2.0 kg
      }
    }
  }
}

void updateBatterySimulation() {
  // Only simulate if we're not getting data from MATLAB
  static unsigned long lastMATLABData = 0;
  if (millis() - lastMATLABData > 5000) {
    // No MATLAB data for 5 seconds, do simulation
    if (sensorData.mode == "IDLE" && sensorData.solarCharging) {
      // Charging at dock
      sensorData.batteryLevel = (sensorData.batteryLevel + 1.0f > 100.0f) ? 100.0f : sensorData.batteryLevel + 1.0f;
      sensorData.chargeCurrent = 2.5;
    } else if (sensorData.mode == "COVERING" || sensorData.mode == "MOVING") {
      // Discharging during mission
      float dischargeRate = 0.5f + (sensorData.speed / 2.0f);
      float newBattery = sensorData.batteryLevel - dischargeRate;
      sensorData.batteryLevel = (newBattery < 0.0f) ? 0.0f : newBattery;
      sensorData.chargeCurrent = 0.0;
    }
    
    // Update voltage based on battery level
    sensorData.voltage = 11.0f + (sensorData.batteryLevel / 100.0f) * 2.0f;
  } else {
    // We're getting data from MATLAB, use it directly
    lastMATLABData = millis();
  }
}

void updateEnvironmentSimulation() {
  // Only simulate if we're not getting data from MATLAB
  static unsigned long lastMATLABEnvData = 0;
  if (millis() - lastMATLABEnvData > 10000) {
    // Simulate environmental changes
    sensorData.windSpeed = random(0, 120) / 10.0; // 0-12 m/s
    sensorData.waterCurrent = random(0, 30) / 10.0; // 0-3 m/s
    sensorData.waveHeight = random(0, 30) / 100.0; // 0-0.3 m
    
    // Trash density (changes slowly)
    static int trashCounter = 0;
    if (trashCounter++ > 10) {
      trashCounter = 0;
      int density = random(3);
      switch(density) {
        case 0: sensorData.trashDensity = "Low"; break;
        case 1: sensorData.trashDensity = "Medium"; break;
        case 2: sensorData.trashDensity = "High"; break;
      }
    }
    
    // Update water temperature based on air temp
    sensorData.waterTemp = sensorData.temperature - 2.0 + (random(0, 40) / 10.0);
    sensorData.humidity = 60 + random(-20, 30);
    sensorData.pressure = 1013 + random(-20, 20);
    
    // Simulate LIDAR readings
    sensorData.lidar = 5.0 + random(0, 450) / 10.0;
    
    // Simulate motor RPM
    if (sensorData.mode == "COVERING") {
      sensorData.rpmLeft = 800 + random(-100, 100);
      sensorData.rpmRight = 800 + random(-100, 100);
      sensorData.speed = 0.5 + random(0, 50) / 100.0;
      sensorData.motorTemp = 35 + random(0, 20);
    } else if (sensorData.mode == "IDLE") {
      sensorData.rpmLeft = 0;
      sensorData.rpmRight = 0;
      sensorData.speed = 0;
      sensorData.motorTemp = 25 + random(0, 10);
    }
    
    // Water quality simulation
    int quality = random(3);
    switch(quality) {
      case 0: sensorData.waterQuality = "Good"; break;
      case 1: sensorData.waterQuality = "Fair"; break;
      case 2: sensorData.waterQuality = "Poor"; break;
    }
  } else {
    lastMATLABEnvData = millis();
  }
}

void publishAllData() {
  if (!client.connected()) return;
  
  publishBatteryData();
  publishGPSData();
  publishStatusData();
  publishSensorData();
  publishProgressData();
  publishEnvironmentData();
  publishCoverageData();
  
  // Check for alerts
  if (sensorData.batteryLevel < 20) {
    publishAlert("Low battery: " + String(sensorData.batteryLevel, 0) + "%", "warning");
  }
  
  if (sensorData.batteryLevel < 10) {
    publishAlert("Critical battery! Return to dock immediately.", "error");
  }
  
  if (sensorData.motorTemp > 60) {
    publishAlert("Motor overheating: " + String(sensorData.motorTemp, 0) + "°C", "warning");
  }
  
  if (sensorData.waterQuality == "Poor") {
    publishAlert("Poor water quality detected", "warning");
  }
}

void publishBatteryData() {
  StaticJsonDocument<256> doc;
  doc["level"] = sensorData.batteryLevel;
  doc["charging"] = sensorData.solarCharging;
  doc["voltage"] = sensorData.voltage;
  doc["temperature"] = sensorData.temperature;
  doc["current"] = sensorData.chargeCurrent;
  
  String output;
  serializeJson(doc, output);
  
  String topic = String(topic_prefix) + "/battery";
  client.publish(topic.c_str(), output.c_str());
}

void publishGPSData() {
  StaticJsonDocument<256> doc;
  doc["lat"] = sensorData.latitude;
  doc["lng"] = sensorData.longitude;
  doc["alt"] = sensorData.altitude;
  doc["heading"] = sensorData.heading;
  doc["mode"] = sensorData.mode;
  doc["speed"] = sensorData.speed;
  
  String output;
  serializeJson(doc, output);
  
  String topic = String(topic_prefix) + "/gps";
  client.publish(topic.c_str(), output.c_str());
}

void publishStatusData() {
  StaticJsonDocument<256> doc;
  doc["mode"] = sensorData.mode;
  doc["lidar"] = sensorData.lidar;
  doc["speed"] = sensorData.speed;
  doc["motorTemp"] = sensorData.motorTemp;
  
  // Include coverage info
  JsonObject coverage = doc.createNestedObject("coverage");
  coverage["percent"] = sensorData.coveragePercent;
  coverage["cellsCovered"] = sensorData.cellsCovered;
  coverage["totalCells"] = sensorData.totalCells;
  
  String output;
  serializeJson(doc, output);
  
  String topic = String(topic_prefix) + "/status";
  client.publish(topic.c_str(), output.c_str());
}

void publishSensorData() {
  StaticJsonDocument<256> doc;
  doc["waterTemp"] = sensorData.waterTemp;
  doc["humidity"] = sensorData.humidity;
  doc["pressure"] = sensorData.pressure;
  doc["rpmLeft"] = sensorData.rpmLeft;
  doc["rpmRight"] = sensorData.rpmRight;
  doc["waterQuality"] = sensorData.waterQuality;
  
  String output;
  serializeJson(doc, output);
  
  String topic = String(topic_prefix) + "/sensors";
  client.publish(topic.c_str(), output.c_str());
}

void publishProgressData() {
  StaticJsonDocument<256> doc;
  doc["area"] = sensorData.areaCovered;
  doc["trash"] = sensorData.trashCollected;
  doc["cellsCovered"] = sensorData.cellsCovered;
  doc["totalCells"] = sensorData.totalCells;
  
  String output;
  serializeJson(doc, output);
  
  String topic = String(topic_prefix) + "/progress";
  client.publish(topic.c_str(), output.c_str());
}

void publishEnvironmentData() {
  StaticJsonDocument<256> doc;
  doc["windSpeed"] = sensorData.windSpeed;
  doc["waterCurrent"] = sensorData.waterCurrent;
  doc["waveHeight"] = sensorData.waveHeight;
  doc["trashDensity"] = sensorData.trashDensity;
  
  String output;
  serializeJson(doc, output);
  
  String topic = String(topic_prefix) + "/environment";
  client.publish(topic.c_str(), output.c_str());
}

void publishCoverageData() {
  StaticJsonDocument<512> doc;
  doc["percent"] = sensorData.coveragePercent;
  doc["cellsCovered"] = sensorData.cellsCovered;
  doc["totalCells"] = sensorData.totalCells;
  doc["currentCell"] = pathPlan.currentPathIndex;
  
  // Simulate coverage cells
  JsonArray cells = doc.createNestedArray("cells");
  int maxCells = (sensorData.cellsCovered < 10) ? sensorData.cellsCovered : 10;
  for (int i = 0; i < maxCells; i++) {
    if (i < pathPlan.pathPoints) {
      JsonObject cell = cells.createNestedObject();
      cell["lat"] = pathPlan.path[i][0];
      cell["lng"] = pathPlan.path[i][1];
    }
  }
  
  String output;
  serializeJson(doc, output);
  
  String topic = String(topic_prefix) + "/coverage";
  client.publish(topic.c_str(), output.c_str());
}

void publishAlert(String message, String type) {
  StaticJsonDocument<256> doc;
  doc["message"] = message;
  doc["type"] = type;
  doc["timestamp"] = millis();
  
  String output;
  serializeJson(doc, output);
  
  String topic = String(topic_prefix) + "/alerts";
  client.publish(topic.c_str(), output.c_str());
}

void initializeSensorData() {
  sensorData.batteryLevel = 100.0;
  sensorData.solarCharging = false;
  sensorData.voltage = 12.6;
  sensorData.temperature = 28.0;
  sensorData.chargeCurrent = 0.0;
  
  sensorData.latitude = 13.0827;
  sensorData.longitude = 80.2707;
  sensorData.altitude = 0.0;
  sensorData.heading = 0.0;
  
  sensorData.mode = "IDLE";
  sensorData.lidar = 0.0;
  sensorData.speed = 0.0;
  sensorData.motorTemp = 25.0;
  
  sensorData.waterTemp = 26.0;
  sensorData.humidity = 65.0;
  sensorData.pressure = 1013.25;
  sensorData.rpmLeft = 0;
  sensorData.rpmRight = 0;
  sensorData.waterQuality = "Good";
  
  sensorData.areaCovered = 0.0;
  sensorData.trashCollected = 0.0;
  sensorData.cellsCovered = 0;
  sensorData.totalCells = 0;
  sensorData.coveragePercent = 0.0;
  
  sensorData.windSpeed = 0.0;
  sensorData.waterCurrent = 0.0;
  sensorData.waveHeight = 0.0;
  sensorData.trashDensity = "Low";
}

void initializePathPlanning() {
  pathPlan.areaPoints = 0;
  pathPlan.areaSet = false;
  pathPlan.cellSize = 5.0;
  pathPlan.pathPoints = 0;
  pathPlan.currentPathIndex = 0;
  pathPlan.missionActive = false;
  pathPlan.missionStartTime = 0;
}