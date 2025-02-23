
#include "functions.h"


// === Functions prototypes ===
void processCommand(String);

// === Sensor and Multiplexer Initialization ===
// Example sensor initialization (choose your sensor class as needed)
// TLx493D_A1B6 Sensor(Wire, TLx493D_IIC_ADDR_A0_e);
Tlv493d Sensor = Tlv493d();
TCA9548 mux_sensors(0x70);


/*
 * SETUP
 */
void setup() {
  
  initializeSerial();
  
  initializeSensor();

  initializeSolenoids();

  delay(500); // wait before starting main loop

}


/*
 * LOOP
 */
void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }
}

// process incoming JSON command
void processCommand(String raw_command) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, raw_command);

  if (error) {
    sendError(101, "Invalid JSON Format");
    return;
  }

  const char* cmd_name = doc["command"];
  if (!cmd_name) {
    sendError(102, "Missing command field");
    return;
  }

  if (strcmp(cmd_name, "READ_SENSORS") == 0) {
    sendSensorValues();
  } else if (strcmp(cmd_name, "READ_CURRENTS") == 0) {
    sendCurrentValues();
  } else if (strcmp(cmd_name, "SET_CURRENTS") == 0) {
    setSolenoidCurrents(doc["solenoids"].as<JsonArray>());
  } else if (strcmp(cmd_name, "RESET_CURRENTS") == 0) {
    resetSolenoids();
  } else if (strcmp(cmd_name, "GET_STATUS") == 0) {
    sendStatus();
  } else {
    sendError(103, "Unknown command");
  }
}




