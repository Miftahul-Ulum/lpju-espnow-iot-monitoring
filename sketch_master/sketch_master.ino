#include <ESP8266WiFi.h>
#include <espnow.h>

#define LED_PIN 2

// =====================================
// STRUCT COMMAND
// =====================================

typedef struct struct_command {
  bool command;
} struct_command;

// =====================================
// STRUCT ACK
// =====================================

typedef struct struct_ack {

  uint8_t slaveId;

  char slaveName[20];

  uint8_t relayState;

  uint8_t ledState;

  uint8_t ldrState;

} struct_ack;

// =====================================
// SLAVE STRUCT
// =====================================

typedef struct {

  uint8_t mac[6];

  uint8_t id;

  const char* name;

} SlaveNode;

// =====================================
// REGISTER SLAVES
// =====================================

SlaveNode slaves[] = {

  {
    {0x98, 0xF4, 0xAB, 0xF5, 0x2A, 0x99},
    1,
    "SLAVE 1"
  },

  {
    {0xC8, 0x2B, 0x96, 0x1B, 0xAA, 0x07},
    2,
    "SLAVE 2"
  },

  {
    {0xE8, 0x68, 0xE7, 0xFB, 0xE3, 0x5D},
    3,
    "SLAVE 3"
  }

  // =====================================
  // TAMBAH SLAVE BARU DISINI
  // =====================================

  /*
  ,
  {
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF},
    3,
    "SLAVE 3"
  }
  */
};

const int TOTAL_SLAVES =
  sizeof(slaves) / sizeof(slaves[0]);

// =====================================
// GLOBAL
// =====================================

struct_command sendData;

bool currentState = false;

// =====================================
// FIND SLAVE
// =====================================

SlaveNode* findSlave(uint8_t *mac) {

  for (int i = 0; i < TOTAL_SLAVES; i++) {

    bool match = true;

    for (int j = 0; j < 6; j++) {

      if (mac[j] != slaves[i].mac[j]) {

        match = false;

        break;
      }
    }

    if (match) {
      return &slaves[i];
    }
  }

  return NULL;
}

// =====================================
// CALLBACK SEND
// =====================================

void onDataSent(uint8_t *mac_addr,
                uint8_t sendStatus) {

  SlaveNode* slave = findSlave(mac_addr);

  Serial.print("SEND TO ");

  if (slave != NULL) {

    Serial.print(slave->name);

  } else {

    Serial.print("UNKNOWN");
  }

  Serial.print(" -> ");

  Serial.println(sendStatus == 0 ?
                 "SUCCESS" : "FAIL");
}

// =====================================
// CALLBACK RECEIVE ACK
// =====================================

void onDataRecv(uint8_t *mac,
                uint8_t *incomingData,
                uint8_t len) {

  struct_ack ackData;

  memcpy(&ackData,
         incomingData,
         sizeof(ackData));

  // =====================================
  // DEBUG SERIAL
  // =====================================

  Serial.println();
  Serial.println("========== ACK RECEIVED ==========");

  Serial.print("SLAVE       : ");
  Serial.println(ackData.slaveName);

  Serial.print("SLAVE ID    : ");
  Serial.println(ackData.slaveId);

  Serial.print("RELAY STATE : ");

  Serial.println(ackData.relayState ?
                 "ON" : "OFF");

  Serial.print("LED STATE   : ");

  Serial.println(ackData.ledState ?
                 "ON" : "OFF");

  Serial.print("LDR STATE   : ");

  Serial.println(ackData.ldrState);

  // =====================================
  // VALIDASI LAMPU
  // =====================================

  Serial.print("LAMP STATUS : ");

  if (ackData.relayState == 1 &&
      ackData.ldrState == 0) {

    Serial.println("ON CONFIRMED");
  }

  else if (ackData.relayState == 0 &&
           ackData.ldrState == 1) {

    Serial.println("OFF CONFIRMED");
  }

  else {

    Serial.println("ERROR / MISMATCH");
  }

  Serial.println("==================================");

  // =====================================
  // FORWARD DATA TO ESP32 GATEWAY
  //
  // FORMAT:
  // DATA:SLAVE_ID,RELAY,LDR
  //
  // EXAMPLE:
  // DATA:1,ON,0
  // =====================================

  String uartData = "";

  uartData += "DATA:";

  uartData += String(ackData.slaveId);

  uartData += ",";

  uartData += String(
    ackData.relayState ? "ON" : "OFF"
  );

  uartData += ",";

  uartData += String(ackData.ldrState);

  // SEND TO ESP32
  Serial.println(uartData);
}

// =====================================
// NOTIFICATION
// =====================================

void blinkLED(int times) {

  for (int i = 0; i < times; i++) {

    // LED ON (active low)
    digitalWrite(LED_PIN, LOW);

    delay(100);

    // LED OFF
    digitalWrite(LED_PIN, HIGH);

    delay(100);
  }
}

// =====================================
// SETUP
// =====================================

void setup() {

  // =====================================
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  

  // =====================================
  // UART:
  // GPIO1 = TX
  // GPIO3 = RX
  // =====================================

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.println("ESP NOW MASTER");

  // =====================================
  // INIT ESPNOW
  // =====================================

  if (esp_now_init() != 0) {

    Serial.println("ESP NOW INIT FAILED");

    return;
  }

  esp_now_set_self_role(
    ESP_NOW_ROLE_CONTROLLER
  );

  esp_now_register_send_cb(
    onDataSent
  );

  esp_now_register_recv_cb(
    onDataRecv
  );

  // =====================================
  // REGISTER ALL SLAVES
  // =====================================

  for (int i = 0; i < TOTAL_SLAVES; i++) {

    esp_now_add_peer(
      slaves[i].mac,
      ESP_NOW_ROLE_SLAVE,
      1,
      NULL,
      0
    );

    Serial.print("REGISTERED : ");

    Serial.println(slaves[i].name);
  }

  Serial.println();
  Serial.println("MASTER READY");
}

// =====================================
// LOOP
// =====================================

void loop() {

  // =====================================
  // RECEIVE COMMAND FROM ESP32
  // =====================================

  if (Serial.available()) {

    String incoming =
      Serial.readStringUntil('\n');

    incoming.trim();
    
    blinkLED(1);

    // =====================================
    // COMMAND ON
    // =====================================

    if (incoming == "ON") {

      currentState = true;
    }

    // =====================================
    // COMMAND OFF
    // =====================================

    else if (incoming == "OFF") {

      currentState = false;
    }

    // =====================================
    // INVALID COMMAND
    // =====================================

    else {

      Serial.println("INVALID COMMAND");

      return;
    }

    // =====================================
    // PREPARE DATA
    // =====================================

    sendData.command = currentState;

    Serial.println();
    Serial.println("========== COMMAND RECEIVED ==========");

    Serial.print("UART COMMAND : ");

    Serial.println(currentState ?
                   "ON" : "OFF");

    // =====================================
    // SEND TO ALL SLAVES
    // =====================================

    for (int i = 0; i < TOTAL_SLAVES; i++) {

      Serial.print("TARGET       : ");

      Serial.println(slaves[i].name);

      esp_now_send(
        slaves[i].mac,
        (uint8_t *) &sendData,
        sizeof(sendData)
      );
    }

    blinkLED(2);

    Serial.println("======================================");
  }
}
