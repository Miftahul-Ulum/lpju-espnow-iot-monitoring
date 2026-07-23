#include <ESP8266WiFi.h>
#include <espnow.h>

// =====================================
// SLAVE IDENTITY
// =====================================

#define SLAVE_ID   2 // GANTI MENJADI SLAVE ID SELANJUT MISAL 2, JANGAN SAMPAI DUPLIKAT
#define SLAVE_NAME "SLAVE 2" // GANTI MENJADI SLAVE NAME, MISAL 'SLAVE 2'

// =====================================
// PIN CONFIG
// =====================================

#define LDR_PIN    3
#define RELAY_PIN  0
#define LED_PIN    2

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
// GLOBAL
// =====================================

struct_command incomingData;

struct_ack ackData;

// =====================================
// CALLBACK RECEIVE
// =====================================

void onDataRecv(uint8_t *mac,
                uint8_t *incoming,
                uint8_t len) {

  memcpy(&incomingData,
         incoming,
         sizeof(incomingData));

  Serial.println();
  Serial.println("======= COMMAND RECEIVED =======");

  Serial.print("COMMAND : ");

  Serial.println(incomingData.command ?
                 "ON" : "OFF");

  // =====================================
  // RELAY CONTROL
  // =====================================

  if (incomingData.command) {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
  }

  else {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);
  }
  
  delay(1000);

  // =====================================
  // READ LDR
  // =====================================

  uint8_t ldrState = digitalRead(LDR_PIN);

  // =====================================
  // PREPARE ACK
  // =====================================

  ackData.slaveId = SLAVE_ID;

  strcpy(ackData.slaveName,
         SLAVE_NAME);

  ackData.relayState =
    digitalRead(RELAY_PIN);

  ackData.ledState =
    !digitalRead(LED_PIN);

  ackData.ldrState =
    ldrState;

  // =====================================
  // SEND ACK
  // =====================================

  esp_now_send(mac,
               (uint8_t *) &ackData,
               sizeof(ackData));

  // =====================================
  // SERIAL MONITOR
  // =====================================

  Serial.println();
  Serial.println("========== ACK SENT ==========");

  Serial.print("SLAVE       : ");
  Serial.println(SLAVE_NAME);

  Serial.print("SLAVE ID    : ");
  Serial.println(SLAVE_ID);

  Serial.print("RELAY STATE : ");

  Serial.println(ackData.relayState ?
                 "ON" : "OFF");

  Serial.print("LED STATE   : ");

  Serial.println(ackData.ledState ?
                 "ON" : "OFF");

  Serial.print("LDR STATE   : ");

  Serial.println(ackData.ldrState);

  Serial.println("================================");
}

// =====================================
// CALLBACK SEND
// =====================================

void onDataSent(uint8_t *mac_addr,
                uint8_t sendStatus) {

  Serial.print("ACK SEND STATUS : ");

  Serial.println(sendStatus == 0 ?
                 "SUCCESS" : "FAIL");
}

// =====================================
// SETUP
// =====================================

void setup() {
  Serial.begin(115200,
               SERIAL_8N1,
               SERIAL_TX_ONLY);

  pinMode(LDR_PIN, INPUT);

  pinMode(RELAY_PIN, OUTPUT);

  pinMode(LED_PIN, OUTPUT);

  // =====================================
  // DEFAULT STATE
  // =====================================

  digitalWrite(RELAY_PIN, LOW);

  digitalWrite(LED_PIN, HIGH);

  WiFi.mode(WIFI_STA);

  Serial.println();
  Serial.println("ESP NOW SLAVE");

  Serial.print("DEVICE NAME : ");

  Serial.println(SLAVE_NAME);

  Serial.print("DEVICE ID   : ");

  Serial.println(SLAVE_ID);

  // =====================================
  // INIT ESPNOW
  // =====================================

  if (esp_now_init() != 0) {

    Serial.println("ESP NOW INIT FAILED");

    return;
  }

  esp_now_set_self_role(
    ESP_NOW_ROLE_SLAVE
  );

  esp_now_register_recv_cb(
    onDataRecv
  );

  esp_now_register_send_cb(
    onDataSent
  );

  Serial.println("ESP NOW READY");
}

// =====================================
// LOOP
// =====================================

void loop() {

}
