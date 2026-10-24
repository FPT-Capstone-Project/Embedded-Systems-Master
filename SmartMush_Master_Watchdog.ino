
/*
  SmartMush ESP32-S3 Master
  Features:
  - WiFiManager Captive Portal (no AP password)
  - Auto reconnect saved Wi-Fi every 2 minutes
  - MQTT over TLS 8883 (TEST MODE - no CA verification)
  - MQTT reconnect every 5 seconds
  - RS485 Modbus communication with STM32 nodes
  - Publish sensor temperature/humidity to MQTT
  - Control 8 SSR via MQTT ON/OFF commands
  - Publish SSR states to MQTT
  - Turn all SSR OFF when Wi-Fi/MQTT disconnects
  - Independent 10-second maximum-ON watchdog for misting SSR1/SSR2

  Libraries:
  - WiFiManager by tzapu
  - PubSubClient by Nick O'Leary
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

// =====================================================
// HARDWARE CONFIG
// =====================================================

// RS485 UART
static const int PIN_TX = 40;
static const int PIN_RX = 42;
static const int PIN_DE = 41;

// 8 SSR outputs
static const uint8_t SSR_COUNT = 8;
static const uint8_t SSR_PINS[SSR_COUNT] = {
  4, 5, 6, 7, 15, 16, 17, 18
};

// RS485
static const uint32_t MODBUS_BAUD = 9600;
static const uint32_t MODBUS_TIMEOUT_MS = 1000;

// Timing
static const uint32_t NODE_POLL_MS = 5000;
static const uint32_t WIFI_TRY_MS = 30000;

// NEW: Retry saved Wi-Fi every 2 minutes
static const uint32_t WIFI_RETRY_MS = 330000;

static const uint32_t MQTT_RETRY_MS = 5000;
static const uint8_t SENSOR_COUNT = 3;

// =====================================================
// OBJECTS AND STATE
// =====================================================

HardwareSerial RS485(1);

WiFiManager wm;

WiFiClientSecure net;
PubSubClient mqtt(net);

String apSSID;
String topicCommand;
String topicStatus;
String topicSsrPrefix;

uint32_t stateStarted = 0;
uint32_t lastMqttAttempt = 0;
uint32_t lastPoll = 0;

// NEW: Time of the last saved Wi-Fi connection attempt
uint32_t lastWifiAttemptAt = 0;

bool portalActive = false;
bool ssrState[SSR_COUNT] = {false};

// SSR1 GPIO4 and SSR2 GPIO5: independent local maximum-ON watchdog.
// 10 seconds is an initial TEST LIMIT, not an independently verified safety rating.
static const uint32_t MISTING_MAX_ON_MS = 10000;
static uint32_t mistingOnAt[SSR_COUNT] = {0};
static bool watchdogPendingOff[SSR_COUNT] = {false};
static bool watchdogTripped[SSR_COUNT] = {false};
static portMUX_TYPE ssrMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t watchdogTaskHandle = nullptr;
static bool watchdogTaskReady = false;

void flushWatchdogEvents(); // Forward declaration for RS485 receiveFrame.

static bool isMistingChannel(uint8_t channel) {
  return channel == 1 || channel == 2;
}

static bool getSSRState(uint8_t index) {
  bool state;
  portENTER_CRITICAL(&ssrMux);
  state = ssrState[index];
  portEXIT_CRITICAL(&ssrMux);
  return state;
}


uint32_t ignoreCommandsUntil = 0;

enum NetState {
  WIFI_CONNECTING,
  WIFI_CONNECTED,
  CONFIG_PORTAL
};

NetState netState = WIFI_CONNECTING;

// =====================================================
// MODBUS CRC16
// =====================================================

uint16_t crc16(const uint8_t *p, size_t n) {
  uint16_t c = 0xFFFF;

  while (n--) {
    c ^= *p++;

    for (int i = 0; i < 8; ++i) {
      c = (c & 1)
        ? (c >> 1) ^ 0xA001
        : (c >> 1);
    }
  }

  return c;
}

// =====================================================
// RS485
// =====================================================

void rs485Tx(bool tx) {
  digitalWrite(PIN_DE, tx ? HIGH : LOW);
  delayMicroseconds(100);
}

void sendFrame(uint8_t *q, size_t len) {
  while (RS485.available()) {
    RS485.read();
  }

  rs485Tx(true);

  RS485.write(q, len);
  RS485.flush();

  delayMicroseconds(150);

  rs485Tx(false);
}

bool receiveFrame(
  uint8_t *r,
  size_t len,
  uint32_t timeoutMs
) {
  size_t received = 0;
  uint32_t started = millis();

  while (
    received < len &&
    millis() - started < timeoutMs
  ) {
    while (
      RS485.available() &&
      received < len
    ) {
      r[received++] = (uint8_t)RS485.read();
    }

    if (mqtt.connected()) {
      mqtt.loop();
    }

    flushWatchdogEvents();
    delay(1);
  }

  return received == len;
}

// =====================================================
// SSR CONTROL + INDEPENDENT LOCAL MAXIMUM-ON WATCHDOG
// =====================================================

// Publishes only from the Arduino loop/task owning the PubSubClient.
// The watchdog task never touches MQTT (PubSubClient is not thread safe).
void publishSSRState(uint8_t channel, bool turnOn) {
  if (!mqtt.connected()) return;
  String topic = topicSsrPrefix + String(channel) + "/state";
  bool sent = mqtt.publish(topic.c_str(), turnOn ? "ON" : "OFF", true);
  Serial.printf("[MQTT] SSR%u state=%s publish=%s\n",
                channel, turnOn ? "ON" : "OFF", sent ? "OK" : "FAILED");
}

// Repeat ON for an already-ON valve DOES NOT refresh the 10-second deadline.
void setSSR(uint8_t channel, bool turnOn, bool publishState = true) {
  if (channel < 1 || channel > SSR_COUNT) return;
  const uint8_t idx = channel - 1;
  bool duplicateOn = false;
  bool appliedState = false;
  bool denied = false;

  portENTER_CRITICAL(&ssrMux);
  if (turnOn && isMistingChannel(channel) && !watchdogTaskReady) {
    denied = true; // Never run a misting valve without the local watchdog task.
  } else if (turnOn && isMistingChannel(channel) && watchdogTripped[idx]) {
    denied = true; // After timeout, require explicit OFF before the next ON.
  } else if (turnOn && isMistingChannel(channel) && ssrState[idx]) {
    duplicateOn = true;
  } else {
    if (turnOn && isMistingChannel(channel)) mistingOnAt[idx] = millis();
    if (!turnOn) {
      mistingOnAt[idx] = 0;
      watchdogTripped[idx] = false;
    }
    // GPIO and software state transition happen under the same short lock.
    digitalWrite(SSR_PINS[idx], (turnOn != SSR_ACTIVE_LOW) ? HIGH : LOW);
    ssrState[idx] = turnOn;
    appliedState = true;
  }
  portEXIT_CRITICAL(&ssrMux);

  if (denied) {
    Serial.printf("[WATCHDOG] Refusing CH%u ON: watchdog unavailable or timeout latched\n", channel);
    return;
  }
  if (duplicateOn) {
    Serial.printf("[WATCHDOG] CH%u duplicate ON ignored (deadline unchanged)\n", channel);
    // Acknowledge existing state to broker without extending physical ON time.
    if (publishState) publishSSRState(channel, true);
    return;
  }
  if (appliedState) {
    Serial.printf("[SSR] CH%u GPIO%u = %s\n", channel, SSR_PINS[idx],
                  turnOn ? "ON" : "OFF");
    if (publishState) publishSSRState(channel, turnOn);
  }
}

// Watchdog checks independently of blocking MQTT TLS connections or RS485 polls.
// Never allocate, log or call MQTT in this task.
void mistingWatchdogTask(void *parameter) {
  (void)parameter;
  for (;;) {
    const uint32_t now = millis();
    for (uint8_t idx = 0; idx < 2; ++idx) {
      portENTER_CRITICAL(&ssrMux);
      if (ssrState[idx] &&
          (uint32_t)(now - mistingOnAt[idx]) >= MISTING_MAX_ON_MS) {
        // Physical output is switched OFF here, not deferred to the main loop.
        digitalWrite(SSR_PINS[idx], SSR_ACTIVE_LOW ? HIGH : LOW);
        ssrState[idx] = false;
        mistingOnAt[idx] = 0;
        watchdogPendingOff[idx] = true;
        watchdogTripped[idx] = true;
      }
      portEXIT_CRITICAL(&ssrMux);
    }
    vTaskDelay(pdMS_TO_TICKS(10) > 0 ? pdMS_TO_TICKS(10) : 1);
  }
}

// Consume watchdog events in the main loop, including while waiting for Modbus.
void flushWatchdogEvents() {
  for (uint8_t idx = 0; idx < 2; ++idx) {
    bool timedOut;
    portENTER_CRITICAL(&ssrMux);
    timedOut = watchdogPendingOff[idx];
    watchdogPendingOff[idx] = false;
    portEXIT_CRITICAL(&ssrMux);
    if (timedOut) {
      Serial.printf("[WATCHDOG] CH%u GPIO%u MAX-ON timeout -> OFF\n",
                    idx + 1, SSR_PINS[idx]);
      // Publish the latest actual state; never re-advertise ON after timeout.
      publishSSRState(idx + 1, getSSRState(idx));
    }
  }
}

void allSSRoff(bool publishState = true) {
  for (uint8_t channel = 1; channel <= SSR_COUNT; ++channel) {
    setSSR(channel, false, publishState);
  }
}

void publishAllSSRStates() {
  if (!mqtt.connected()) return;
  for (uint8_t channel = 1; channel <= SSR_COUNT; ++channel) {
    publishSSRState(channel, getSSRState(channel - 1));
  }
}

// =====================================================
// SSR NETWORK SAFETY
// =====================================================

void safetyCheckConnection() {
  if (
    WiFi.status() == WL_CONNECTED &&
    mqtt.connected()
  ) {
    return;
  }

  bool anyOn = false;

  for (uint8_t i = 0; i < SSR_COUNT; ++i) {
    if (getSSRState(i)) {
      anyOn = true;
    }
  }

  if (anyOn) {
    Serial.println(
      "[SAFETY] Network/MQTT disconnected -> ALL SSR OFF"
    );

    allSSRoff(false);
  }
}

// =====================================================
// MQTT COMMAND CALLBACK
// =====================================================

// Command:
// smartmush/master01/ssr/1/set
// Payload: ON or OFF

void onMqttMessage(
  char *topic,
  byte *payload,
  unsigned int length
) {
  String t(topic);

  if (
    !t.startsWith(topicSsrPrefix) ||
    !t.endsWith("/set")
  ) {
    return;
  }

  String channelText = t.substring(
    topicSsrPrefix.length(),
    t.length() - 4
  );

  if (
    channelText.length() == 0 ||
    channelText.length() > 2
  ) {
    return;
  }

  for (
    size_t i = 0;
    i < channelText.length();
    ++i
  ) {
    if (!isDigit(channelText[i])) {
      return;
    }
  }

  const int channel = channelText.toInt();

  if (
    channel < 1 ||
    channel > SSR_COUNT
  ) {
    return;
  }

  if (length > 16) {
    return;
  }

  String cmd;

  for (
    unsigned int i = 0;
    i < length;
    ++i
  ) {
    cmd += char(payload[i]);
  }

  cmd.trim();
  cmd.toUpperCase();

  if (
    cmd != "ON" &&
    cmd != "OFF" &&
    cmd != "1" &&
    cmd != "0"
  ) {
    Serial.printf(
      "[MQTT] Invalid SSR command on CH%d\n",
      channel
    );
    return;
  }

  // Brief reconnect guard
  if (
    (int32_t)(
      millis() - ignoreCommandsUntil
    ) < 0
  ) {
    Serial.println(
      "[MQTT] Ignored command during reconnect guard"
    );
    return;
  }

  if (
    netState != WIFI_CONNECTED ||
    WiFi.status() != WL_CONNECTED ||
    !mqtt.connected()
  ) {
    return;
  }

  bool turnOn =
    (cmd == "ON" || cmd == "1");

  setSSR(
    (uint8_t)channel,
    turnOn
  );
}

// =====================================================
// WIFI MANAGER
// =====================================================

void stopPortal() {
  if (!portalActive) {
    return;
  }

  wm.stopConfigPortal();
  portalActive = false;

  Serial.println(
    "[WIFI] Captive Portal stopped"
  );
}

// Try stored Wi-Fi credentials
void beginWifiAttempt() {
  allSSRoff(false);

  stopPortal();

  mqtt.disconnect();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);

  // Disconnect without erasing saved credentials
  WiFi.disconnect(false, false);

  netState = WIFI_CONNECTING;

  stateStarted = millis();

  // NEW: Save retry timestamp
  lastWifiAttemptAt = stateStarted;

  Serial.println(
    "[WIFI] Trying saved Wi-Fi for 30 seconds..."
  );

  WiFi.begin();
}

// Start configuration portal
void startPortal() {
  if (portalActive) {
    return;
  }

  allSSRoff(false);

  mqtt.disconnect();

  WiFi.mode(WIFI_AP_STA);

  wm.setConfigPortalBlocking(false);

  wm.startConfigPortal(
    apSSID.c_str()
  );

  portalActive = true;
  netState = CONFIG_PORTAL;

  stateStarted = millis();

  Serial.printf(
    "[WIFI] Captive Portal: %s | http://192.168.4.1\n",
    apSSID.c_str()
  );
}

// =====================================================
// WIFI STATE MACHINE - RETRY EVERY 2 MINUTES
// =====================================================

void manageWifi() {
  const uint32_t now = millis();

  // ---------------------------------
  // STATE 1: CONNECTING TO SAVED WIFI
  // ---------------------------------

  if (netState == WIFI_CONNECTING) {

    if (WiFi.status() == WL_CONNECTED) {

      netState = WIFI_CONNECTED;

      Serial.printf(
        "[WIFI] Connected to %s | IP %s\n",
        WiFi.SSID().c_str(),
        WiFi.localIP().toString().c_str()
      );

      // Allow MQTT to reconnect immediately
      lastMqttAttempt =
        millis() - MQTT_RETRY_MS;

    } else if (
      now - stateStarted >= WIFI_TRY_MS
    ) {

      Serial.println(
        "[WIFI] Connection timeout -> Open Captive Portal"
      );

      startPortal();
    }
  }

  // ---------------------------------
  // STATE 2: WIFI CONNECTED
  // ---------------------------------

  else if (netState == WIFI_CONNECTED) {

    if (WiFi.status() != WL_CONNECTED) {

      Serial.println(
        "[WIFI] Disconnected. Retrying saved network..."
      );

      beginWifiAttempt();
    }
  }

  // ---------------------------------
  // STATE 3: CAPTIVE PORTAL
  // ---------------------------------

  else if (netState == CONFIG_PORTAL) {

    // Keep DNS/HTTP portal responsive
    wm.process();

    // User configured Wi-Fi successfully
    if (WiFi.status() == WL_CONNECTED) {

      stopPortal();

      netState = WIFI_CONNECTED;

      Serial.printf(
        "[WIFI] Configured and connected | IP %s\n",
        WiFi.localIP().toString().c_str()
      );

      // Automatically attempt MQTT connection
      lastMqttAttempt =
        millis() - MQTT_RETRY_MS;

    }

    // NEW: Automatically retry saved Wi-Fi
    // every 120 seconds from previous attempt
    else if (
      wm.getWiFiIsSaved() &&
      now - lastWifiAttemptAt >= WIFI_RETRY_MS
    ) {

      Serial.println(
        "[WIFI] 2-minute retry: reconnecting to saved Wi-Fi..."
      );

      beginWifiAttempt();
    }
  }
}

// =====================================================
// MQTT CONNECTION
// =====================================================

void connectMqttIfNeeded() {
  if (
    netState != WIFI_CONNECTED ||
    WiFi.status() != WL_CONNECTED
  ) {
    return;
  }

  if (mqtt.connected()) {
    mqtt.loop();
    return;
  }

  if (
    millis() - lastMqttAttempt <
    MQTT_RETRY_MS
  ) {
    return;
  }

  lastMqttAttempt = millis();

  String clientId =
    String("smartmush-") +
    MASTER_ID +
    "-" +
    String(
      (uint32_t)(
        ESP.getEfuseMac() & 0xFFFFFF
      ),
      HEX
    );

  bool ok = mqtt.connect(
    clientId.c_str(),
    MQTT_USER,
    MQTT_PASSWORD,
    topicStatus.c_str(),
    1,
    true,
    "offline"
  );

  if (ok) {

    ignoreCommandsUntil =
      millis() + 2000;

    mqtt.publish(
      topicStatus.c_str(),
      "online",
      true
    );

    if (
      !mqtt.subscribe(
        topicCommand.c_str(),
        1
      )
    ) {
      Serial.println(
        "[MQTT] Subscribe failed"
      );
    }

    publishAllSSRStates();

    Serial.printf(
      "[MQTT] Connected | command topic: %s\n",
      topicCommand.c_str()
    );

  } else {

    Serial.printf(
      "[MQTT] Connect error=%d\n",
      mqtt.state()
    );
  }
}

// =====================================================
// PUBLISH SENSOR DATA
// =====================================================

void publishSensor(
  uint8_t nodeId,
  uint8_t sensorId,
  bool ok,
  int16_t t,
  uint16_t h
) {
  String topic =
    String("smartmush/") +
    MASTER_ID +
    "/node/" +
    nodeId +
    "/sensor/" +
    sensorId;

  char payload[160];

  if (ok) {
    snprintf(
      payload,
      sizeof(payload),
      "{\"node_id\":%u,\"sensor_id\":%u,"
      "\"ok\":true,\"temperature\":%.1f,"
      "\"humidity\":%.1f}",
      nodeId,
      sensorId,
      t / 10.0,
      h / 10.0
    );
  } else {
    snprintf(
      payload,
      sizeof(payload),
      "{\"node_id\":%u,\"sensor_id\":%u,"
      "\"ok\":false,\"temperature\":null,"
      "\"humidity\":null}",
      nodeId,
      sensorId
    );
  }

  if (mqtt.connected()) {

    if (
      !mqtt.publish(
        topic.c_str(),
        payload,
        false
      )
    ) {
      Serial.println(
        "[MQTT] Sensor publish failed"
      );
    }
  }
}

// =====================================================
// NODE OFFLINE
// =====================================================

void publishNodeOffline(
  uint8_t nodeId
) {
  Serial.printf(
    "Node %u: no valid reply\n",
    nodeId
  );

  for (
    uint8_t sensorId = 1;
    sensorId <= SENSOR_COUNT;
    ++sensorId
  ) {
    publishSensor(
      nodeId,
      sensorId,
      false,
      0,
      0
    );
  }
}

// =====================================================
// READ STM32 NODE VIA MODBUS
// =====================================================

void pollNode(uint8_t nodeId) {

  uint8_t q[8] = {
    nodeId,
    0x03,
    0,
    0,
    0,
    9,
    0,
    0
  };

  uint16_t c = crc16(q, 6);

  q[6] = c & 0xFF;
  q[7] = c >> 8;

  sendFrame(q, 8);

  uint8_t r[23] = {};

  if (
    !receiveFrame(
      r,
      sizeof(r),
      MODBUS_TIMEOUT_MS
    )
  ) {
    publishNodeOffline(nodeId);
    return;
  }

  if (
    r[0] != nodeId ||
    r[1] != 3 ||
    r[2] != 18 ||
    crc16(r, 21) !=
      (uint16_t)(
        r[21] | (r[22] << 8)
      )
  ) {

    Serial.printf(
      "Node %u: invalid reply/CRC\n",
      nodeId
    );

    publishNodeOffline(nodeId);
    return;
  }

  for (
    uint8_t i = 0;
    i < SENSOR_COUNT;
    ++i
  ) {

    size_t k = 3 + i * 6;

    int16_t t =
      (int16_t)(
        ((uint16_t)r[k] << 8) |
        r[k + 1]
      );

    uint16_t h =
      ((uint16_t)r[k + 2] << 8) |
      r[k + 3];

    bool ok =
      (
        ((uint16_t)r[k + 4] << 8) |
        r[k + 5]
      ) == 1;

    if (ok) {

      Serial.printf(
        "Node %u | Sensor %u | "
        "T=%.1f C | H=%.1f %% | OK\n",
        nodeId,
        i + 1,
        t / 10.0,
        h / 10.0
      );

    } else {

      Serial.printf(
        "Node %u | Sensor %u | "
        "no response/error\n",
        nodeId,
        i + 1
      );
    }

    publishSensor(
      nodeId,
      i + 1,
      ok,
      t,
      h
    );

    if (mqtt.connected()) {
      mqtt.loop();
    }
  }
}

// =====================================================
// POLL ALL STM32 NODES
// =====================================================

void pollAll() {

  for (
    uint16_t id = FIRST_NODE_ID;
    id <= LAST_NODE_ID;
    ++id
  ) {

    flushWatchdogEvents();
    pollNode((uint8_t)id);
    flushWatchdogEvents();

    manageWifi();
    safetyCheckConnection();
    connectMqttIfNeeded();

    delay(50);
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(100);

  // SSR OFF at startup
  for (
    uint8_t i = 0;
    i < SSR_COUNT;
    ++i
  ) {

    digitalWrite(
      SSR_PINS[i],
      SSR_ACTIVE_LOW ? HIGH : LOW
    );

    pinMode(
      SSR_PINS[i],
      OUTPUT
    );
  }

  // Run a separate watchdog task; if it cannot start, misting ON is denied.
  BaseType_t created = xTaskCreate(
    mistingWatchdogTask, "misting_wdt", 4096, nullptr, 3,
    &watchdogTaskHandle
  );
  portENTER_CRITICAL(&ssrMux);
  watchdogTaskReady = (created == pdPASS);
  portEXIT_CRITICAL(&ssrMux);
  Serial.printf("[WATCHDOG] Misting SSR1/SSR2 max ON=%lu ms | task=%s\n",
                (unsigned long)MISTING_MAX_ON_MS,
                watchdogTaskReady ? "READY" : "FAILED - ON DISABLED");

  pinMode(PIN_DE, OUTPUT);
  digitalWrite(PIN_DE, LOW);

  RS485.begin(
    MODBUS_BAUD,
    SERIAL_8N1,
    PIN_RX,
    PIN_TX
  );

  apSSID =
    String("SmartMush-Setup-") +
    String(
      (uint32_t)(
        ESP.getEfuseMac() & 0xFFFFFF
      ),
      HEX
    );

  topicStatus =
    String("smartmush/") +
    MASTER_ID +
    "/status";

  topicSsrPrefix =
    String("smartmush/") +
    MASTER_ID +
    "/ssr/";

  topicCommand =
    topicSsrPrefix +
    "+/set";

  // MQTT TLS TEST MODE
  if (MQTT_SKIP_CERT_VERIFY_FOR_TEST) {

    net.setInsecure();

    Serial.println(
      "[MQTT] WARNING: TLS server certificate verification DISABLED (test only)"
    );

  } else {

    if (strlen(MQTT_CA_CERT) < 50) {

      Serial.println(
        "[MQTT] ERROR: Missing CA certificate"
      );

    } else {

      net.setCACert(
        MQTT_CA_CERT
      );
    }

    configTime(
      7 * 3600,
      0,
      "pool.ntp.org",
      "time.google.com"
    );
  }

  mqtt.setServer(
    MQTT_HOST,
    MQTT_PORT
  );

  mqtt.setCallback(
    onMqttMessage
  );

  mqtt.setKeepAlive(15);
  mqtt.setSocketTimeout(2);

  wm.setConfigPortalBlocking(false);
  wm.setConnectTimeout(20);
  wm.setSaveConnect(true);

  WiFi.mode(WIFI_STA);

  if (wm.getWiFiIsSaved()) {

    beginWifiAttempt();

  } else {

    startPortal();
  }

  Serial.println(
    "SmartMush Master ready | HELP | READNODE | CONFIG | WIFI | MQTT | SSR"
  );
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  flushWatchdogEvents();
  manageWifi();

  safetyCheckConnection();

  connectMqttIfNeeded();
  flushWatchdogEvents();

  // Serial commands
  if (Serial.available()) {

    String cmd =
      Serial.readStringUntil('\n');

    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "HELP") {

      Serial.println(
        "Commands: READNODE, CONFIG, WIFI, MQTT, SSR, HELP"
      );

    } else if (cmd == "CONFIG") {

      startPortal();

    } else if (cmd == "READNODE") {

      pollAll();

    } else if (cmd == "WIFI") {

      Serial.printf(
        "WiFi status=%d, SSID=%s, RSSI=%d\n",
        WiFi.status(),
        WiFi.SSID().c_str(),
        WiFi.RSSI()
      );

    } else if (cmd == "MQTT") {

      Serial.printf(
        "MQTT connected=%s, state=%d\n",
        mqtt.connected() ? "yes" : "no",
        mqtt.state()
      );

    } else if (cmd == "SSR") {

      for (
        uint8_t i = 0;
        i < SSR_COUNT;
        ++i
      ) {

        Serial.printf(
          "CH%u GPIO%u %s\n",
          i + 1,
          SSR_PINS[i],
          getSSRState(i) ? "ON" : "OFF"
        );
      }
    }
  }

  // Poll STM32 nodes every 5 seconds
  if (
    millis() - lastPoll >=
    NODE_POLL_MS
  ) {

    lastPoll = millis();

    pollAll();
  }

  if (mqtt.connected()) {
    mqtt.loop();
  }

  safetyCheckConnection();
  flushWatchdogEvents();

  delay(2);
}
