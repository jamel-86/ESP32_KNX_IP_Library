#include <NewPing.h>
#include <esp-knx-ip.h>

const char *ssid = "my_ssid";   //  your network SSID (name)
const char *pass = "my_pwd";  // your network password

// Pin definitions
#define LED_PIN_1 22
#define LED_PIN_2 23
#define TRIGGER_PIN 19
#define ECHO_PIN 18
#define MAX_DISTANCE 500

// Update interval
#define UPDATE_INTERVAL 300

unsigned long next_change = 0;
float last_distance = 0.0;

// Configuration IDs
config_id_t distance_ga, min_threshold_ga, max_threshold_ga;
config_id_t hostname_id;
config_id_t update_rate_id;
config_id_t min_threshold_id, max_threshold_id;
config_id_t variation_threshold_id;
config_id_t param_id;
config_id_t my_GA;
int8_t some_var = 0;
bool last_min_threshold_state = false;
bool last_max_threshold_state = false;
float last_sent_distance = 0.0;

// Initialize the ultrasonic sensor
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  Serial.begin(115200);

  // Register configuration options

  hostname_id = knx.config_register_string("Hostname", 20, String("ultrasonic"));
  update_rate_id = knx.config_register_int("Update rate (ms)", UPDATE_INTERVAL);
  distance_ga = knx.config_register_ga("Distance (mm)");
  min_threshold_ga = knx.config_register_ga("Minimum Threshold GA");
  max_threshold_ga = knx.config_register_ga("Maximum Threshold GA");
  min_threshold_id = knx.config_register_int("Min Threshold (mm)", 500);
  max_threshold_id = knx.config_register_int("Max Threshold (mm)", 1000);
  variation_threshold_id = knx.config_register_int("Send on Variation (mm)", 10);


  // Load previous configuration from EEPROM
  knx.load();

  // Initialize WiFi
  WiFi.hostname(knx.config_get_string(hostname_id));
  WiFi.begin(ssid, pass);

  Serial.println("");
  Serial.print("[Connecting]");
  Serial.print(ssid);

  digitalWrite(LED_PIN_1, LOW);
  digitalWrite(LED_PIN_2, LOW);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN_1, HIGH);
    delay(250);
    Serial.print(".");
    digitalWrite(LED_PIN_1, LOW);
    delay(250);
  }
  digitalWrite(LED_PIN_1, HIGH);

  // Start KNX
  knx.start();

  Serial.println();
  Serial.println("Connected to wifi");
  Serial.println(WiFi.localIP());
}

void loop() {
  knx.loop();

  unsigned long now = millis();

  if (next_change < now) {
    next_change = now + knx.config_get_int(update_rate_id);

    // Get the distance from the ultrasonic sensor
    last_distance = (0.034027 / 2) * sonar.ping_median(10);  // Convert time to distance in cm
    // Debugging
    // Serial.print("Distance: ");
    // Serial.print(last_distance);
    // Serial.println("cm");

    // Send the distance to the KNX bus (scaled to mm)
    float variation = abs(last_distance - last_sent_distance);             // Calculate the difference
    if (variation >= knx.config_get_int(variation_threshold_id) / 10.0) {  // Convert threshold to cm for comparison
      uint16_t distance_mm = (uint16_t)(last_distance * 10); 
      knx.write_2byte_uint(knx.config_get_ga(distance_ga), distance_mm);
      last_sent_distance = last_distance;  // Update the last sent distance
    }

    // Check against thresholds and update LEDs and KNX accordingly
    bool current_min_threshold_state = (last_distance < knx.config_get_int(min_threshold_id) / 10.0);
    if (current_min_threshold_state != last_min_threshold_state) {
      knx.write_1bit(knx.config_get_ga(min_threshold_ga), current_min_threshold_state);
      digitalWrite(LED_PIN_1, current_min_threshold_state ? HIGH : LOW);
      last_min_threshold_state = current_min_threshold_state;
    }


    bool current_max_threshold_state = (last_distance > knx.config_get_int(max_threshold_id) / 10.0);
    if (current_max_threshold_state != last_max_threshold_state) {
      knx.write_1bit(knx.config_get_ga(max_threshold_ga), current_max_threshold_state);
      digitalWrite(LED_PIN_2, current_max_threshold_state ? HIGH : LOW);
      last_max_threshold_state = current_max_threshold_state;
    }
  }

  delay(50);
}

// Raw Data
void my_callback(message_t const &msg, void *arg) {
  switch (msg.ct) {
    case KNX_CT_WRITE:
      // Save received data
      some_var = knx.data_to_1byte_int(msg.data);
      break;
    case KNX_CT_READ:
      // Answer with saved data
      knx.answer_1byte_int(msg.received_on, some_var);
      break;
  }
}

void my_other_callback(message_t const &msg, void *arg) {
  switch (msg.ct) {
    case KNX_CT_WRITE:
      // Write an answer somewhere else
      int value = knx.config_get_int(param_id);
      address_t ga = knx.config_get_ga(my_GA);
      knx.answer_1byte_int(ga, (int8_t)value);
      break;
  }
}
