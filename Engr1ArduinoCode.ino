#include <Stepper.h>
#include <DHT.h>

// ===== Motor config =====
const long STEPS_PER_REV = 2048;   // 28BYJ-48
const long OPEN_STEPS     = 900;   // adjust for your lid angle (+/- for direction)
const int  RPM            = 12;

const int IN1 = 8;
const int IN2 = 9;
const int IN3 = 10;
const int IN4 = 11;

enum LidState { CLOSED, OPEN };
LidState lidState = CLOSED;

Stepper motor(STEPS_PER_REV, IN1, IN3, IN2, IN4);

// ===== Water sensor (analog on A5) =====
const int  WATER_PIN       = A5;     // sensor SIG -> A5
int        WET_THRESHOLD   = 600;    // tune after reading serial output
int        DRY_THRESHOLD   = 450;    // must be < WET_THRESHOLD
const int  WATER_SAMPLES   = 10;     // averaging
const unsigned long WATER_INTERVAL_MS = 200;
bool waterWet = false;

// ===== Humidity sensor (DHT11 on D7) =====
#define DHTPIN   7
#define DHTTYPE  DHT11
DHT dht(DHTPIN, DHTTYPE);

const float HUMID_HIGH = 85.0;       // >= this => considered humid
const float HUMID_LOW  = 70.0;       // <= this => considered not humid (hysteresis)
const unsigned long HUM_INTERVAL_MS = 2000;
bool humidHigh = false;

// ===== Behavior tuning =====
const unsigned long MIN_DWELL_MS = 3000; // min time between moves
unsigned long lastMoveMs   = 0;
unsigned long lastWaterMs  = 0;
unsigned long lastHumMs    = 0;

// ---------- Helpers ----------
void releaseCoils() {
  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);
}

void engageDriver() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
}

void moveSteps(long steps) {
  engageDriver();
  motor.step(steps); // holds position energized
}

void openLid()  { moveSteps( OPEN_STEPS); lidState = OPEN;   }
void closeLid() { moveSteps(-OPEN_STEPS); lidState = CLOSED; }
void toggleLid(){ (lidState == CLOSED) ? openLid() : closeLid(); }

int readWaterAveraged() {
  long sum = 0;
  for (int i = 0; i < WATER_SAMPLES; ++i) {
    sum += analogRead(WATER_PIN);   // 0..1023
    delayMicroseconds(800);
  }
  return (int)(sum / WATER_SAMPLES);
}

// Serial line reader (expects Newline)
bool readLine(String &out) {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { out = buf; buf = ""; return true; }
    buf += c;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  motor.setSpeed(RPM);
  releaseCoils();         // start safe
  dht.begin();
  pinMode(WATER_PIN, INPUT);

  Serial.println("Lid control: OPEN on water (A5), CLOSE on humidity (D7/DHT11).");
  Serial.println("Priority: Water > Humidity. Commands: true | off");
}

void loop() {
  unsigned long now = millis();

  // ---- Manual serial control ----
  String line;
  if (readLine(line)) {
    line.trim(); line.toLowerCase();
    if (line == "true") {
      Serial.println("Manual: toggling lid...");
      toggleLid();
      lastMoveMs = now;
      Serial.println(lidState == OPEN ? "OPEN (holding)" : "CLOSED (holding)");
    } else if (line == "off") {
      Serial.println("Coils released.");
      releaseCoils();
    } else if (line.length()) {
      Serial.println("Commands: true | off");
    }
  }

  // ---- Water sensor ----
  if (now - lastWaterMs >= WATER_INTERVAL_MS) {
    lastWaterMs = now;
    int waterVal = readWaterAveraged();

    // Hysteresis
    if (!waterWet && waterVal >= WET_THRESHOLD) {
      waterWet = true;
      Serial.print("A5="); Serial.print(waterVal); Serial.println(" → WET");
    } else if (waterWet && waterVal <= DRY_THRESHOLD) {
      waterWet = false;
      Serial.print("A5="); Serial.print(waterVal); Serial.println(" → DRY");
    }
  }

  // ---- Humidity sensor ----
  if (now - lastHumMs >= HUM_INTERVAL_MS) {
    lastHumMs = now;
    float h = dht.readHumidity();
    if (!isnan(h)) {
      // Hysteresis
      if (!humidHigh && h >= HUMID_HIGH) {
        humidHigh = true;
        Serial.print("Humidity="); Serial.print(h); Serial.println("% → HUMID");
      } else if (humidHigh && h <= HUMID_LOW) {
        humidHigh = false;
        Serial.print("Humidity="); Serial.print(h); Serial.println("% → OK");
      } else {
        Serial.print("Humidity="); Serial.print(h); Serial.println("%");
      }
    } else {
      Serial.println("Humidity read error.");
    }
  }

  // ---- Decision logic (water has priority) ----
  if (now - lastMoveMs >= MIN_DWELL_MS) {
    if (waterWet) {
      if (lidState == CLOSED) {
        Serial.println("Auto: Water present → OPENING...");
        openLid();
        lastMoveMs = now;
        Serial.println("OPEN (holding).");
      }
    } else { // only consider humidity if no water
      if (humidHigh && lidState == OPEN) {
        Serial.println("Auto: Humid and no water → CLOSING...");
        closeLid();
        lastMoveMs = now;
        Serial.println("CLOSED.");
      }
    }
  }
}
