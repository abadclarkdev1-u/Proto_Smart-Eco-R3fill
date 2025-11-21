#include <LiquidCrystal_I2C.h>
#include <HX711.h>
#include <Servo.h>

// --- LCD (I2C 16x2) ---
LiquidCrystal_I2C lcd(0x27,16,2);

// --- HX711 Pins ---
#define LOADCELL_DOUT_PIN A0
#define LOADCELL_SCK_PIN  A1

// --- Ultrasonic Pins ---
#define TRIG_PIN 10
#define ECHO_PIN 11

// --- Other Pins ---
#define BUZZER_PIN 8
#define SERVO_PIN  9     // Accept gate servo

// --- NEW PINS ---
#define INDUCTIVE_PIN 6   // Metal detection
#define REJECT_SERVO_PIN 7 // Servo for pushing rejects

HX711 scale;
Servo gateServo;
Servo rejectServo;

// --- HX711 Scaling ---
float kg1_raw = 280.0;
float grams_per_unit = 2.383;

// --- Detection Settings ---
float weightThreshold = 50.0;
int distanceThreshold = 12;
unsigned long detectionWindow = 1500;
unsigned long cooldownTime = 1500;

unsigned long cooldownStart = 0;
bool inCooldown = false;
bool itemProcessing = false;

float lastWeight = 0.0;
unsigned long itemDetectTime = 0;

void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  showIdleScreen();

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.tare();
  delay(500);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  gateServo.attach(SERVO_PIN);
  gateServo.write(0);

  rejectServo.attach(REJECT_SERVO_PIN);
  rejectServo.write(0);  // idle position

  pinMode(BUZZER_PIN, OUTPUT);

  // NEW: inductive sensor input
  pinMode(INDUCTIVE_PIN, INPUT);
}

void loop() {
  unsigned long now = millis();

  // --- Cooldown ---
  if (inCooldown && (now - cooldownStart >= cooldownTime)) {
    inCooldown = false;
    showIdleScreen();
  }

  // --- Read weight ---
  float weight = 0.0;
  if(scale.is_ready()){
    long raw = scale.read();
    weight = raw * grams_per_unit;
    if(weight < 0) weight = 0;
  }

  if (!itemProcessing && !inCooldown) {
    lcd.setCursor(0,1);
    lcd.print("Weight: ");
    lcd.print(weight,1);
    lcd.print(" g   ");
  }

  // --- Ultrasonic ---
  int distance = getUltrasonicDistance();

  // --- Read inductive sensor ---
  int isMetal = digitalRead(INDUCTIVE_PIN); 
  // HIGH = metal detected (NPN sensor)

  // --- Detect object ---
  if (!itemProcessing && !inCooldown) {
    if (distance < distanceThreshold && (weight - lastWeight > weightThreshold)) {
      itemProcessing = true;
      itemDetectTime = now;

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Item detected!");
      lcd.setCursor(0,1);
      lcd.print("Checking...");
    }
  }

  // --- Classification ---
  if (itemProcessing) {
    if (distance < distanceThreshold) {
      
      // NEW: check inductive sensor
      if (isMetal == HIGH) {
        itemProcessing = false;
        handleAccept(weight);
      } else {
        itemProcessing = false;
        handleReject(); // wrong item
      }

    } else if (now - itemDetectTime > detectionWindow) {
      itemProcessing = false;
      handleReject();
    }
  }

  lastWeight = weight;
  delay(100);
}

int getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  int distance = duration * 0.034 / 2;

  if (distance == 0 || distance > 400) return 999;
  return distance;
}

// --- ACCEPT ---
void handleAccept(float measuredWeight) {
  beep(100);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("ACCEPT");
  lcd.setCursor(0,1);
  lcd.print("Loading...");

  delay(3000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Weight: ");
  lcd.print(measuredWeight,1);
  lcd.print(" g");

  lcd.setCursor(0,1);
  lcd.print("Volume: ");
  lcd.print(measuredWeight,1);
  lcd.print(" ml");

  // Open accept gate
  gateServo.write(90);
  delay(700);
  gateServo.write(0);

  cooldownStart = millis();
  inCooldown = true;
}

// --- REJECT ---
void handleReject() {
  beep(200);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Rejected!");
  lcd.setCursor(0,1);
  lcd.print("Wrong item!");

  // NEW: reject servo push
  rejectServo.write(90);
  delay(700);
  rejectServo.write(0);

  cooldownStart = millis();
  inCooldown = true;
}

void beep(unsigned int ms) {
  digitalWrite(BUZZER_PIN,HIGH);
  delay(ms);
  digitalWrite(BUZZER_PIN,LOW);
}

void showIdleScreen() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Eco-R3fill");
  lcd.setCursor(0,1);
  lcd.print("Insert item");
}
