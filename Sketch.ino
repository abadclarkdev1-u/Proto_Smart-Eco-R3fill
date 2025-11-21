#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "HX711.h"
#include <Servo.h>

// --- I2C LCD (16x2) ---
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- HX711 Pins ---
#define LOADCELL_DOUT_PIN A0
#define LOADCELL_SCK_PIN A1

// --- Other Pins ---
const int sensorPin = 7;      // Inductive NPN sensor (LOW = metal)
const int buzzerPin = 8;      // Active buzzer
const int servoPin  = 9;      // Servo for reject

HX711 scale;
Servo gateServo;

//  HC-SR04
#define TRIG_PIN 7
#define ECHO_PIN 6

// --- Calibration ---
float calibration_factor = -7050.0;  // Replace with your actual calibration factor

// --- Detection Settings ---
const float weightThreshold = 20.0;  // Sudden increase = item inserted
const unsigned long detectionWindow = 1500;
const unsigned long cooldownTime = 1500;

unsigned long cooldownStart = 0;
bool inCooldown = false;
bool itemProcessing = false;

float lastWeight = 0.0;
unsigned long itemDetectTime = 0;

#define BIN_HEIGHT 40 // cm
#define THRESHOLD 3
int points = 0;
long prevDistance = 0;

void setup() {
  Serial.begin(9600);
  // startCountdown();
  // LCD Setup
  lcd.init();
  lcd.backlight();
  showIdleScreen();

  // HX711 Setup
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale();
  scale.tare();

  pinMode(sensorPin, INPUT_PULLUP); 
  pinMode(buzzerPin, OUTPUT);

  gateServo.attach(servoPin);
  gateServo.write(0);  // Neutral

// HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

unsigned long countdownStart = 0;
unsigned long countdownDuration = 13000; // 10 seconds
bool isCountingDown = false;
bool isTrigger = false;

void loop() {

  SensorStatus();
  if(isTrigger && !isCountingDown){
    startCountdown();
  }else{
    showIdleScreen();
  }
  countDown();

  delay(200);
}

// --- IDLE SCREEN ---
void showIdleScreen() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Eco-R3fill");
  lcd.setCursor(0,1);
  lcd.print("Pls, insert item");
}

long getDistance(){
  // Send a 10µs pulse to trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pulse duration
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate distance in cm
  long distance = duration * 0.034 / 2; // speed of sound ~ 0.034 cm/µs

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if(distance > BIN_HEIGHT) distance = BIN_HEIGHT;
  return distance;
  delay(1000);
}

// Dto mo malalaman kung may naglalagay ng item sa loob
void SensorStatus(){
  long currentDistance = getDistance();
  Serial.println(currentDistance);
  if(prevDistance - currentDistance > THRESHOLD){
    isTrigger = true;
  }

  prevDistance = currentDistance;
  delay(500);
}

void ProcessInput(){
  long currentDistance = getDistance();
  Serial.println(currentDistance);
  if(prevDistance - currentDistance > THRESHOLD){
    points++;
    Serial.print("New Trash Addded");
    Serial.print(points);

  }

  prevDistance = currentDistance;
}

void countDown(){
  if (isCountingDown) {
    unsigned long elapsed = millis() - countdownStart;

    int remaining = (countdownDuration - elapsed) / 1000;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(String("Points: ")+ points);
    lcd.setCursor(0,1);
    lcd.print(String("Remaining: ") + remaining);
    Serial.print("Countdown: === ");
    Serial.println(remaining);

    // Stop countdown and stop function
    if (elapsed >= countdownDuration) {
      Serial.println("Countdown finished! Stopping function.");
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Your Points: ");
      lcd.print(points);
      isCountingDown = false;
      isTrigger = false;
      delay(3000);
      PointsReward();
    }

    // While countdown active, run your function
    if (isCountingDown) {
      ProcessInput();   // Main Process (Dto na nag lalagay ng item)
    }
  }
}

void PointsReward(){
  // Dto mo lalagay kung kelan yung servo mag bibigay ng tubig
  gateServo.write(90);
  delay(700);
  gateServo.write(0);
}

void startCountdown() {
  countdownStart = millis();
  isCountingDown = true;
}