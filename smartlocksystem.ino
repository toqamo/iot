#include <Arduino.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define ROWS 4
#define COLS 4
#define ir 11
#define servo 10
#define ledtrue 12
#define ledfalse 13

char keyMap[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

byte rowPins[ROWS] = { 9, 8, 7, 6 };
byte colPins[COLS] = { 5, 4, 3, 2 };

Keypad keypad = Keypad(makeKeymap(keyMap), rowPins, colPins, ROWS, COLS);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Check the I2C address of your LCD
Servo myservo;

String password = "";

void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(" welcome! ");
  pinMode(ir, INPUT);
  pinMode(ledtrue, OUTPUT);
  pinMode(ledfalse, OUTPUT);
  myservo.attach(servo);
  myservo.write(0);  // Initial position for the servo
}

void loop() {
  // Check if the IR sensor detects motion
  if (digitalRead(ir) == LOW) {  // Adjust to HIGH or LOW based on your IR sensor behavior
    Serial.println("Motion detected!");  
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Enter your pass:");
    lcd.setCursor(0, 1);

    // Wait for the password input
    unsigned long startTime = millis();     // Start timing
    while (millis() - startTime < 20000) {  // 20-second timeout
      char key = keypad.getKey();

      if (key) {
        // Append key to the password
        if (key != '*' && key != '#') {
          password += key;
          Serial.println(key);  // Debug: print entered key
          lcd.print("*");  // Display * for each character
        }

        // Handle '*' for delete
        if (key == '*') {
          if (password.length() > 0) {
            password.remove(password.length