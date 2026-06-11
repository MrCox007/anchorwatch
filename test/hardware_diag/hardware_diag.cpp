// Hardware diagnostic for AnchorWatch
// Upload this to verify each component works individually.
// Open Serial Monitor at 115200 baud and follow instructions.

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static const int GPS_RX_PIN = 14;  // D5
static const int GPS_TX_PIN = 12;  // D6
static const int BUZZER_PIN = 13;  // D7
static const int BUTTON_PIN = 0;   // D3

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

enum TestPhase { TEST_BUZZER, TEST_BUTTON, TEST_GPS, TEST_DONE };
TestPhase phase = TEST_BUZZER;
unsigned long phaseStart = 0;
int buttonPresses = 0;
bool lastButtonState = HIGH;
unsigned long gpsStart = 0;
bool gpsFixed = false;

// ---- OLED (I2C: SDA=D2/GPIO4, SCL=D1/GPIO5) ----
static const uint8_t OLED_ADDR = 0x3C;  // try 0x3D if the screen stays blank
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOK = false;

// Forward declarations (defined later in this file).
void printResults();

void printHeader(const char* title) {
  Serial.println();
  Serial.println("========================================");
  Serial.print("  TEST: ");
  Serial.println(title);
  Serial.println("========================================");
}

void startPhase(TestPhase p) {
  phase = p;
  phaseStart = millis();
}

// Map satellite count to a 0..5 "bars" signal level.
int signalBars(uint32_t sats) {
  if (sats >= 10) return 5;
  if (sats >= 8)  return 4;
  if (sats >= 6)  return 3;
  if (sats >= 4)  return 2;
  if (sats >= 1)  return 1;
  return 0;
}

// Draw a 5-bar signal meter with its top-left corner at (x, y).
void drawSignalBars(int x, int y, int bars) {
  const int barW = 4, gap = 2, maxH = 14;
  for (int i = 0; i < 5; i++) {
    int h = (maxH * (i + 1)) / 5;
    int bx = x + i * (barW + gap);
    int by = y + (maxH - h);
    if (i < bars) display.fillRect(bx, by, barW, h, SSD1306_WHITE);
    else          display.drawRect(bx, by, barW, h, SSD1306_WHITE);
  }
}

// Show a title + up to two message lines on the OLED (no-op if no screen).
void oledStatus(const char* title, const char* line1 = "", const char* line2 = "") {
  if (!oledOK) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(title);
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
  display.setCursor(0, 16);
  display.println(line1);
  display.setCursor(0, 28);
  display.println(line2);
  display.display();
}

// Live GPS signal screen used during the GPS test.
void oledSignal() {
  if (!oledOK) return;
  uint32_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  bool hasFix = gps.location.isValid();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("GPS signal");
  drawSignalBars(128 - (5 * 6), 0, signalBars(sats));

  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(sats);
  display.setTextSize(1);
  display.setCursor(display.getCursorX(), 23);
  display.print(" sats");

  display.setCursor(0, 36);
  display.print("HDOP: ");
  if (gps.hdop.isValid()) display.print(gps.hdop.hdop(), 1);
  else display.print("--");

  display.setCursor(0, 46);
  display.print("Fix:  ");
  display.print(hasFix ? "YES" : "no");

  display.setCursor(0, 56);
  if (hasFix) {
    display.print(gps.location.lat(), 4);
    display.print(",");
    display.print(gps.location.lng(), 4);
  } else {
    display.print("waiting for sky...");
  }
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("######################################");
  Serial.println("#   AnchorWatch Hardware Diagnostic   #");
  Serial.println("######################################");
  Serial.println();

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);

  // --- Test 0: OLED ---
  Wire.begin();  // SDA=D2 (GPIO4), SCL=D1 (GPIO5)
  printHeader("OLED (D2 SDA, D1 SCL)");
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) {
    Serial.println("[OK] OLED found at 0x3C.");
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Anchor");
    display.println("Watch");
    display.setTextSize(1);
    display.setCursor(0, 40);
    display.println("Diagnostic");
    display.display();
    delay(1500);
  } else {
    Serial.println("[FAIL] OLED not found at 0x3C.");
    Serial.println("  Try 0x3D, or check SDA->D2, SCL->D1, VCC->3V3, GND->G.");
    Serial.println("  (Continuing - other tests still run.)");
  }

  // --- Test 1: Buzzer ---
  oledStatus("TEST: Buzzer", "Listen for", "3 beeps...");
  printHeader("BUZZER (D7/GPIO13)");
  Serial.println("Buzzer should beep 3 times...");

  for (int i = 0; i < 3; i++) {
    Serial.print("  Beep ");
    Serial.println(i + 1);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(300);
  }
  Serial.println("[OK] If you heard 3 beeps, buzzer works!");
  Serial.println("[FAIL] No sound? Check wiring: D7 -> buzzer+ -> GND");

  // --- Test 2: LED ---
  oledStatus("TEST: LED", "Watch the", "blue LED x3");
  printHeader("BUILT-IN LED");
  Serial.println("LED should blink 3 times...");

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, LOW);  // ESP8266 LED is active LOW
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
  }
  Serial.println("[OK] If LED blinked, it works!");

  // --- Test 3: Button ---
  oledStatus("TEST: Button", "Press button", "3 times");
  printHeader("BUTTON (D3/GPIO0)");
  Serial.println("Press the button 3 times within 10 seconds...");
  startPhase(TEST_BUTTON);
}

void testButton() {
  bool currentState = digitalRead(BUTTON_PIN);

  if (lastButtonState == HIGH && currentState == LOW) {
    buttonPresses++;
    Serial.print("  Button press detected! (");
    Serial.print(buttonPresses);
    Serial.println("/3)");
    char buf[20];
    snprintf(buf, sizeof(buf), "%d / 3 presses", buttonPresses);
    oledStatus("TEST: Button", buf);
    delay(50); // debounce
  }
  lastButtonState = currentState;

  if (buttonPresses >= 3) {
    Serial.println("[OK] Button works!");
    printHeader("GPS MODULE (D5/GPIO14 RX, D6/GPIO12 TX)");
    Serial.println("Waiting for GPS data (up to 120 sec)...");
    Serial.println("Note: GPS needs clear sky view for first fix.");
    gpsSerial.begin(9600);
    gpsStart = millis();
    startPhase(TEST_GPS);
  }

  if (millis() - phaseStart > 10000 && buttonPresses < 3) {
    Serial.print("[WARN] Only ");
    Serial.print(buttonPresses);
    Serial.println(" presses detected.");
    if (buttonPresses == 0) {
      Serial.println("[FAIL] No presses? Check wiring: D3 -> button -> GND");
    }
    printHeader("GPS MODULE (D5/GPIO14 RX, D6/GPIO12 TX)");
    Serial.println("Waiting for GPS data (up to 120 sec)...");
    gpsSerial.begin(9600);
    gpsStart = millis();
    startPhase(TEST_GPS);
  }
}

void testGPS() {
  bool dataReceived = false;

  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
    dataReceived = true;

    // Print raw NMEA for debugging (first few chars)
    Serial.print(c);
  }

  if (dataReceived && !gpsFixed) {
    Serial.println();
    Serial.println("  [OK] Receiving serial data from GPS!");
    Serial.print("  Chars processed: ");
    Serial.println(gps.charsProcessed());
    Serial.print("  Sentences with fix: ");
    Serial.println(gps.sentencesWithFix());
    Serial.print("  Failed checksum: ");
    Serial.println(gps.failedChecksum());
  }

  // Refresh the OLED signal screen ~4x/sec while waiting.
  static unsigned long lastOled = 0;
  if (!gpsFixed && millis() - lastOled > 250) {
    lastOled = millis();
    oledSignal();
  }

  if (gps.location.isValid() && !gpsFixed) {
    gpsFixed = true;
    Serial.println();
    Serial.println("  *** GPS FIX ACQUIRED! ***");
    Serial.print("  Latitude:   ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("  Longitude:  ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("  Satellites: ");
    Serial.println(gps.satellites.value());
    Serial.print("  HDOP:       ");
    Serial.println(gps.hdop.hdop());
    Serial.println("[OK] GPS module works perfectly!");

    startPhase(TEST_DONE);
    printResults();
  }

  if (millis() - gpsStart > 120000 && !gpsFixed) {
    Serial.println();
    if (gps.charsProcessed() < 10) {
      Serial.println("[FAIL] No data from GPS at all!");
      Serial.println("  Check wiring:");
      Serial.println("  - GPS TX -> D5 (GPIO14)");
      Serial.println("  - GPS RX -> D6 (GPIO12)");
      Serial.println("  - GPS VCC -> 3.3V or VIN");
      Serial.println("  - GPS GND -> GND");
      Serial.println("  - Is GPS LED blinking?");
    } else {
      Serial.println("[WARN] GPS data received but no fix yet.");
      Serial.print("  Chars: ");
      Serial.println(gps.charsProcessed());
      Serial.print("  Sentences: ");
      Serial.println(gps.sentencesWithFix());
      Serial.println("  Try outdoors with clear sky view.");
      Serial.println("  Cold start can take 1-10 minutes.");
    }

    startPhase(TEST_DONE);
    printResults();
  }

  // Periodic status during GPS wait
  static unsigned long lastStatus = 0;
  if (millis() - lastStatus > 10000 && !gpsFixed) {
    lastStatus = millis();
    unsigned long elapsed = (millis() - gpsStart) / 1000;
    Serial.println();
    Serial.print("  [");
    Serial.print(elapsed);
    Serial.print("s] Chars: ");
    Serial.print(gps.charsProcessed());
    Serial.print(" | Sentences: ");
    Serial.print(gps.sentencesWithFix());
    Serial.print(" | Sats: ");
    Serial.println(gps.satellites.value());
  }
}

void printResults() {
  Serial.println();
  Serial.println("######################################");
  Serial.println("#        DIAGNOSTIC COMPLETE          #");
  Serial.println("######################################");
  Serial.println();
  Serial.print("  Buzzer:  Listen for 3 beeps at startup\n");
  Serial.print("  LED:     Check if it blinked\n");
  Serial.print("  Button:  ");
  Serial.print(buttonPresses);
  Serial.println("/3 presses detected");
  Serial.print("  GPS:     ");
  if (gpsFixed) {
    Serial.println("FIX OK!");
  } else if (gps.charsProcessed() > 10) {
    Serial.println("Data OK, no fix (try outdoors)");
  } else {
    Serial.println("NO DATA (check wiring)");
  }
  Serial.println();
  Serial.println("Upload main firmware when all tests pass.");

  if (oledOK) {
    char b[24];
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Diagnostic done");
    display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
    display.setCursor(0, 16);
    snprintf(b, sizeof(b), "Button: %d/3", buttonPresses);
    display.println(b);
    display.setCursor(0, 28);
    if (gpsFixed) display.println("GPS: FIX OK");
    else if (gps.charsProcessed() > 10) display.println("GPS: data,no fix");
    else display.println("GPS: NO DATA");
    display.display();
  }
}

void loop() {
  switch (phase) {
    case TEST_BUTTON:
      testButton();
      break;
    case TEST_GPS:
      testGPS();
      break;
    case TEST_DONE:
      // Idle - blink LED slowly
      digitalWrite(LED_BUILTIN, (millis() / 1000) % 2 == 0 ? LOW : HIGH);
      break;
    default:
      break;
  }
}
