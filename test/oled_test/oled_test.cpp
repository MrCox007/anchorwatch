// OLED + GPS signal test for AnchorWatch
// Upload with:  pio run -e oled -t upload -t monitor
//
// Verifies the SSD1306 OLED works and shows live GPS signal strength
// (satellite count + signal bars + HDOP + fix status) on screen.
//
// Wiring:
//   OLED  SDA -> D2 (GPIO4),  SCL -> D1 (GPIO5),  VCC -> 3V3,  GND -> G
//   GPS   TX  -> D5 (GPIO14), RX  -> D6 (GPIO12), VCC -> 3V3,  GND -> G

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// ---- Display ----
static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;
static const int OLED_RESET = -1;       // shares the ESP reset line
static const uint8_t OLED_ADDR = 0x3C;  // most blue 0.96" panels; try 0x3D if blank
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---- GPS ---- (same pins as the main firmware)
static const int GPS_RX_PIN = 14;  // D5  (<- GPS TX)
static const int GPS_TX_PIN = 12;  // D6  (-> GPS RX)
static const uint32_t GPS_BAUD = 9600;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPSPlus gps;

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
  const int barW = 4;
  const int gap = 2;
  const int maxH = 14;
  for (int i = 0; i < 5; i++) {
    int h = (maxH * (i + 1)) / 5;
    int bx = x + i * (barW + gap);
    int by = y + (maxH - h);
    if (i < bars) {
      display.fillRect(bx, by, barW, h, SSD1306_WHITE);
    } else {
      display.drawRect(bx, by, barW, h, SSD1306_WHITE);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== OLED + GPS signal test ===");

  Wire.begin();  // ESP8266 default I2C: SDA=GPIO4 (D2), SCL=GPIO5 (D1)

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[FAIL] SSD1306 not found at 0x3C.");
    Serial.println("  - Try address 0x3D (change OLED_ADDR).");
    Serial.println("  - Check SDA->D2, SCL->D1, VCC->3V3, GND->G.");
    // Keep the onboard LED blinking so it's clear the sketch is alive.
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(250);
    }
  }
  Serial.println("[OK] OLED initialised.");

  // Splash screen so you can immediately confirm the display works.
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Anchor");
  display.println("Watch");
  display.setTextSize(1);
  display.setCursor(0, 40);
  display.println("OLED OK");
  display.println("Starting GPS...");
  display.display();
  delay(2000);

  gpsSerial.begin(GPS_BAUD);
  Serial.println("GPS serial started @ 9600");
}

void render() {
  uint32_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
  bool hasFix = gps.location.isValid();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Title + signal bars on the top row.
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("GPS signal");
  drawSignalBars(SCREEN_WIDTH - (5 * 6), 0, signalBars(sats));

  // Big satellite count.
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(sats);
  display.setTextSize(1);
  display.setCursor(display.getCursorX(), 23);  // align small label to the big digits
  display.print(" sats");

  // HDOP — horizontal accuracy; lower is better (<2 is great).
  display.setCursor(0, 36);
  display.print("HDOP: ");
  if (gps.hdop.isValid()) display.print(gps.hdop.hdop(), 1);
  else display.print("--");

  // Fix status.
  display.setCursor(0, 46);
  display.print("Fix:  ");
  display.print(hasFix ? "YES" : "no");

  // Position once we have a fix.
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

void loop() {
  // Feed every available byte to the GPS parser.
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Refresh the screen ~4x per second.
  static unsigned long lastDraw = 0;
  if (millis() - lastDraw > 250) {
    lastDraw = millis();
    render();
  }

  // Mirror status to serial, and warn if no GPS bytes are arriving at all.
  static unsigned long lastLog = 0;
  if (millis() - lastLog > 5000) {
    lastLog = millis();
    Serial.print("chars=");
    Serial.print(gps.charsProcessed());
    Serial.print(" sats=");
    Serial.print(gps.satellites.value());
    Serial.print(" fix=");
    Serial.println(gps.location.isValid() ? "yes" : "no");
    if (gps.charsProcessed() < 10) {
      Serial.println("[WARN] No GPS data — check TX->D5, RX->D6, baud 9600.");
    }
  }
}
