#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Pins
const byte SIG_PIN = A0;    // Signal input
const byte TIME_POT = A1;   // Timebase control
const byte TRIG_POT = A2;   // Trigger level

// Sampling
const int BUFFER_SIZE = 128;
int samples[BUFFER_SIZE];
float vpp, vmin, vmax, freq;
unsigned long sampleInterval = 1000;  // microseconds
unsigned long lastTriggerTime = 0;
bool triggered = false;

// Display state
enum DisplayMode { WAVEFORM, METER };
DisplayMode currentMode = WAVEFORM;

void setup() {
  Serial.begin(115200);
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.dim(true);
  
  analogReference(DEFAULT);
  pinMode(SIG_PIN, INPUT);
}

void loop() {
  // Read control potentiometers
  int timePotValue = analogRead(TIME_POT);
  int trigPotValue = analogRead(TRIG_POT);
  
  // Calculate timebase (10μs - 10ms range)
  sampleInterval = map(timePotValue, 0, 1023, 10, 10000);
  int triggerLevel = map(trigPotValue, 0, 1023, 0, 1023);
  
  // Capture waveform
  if (captureWaveform(triggerLevel)) {
    analyzeWaveform();
    
    // Toggle display mode on button press
    if (analogRead(A3) > 512) {
      currentMode = (currentMode == WAVEFORM) ? METER : WAVEFORM;
      delay(200);  // Debounce
    }
    
    // Update display
    display.clearDisplay();
    if (currentMode == WAVEFORM) {
      drawWaveform();
      drawMeasurements(triggerLevel);
    } else {
      drawAnalogMeter();
    }
    display.display();
  }
}

bool captureWaveform(int triggerLevel) {
  bool armed = false;
  unsigned long timeout = millis() + 500;
  
  // Wait for trigger
  while (!triggered) {
    int val = analogRead(SIG_PIN);
    
    if (!armed && val < triggerLevel - 20) {
      armed = true;  // Arm trigger when below threshold
    }
    
    if (armed && val >= triggerLevel) {
      triggered = true;
      break;
    }
    
    if (millis() > timeout) return false;  // Trigger timeout
  }
  
  // Capture samples
  for (int i = 0; i < BUFFER_SIZE; i++) {
    samples[i] = analogRead(SIG_PIN);
    delayMicroseconds(sampleInterval);
  }
  
  // Calculate frequency
  unsigned long currentTime = micros();
  if (lastTriggerTime > 0) {
    freq = 1000000.0 / (currentTime - lastTriggerTime);
  }
  lastTriggerTime = currentTime;
  
  triggered = false;
  return true;
}

void analyzeWaveform() {
  vmin = 1024;
  vmax = 0;
  
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (samples[i] < vmin) vmin = samples[i];
    if (samples[i] > vmax) vmax = samples[i];
  }
  
  vpp = (vmax - vmin) * (5.0 / 1024.0);
}

void drawWaveform() {
  // Draw grid
  for (int i = 0; i <= SCREEN_WIDTH; i += 16) {
    display.drawFastVLine(i, 0, SCREEN_HEIGHT - 16, SSD1306_INVERSE);
  }
  for (int j = 0; j <= SCREEN_HEIGHT - 16; j += 16) {
    display.drawFastHLine(0, j, SCREEN_WIDTH, SSD1306_INVERSE);
  }
  
  // Draw waveform
  for (int i = 1; i < BUFFER_SIZE; i++) {
    int y1 = map(samples[i-1], 0, 1023, SCREEN_HEIGHT - 18, 2);
    int y2 = map(samples[i], 0, 1023, SCREEN_HEIGHT - 18, 2);
    display.drawLine(i-1, y1, i, y2, WHITE);
  }
}

void drawMeasurements(int triggerLevel) {
  // Measurement area
  display.fillRect(0, SCREEN_HEIGHT - 16, SCREEN_WIDTH, 16, BLACK);
  display.drawFastHLine(0, SCREEN_HEIGHT - 17, SCREEN_WIDTH, WHITE);
  
  // Trigger indicator
  int trigY = map(triggerLevel, 0, 1023, SCREEN_HEIGHT - 18, 2);
  display.drawPixel(0, trigY, WHITE);
  display.drawPixel(1, trigY, WHITE);
  
  // Measurements
  display.setCursor(2, SCREEN_HEIGHT - 15);
  display.print("Vpp:");
  display.print(vpp, 1);
  display.print("V");
  
  display.setCursor(65, SCREEN_HEIGHT - 15);
  display.print("Freq:");
  if (freq < 1000) {
    display.print(freq, 0);
    display.print("Hz");
  } else {
    display.print(freq/1000, 1);
    display.print("kHz");
  }
}

void drawAnalogMeter() {
  // Meter frame
  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
  display.drawRect(1, 1, SCREEN_WIDTH-2, SCREEN_HEIGHT-2, WHITE);
  
  // Calculate needle position (0-180° range)
  float voltage = (vmin + vmax) / 2 * (5.0 / 1024.0);
  int angle = constrain(map(voltage * 100, 0, 500, 30, 150), 30, 150);
  
  // Draw meter face
  for (int i = 30; i <= 150; i += 30) {
    float rad = i * PI / 180;
    int x1 = 64 + 40 * cos(rad);
    int y1 = 48 + 40 * sin(rad);
    int x2 = 64 + 45 * cos(rad);
    int y2 = 48 + 45 * sin(rad);
    display.drawLine(x1, y1, x2, y2, WHITE);
    
    // Labels
    display.setCursor(x2 - 5, y2 - 5);
    display.print(map(i, 30, 150, 0, 5));
  }
  
  // Draw needle
  float rad = angle * PI / 180;
  display.drawLine(64, 48, 
                  64 + 40 * cos(rad), 
                  48 + 40 * sin(rad), WHITE);
  
  // Draw center
  display.fillCircle(64, 48, 3, WHITE);
  
  // Display voltage
  display.setCursor(45, SCREEN_HEIGHT - 12);
  display.print("Voltage: ");
  display.print(voltage, 2);
  display.print("V");
}
