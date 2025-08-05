#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include <math.h>
#include <EEPROM.h>
#include <arduinoFFT.h>

// TFT Pins
#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

// Analog Input & Controls
#define CH1 A0
#define CH2 A1
#define HORZ_POT A15
#define VERT1_POT A14
#define VERT2_POT A13
#define TRIG_POT A12
#define TRIG_SW 2
#define MODE_SW 3
#define CURS_SW 4

// Display Dimensions
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define GRID_TOP 20
#define GRID_BOTTOM 220
#define GRID_LEFT 10
#define GRID_RIGHT 310

// Constants
#define SAMPLE_COUNT 512
#define FFT_SAMPLES 256
#define MAX_SAMPLING_RATE 100000  // ~100ksps
#define PERSISTENCE_DEPTH 8

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
arduinoFFT FFT = arduinoFFT();

// Oscilloscope variables
volatile uint16_t ch1_samples[SAMPLE_COUNT];
volatile uint16_t ch2_samples[SAMPLE_COUNT];
double fft_input[FFT_SAMPLES];
double fft_output[FFT_SAMPLES];
uint8_t persistence[PERSISTENCE_DEPTH][SAMPLE_COUNT];

uint16_t triggerLevel = 512;
uint16_t timeBase = 2;  // ms/div
float voltDiv1 = 0.5;   // V/div for CH1
float voltDiv2 = 0.5;   // V/div for CH2
bool triggerState = false;
bool triggerEdge = RISING;
bool autoTrigger = true;
bool ch1_enabled = true;
bool ch2_enabled = false;
bool cursorsEnabled = false;
bool spectrumMode = false;
bool persistenceMode = false;
bool measurementsEnabled = true;

// Measurement variables
float vpp1 = 0, vpp2 = 0;
float freq1 = 0, freq2 = 0;
float vmax1 = 0, vmax2 = 0;
float vmin1 = 0, vmin2 = 0;
float vavg1 = 0, vavg2 = 0;
float vrms1 = 0, vrms2 = 0;
float duty1 = 0;

// Cursors
uint16_t cursorX1 = 80;
uint16_t cursorX2 = 240;
uint16_t cursorY1 = 60;
uint16_t cursorY2 = 180;

// UI Colors
#define GRID_COLOR ILI9341_DARKGREEN
#define CH1_COLOR ILI9341_YELLOW
#define CH2_COLOR ILI9341_CYAN
#define CURSOR_COLOR ILI9341_MAGENTA
#define TRIG_COLOR ILI9341_RED
#define TEXT_COLOR ILI9341_WHITE

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_SW, INPUT_PULLUP);
  pinMode(MODE_SW, INPUT_PULLUP);
  pinMode(CURS_SW, INPUT_PULLUP);
  
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  
  // Load settings from EEPROM
  loadSettings();
  
  drawGrid();
  drawUI();
  
  // Optimize ADC
  ADCSRA = (ADCSRA & 0xF8) | 0x04;  // 16 prescaler (1MHz)
  ADMUX = 0x40;  // A0 input, right-adjusted
}

void loop() {
  static uint32_t lastUpdate = 0;
  
  readControls();
  captureWaveform();
  
  if(millis() - lastUpdate > 50) {
    processMeasurements();
    updateDisplay();
    lastUpdate = millis();
  }
}

void readControls() {
  // Read potentiometers
  timeBase = 1 + analogRead(HORZ_POT) / 103;  // 1-10 ms/div
  voltDiv1 = 0.1 + analogRead(VERT1_POT) / 2048.0;  // 0.1-1 V/div
  voltDiv2 = 0.1 + analogRead(VERT2_POT) / 2048.0;  // 0.1-1 V/div
  triggerLevel = analogRead(TRIG_POT);
  
  // Read switches
  if(digitalRead(TRIG_SW) == LOW) {
    triggerEdge = !triggerEdge;
    delay(200);
  }
  
  if(digitalRead(MODE_SW) == LOW) {
    spectrumMode = !spectrumMode;
    delay(200);
  }
  
  if(digitalRead(CURS_SW) == LOW) {
    cursorsEnabled = !cursorsEnabled;
    delay(200);
  }
}

void captureWaveform() {
  uint32_t sampleInterval = timeBase * 1000 / (GRID_RIGHT - GRID_LEFT) * 10;
  bool triggered = false;
  uint16_t pretTrig = SAMPLE_COUNT / 4;
  uint16_t index = 0;

  // Auto-trigger fallback
  if(autoTrigger) triggerState = true;

  while(!triggered) {
    uint16_t adc_val = analogRead(CH1);
    adc_val = (analogRead(CH1) + adc_val) / 2;  // Oversampling

    // Trigger detection
    if(triggerState) {
      if(triggerEdge == RISING) {
        if(adc_val >= triggerLevel) triggered = true;
      } else {
        if(adc_val <= triggerLevel) triggered = true;
      }
    } else {
      // Reset trigger
      if(triggerEdge == RISING) {
        if(adc_val < triggerLevel - 20) triggerState = true;
      } else {
        if(adc_val > triggerLevel + 20) triggerState = true;
      }
    }

    // Store samples
    ch1_samples[index] = ch1_enabled ? analogRead(CH1) : 0;
    ch2_samples[index] = ch2_enabled ? analogRead(CH2) : 0;
    
    // Update persistence buffer
    if(persistenceMode) {
      for(int p = PERSISTENCE_DEPTH-1; p > 0; p--) {
        persistence[p][index] = persistence[p-1][index];
      }
      persistence[0][index] = ch1_samples[index];
    }
    
    index = (index + 1) % SAMPLE_COUNT;
    delayMicroseconds(sampleInterval);
  }

  // Capture post-trigger samples
  for(uint16_t i = 0; i < SAMPLE_COUNT - pretTrig; i++) {
    ch1_samples[(index + i) % SAMPLE_COUNT] = ch1_enabled ? analogRead(CH1) : 0;
    ch2_samples[(index + i) % SAMPLE_COUNT] = ch2_enabled ? analogRead(CH2) : 0;
    delayMicroseconds(sampleInterval);
  }
  
  // Prepare FFT data if needed
  if(spectrumMode) {
    for(int i=0; i<FFT_SAMPLES; i++) {
      fft_input[i] = ch1_samples[i * SAMPLE_COUNT / FFT_SAMPLES];
    }
  }
}

void processMeasurements() {
  // Channel 1 measurements
  vmin1 = 4096;
  vmax1 = 0;
  vavg1 = 0;
  float sumSquares = 0;
  uint16_t crossings = 0;
  bool lastState = (ch1_samples[0] > vavg1);
  uint16_t highTime = 0;

  for(uint16_t i=0; i<SAMPLE_COUNT; i++) {
    uint16_t val = ch1_samples[i];
    if(val < vmin1) vmin1 = val;
    if(val > vmax1) vmax1 = val;
    vavg1 += val;
    sumSquares += val * val;
    
    // Frequency calculation
    bool currentState = (val > vavg1);
    if(currentState != lastState) {
      crossings++;
      lastState = currentState;
    }
    if(currentState) highTime++;
  }
  
  vavg1 /= SAMPLE_COUNT;
  vrms1 = sqrt(sumSquares / SAMPLE_COUNT) * (5.0/1023.0);
  vpp1 = (vmax1 - vmin1) * (5.0 / 1023.0);
  duty1 = (highTime * 100.0) / SAMPLE_COUNT;
  
  float timeWindow = timeBase * (GRID_RIGHT - GRID_LEFT) / 10.0;
  freq1 = (crossings / 2) * (1000.0 / timeWindow);
  
  // FFT processing if in spectrum mode
  if(spectrumMode) {
    FFT.Windowing(fft_input, FFT_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(fft_input, fft_output, FFT_SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(fft_input, fft_output, FFT_SAMPLES);
  }
}

void updateDisplay() {
  tft.fillRect(GRID_LEFT, GRID_TOP, 
              GRID_RIGHT - GRID_LEFT, 
              GRID_BOTTOM - GRID_TOP, 
              ILI9341_BLACK);
  
  drawGrid();
  
  if(persistenceMode) {
    drawPersistence();
  }
  
  if(spectrumMode) {
    drawSpectrum();
  } else {
    drawWaveform();
  }
  
  if(cursorsEnabled) {
    drawCursors();
  }
  
  drawTriggerLevel();
  drawUI();
  
  if(measurementsEnabled) {
    drawMeasurements();
  }
}

void drawGrid() {
  tft.drawRect(GRID_LEFT, GRID_TOP, 
              GRID_RIGHT - GRID_LEFT, 
              GRID_BOTTOM - GRID_TOP, 
              GRID_COLOR);
  
  // Vertical lines
  for(int x = GRID_LEFT + 30; x < GRID_RIGHT; x += 30) {
    tft.drawFastVLine(x, GRID_TOP, GRID_BOTTOM - GRID_TOP, GRID_COLOR);
  }
  
  // Horizontal lines
  for(int y = GRID_TOP + 20; y < GRID_BOTTOM; y += 20) {
    tft.drawFastHLine(GRID_LEFT, y, GRID_RIGHT - GRID_LEFT, GRID_COLOR);
  }
}

void drawWaveform() {
  // Channel 1
  for(int i=0; i<SAMPLE_COUNT-1; i++) {
    int x1 = map(i, 0, SAMPLE_COUNT, GRID_LEFT, GRID_RIGHT);
    int y1 = map(ch1_samples[i], 0, 1023, GRID_BOTTOM, GRID_TOP);
    int x2 = map(i+1, 0, SAMPLE_COUNT, GRID_LEFT, GRID_RIGHT);
    int y2 = map(ch1_samples[i+1], 0, 1023, GRID_BOTTOM, GRID_TOP);
    tft.drawLine(x1, y1, x2, y2, CH1_COLOR);
  }
  
  // Channel 2
  if(ch2_enabled) {
    for(int i=0; i<SAMPLE_COUNT-1; i++) {
      int x1 = map(i, 0, SAMPLE_COUNT, GRID_LEFT, GRID_RIGHT);
      int y1 = map(ch2_samples[i], 0, 1023, GRID_BOTTOM, GRID_TOP);
      int x2 = map(i+1, 0, SAMPLE_COUNT, GRID_LEFT, GRID_RIGHT);
      int y2 = map(ch2_samples[i+1], 0, 1023, GRID_BOTTOM, GRID_TOP);
      tft.drawLine(x1, y1, x2, y2, CH2_COLOR);
    }
  }
}

void drawPersistence() {
  for(int p=0; p<PERSISTENCE_DEPTH; p++) {
    for(int i=0; i<SAMPLE_COUNT; i++) {
      int x = map(i, 0, SAMPLE_COUNT, GRID_LEFT, GRID_RIGHT);
      int y = map(persistence[p][i], 0, 1023, GRID_BOTTOM, GRID_TOP);
      uint16_t color = ILI9341_GREEN;
      switch(p) {
        case 0: color = ILI9341_YELLOW; break;
        case 1: color = ILI9341_GREEN; break;
        case 2: color = ILI9341_CYAN; break;
        default: color = ILI9341_DARKGREEN;
      }
      tft.drawPixel(x, y, color);
    }
  }
}

void drawSpectrum() {
  // Find max magnitude for scaling
  double maxMag = 0;
  for(int i=2; i<FFT_SAMPLES/2; i++) {
    if(fft_input[i] > maxMag) maxMag = fft_input[i];
  }
  
  // Draw spectrum bars
  for(int i=2; i<FFT_SAMPLES/2; i++) {
    int x = map(i, 0, FFT_SAMPLES/2, GRID_LEFT, GRID_RIGHT);
    int barHeight = constrain(map(fft_input[i], 0, maxMag, 0, GRID_BOTTOM - GRID_TOP), 0, 200);
    tft.drawFastVLine(x, GRID_BOTTOM - barHeight, barHeight, CH1_COLOR);
  }
}

void drawCursors() {
  // Horizontal cursors
  tft.drawFastHLine(GRID_LEFT, cursorY1, GRID_RIGHT - GRID_LEFT, CURSOR_COLOR);
  tft.drawFastHLine(GRID_LEFT, cursorY2, GRID_RIGHT - GRID_LEFT, CURSOR_COLOR);
  
  // Vertical cursors
  tft.drawFastVLine(cursorX1, GRID_TOP, GRID_BOTTOM - GRID_TOP, CURSOR_COLOR);
  tft.drawFastVLine(cursorX2, GRID_TOP, GRID_BOTTOM - GRID_TOP, CURSOR_COLOR);
  
  // Cursor values
  tft.setTextColor(CURSOR_COLOR);
  tft.setCursor(GRID_RIGHT + 5, GRID_TOP);
  tft.print("dV: ");
  tft.print(fabs((cursorY2 - cursorY1) * (voltDiv1 * 5 / 200.0)), 2);
  tft.print("V");
  
  tft.setCursor(GRID_RIGHT + 5, GRID_TOP + 15);
  tft.print("dt: ");
  tft.print(fabs((cursorX2 - cursorX1) * (timeBase * 10 / 300.0)), 2);
  tft.print("ms");
}

void drawTriggerLevel() {
  uint16_t trig_y = map(triggerLevel, 0, 1023, GRID_BOTTOM, GRID_TOP);
  tft.drawFastHLine(GRID_LEFT, trig_y, 10, TRIG_COLOR);
  tft.fillTriangle(GRID_LEFT, trig_y-3, GRID_LEFT, trig_y+3, GRID_LEFT+8, trig_y, TRIG_COLOR);
}

void drawUI() {
  tft.fillRect(0, 0, SCREEN_WIDTH, GRID_TOP-1, ILI9341_BLACK);
  tft.fillRect(0, GRID_BOTTOM+1, SCREEN_WIDTH, SCREEN_HEIGHT - GRID_BOTTOM - 1, ILI9341_BLACK);
  
  tft.setTextColor(TEXT_COLOR);
  tft.setTextSize(1);
  
  // Channel info
  tft.setCursor(5, 5);
  tft.print("CH1:");
  tft.print(voltDiv1, 1);
  tft.print("V/div");
  
  if(ch2_enabled) {
    tft.setCursor(100, 5);
    tft.print("CH2:");
    tft.print(voltDiv2, 1);
    tft.print("V/div");
  }
  
  // Timebase
  tft.setCursor(200, 5);
  tft.print(timeBase);
  tft.print("ms/div");
  
  // Trigger info
  tft.setCursor(200, GRID_BOTTOM + 5);
  tft.print(triggerEdge ? "Rising" : "Falling");
  
  // Mode indicators
  tft.setCursor(5, GRID_BOTTOM + 5);
  if(spectrumMode) tft.print("SPECTRUM");
  if(persistenceMode) tft.print("PERSIST");
  if(cursorsEnabled) tft.print("CURSORS");
}

void drawMeasurements() {
  tft.fillRect(GRID_RIGHT + 5, GRID_TOP, 100, 100, ILI9341_BLACK);
  tft.setTextColor(CH1_COLOR);
  
  tft.setCursor(GRID_RIGHT + 5, GRID_TOP);
  tft.print("F: ");
  tft.print(freq1, 1);
  tft.print("Hz");
  
  tft.setCursor(GRID_RIGHT + 5, GRID_TOP + 15);
  tft.print("Vpp: ");
  tft.print(vpp1, 2);
  tft.print("V");
  
  tft.setCursor(GRID_RIGHT + 5, GRID_TOP + 30);
  tft.print("Duty: ");
  tft.print(duty1, 1);
  tft.print("%");
  
  tft.setCursor(GRID_RIGHT + 5, GRID_TOP + 45);
  tft.print("RMS: ");
  tft.print(vrms1, 2);
  tft.print("V");
}

void loadSettings() {
  // EEPROM structure:
  // 0: triggerEdge (1 byte)
  // 1: voltDiv1 (4 bytes)
  // 5: voltDiv2 (4 bytes)
  // 9: timeBase (2 bytes)
  
  triggerEdge = EEPROM.read(0);
  EEPROM.get(1, voltDiv1);
  EEPROM.get(5, voltDiv2);
  EEPROM.get(9, timeBase);
}

void saveSettings() {
  EEPROM.update(0, triggerEdge);
  EEPROM.put(1, voltDiv1);
  EEPROM.put(5, voltDiv2);
  EEPROM.put(9, timeBase);
}

// Add this to save on shutdown
void powerDown() {
  saveSettings();
  // Add hardware power down code here
}
