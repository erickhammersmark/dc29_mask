#include <Adafruit_BME680.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoLowPower.h>
#include <Debouncinator.h>
#include <qdec.h>
#include <Wire.h>

struct updatingValue
{
  int cur;
  int prev;
};

bool setValue(struct updatingValue* uV, int newValue)
{
  if (uV->cur != newValue)
  {
    uV->prev = uV->cur;
    uV->cur = newValue;
    return true;
  }
  return false;
}

void incrValue(struct updatingValue* uV)
{
  setValue(uV, uV->cur + 1);
}

int diff(struct updatingValue* uV)
{
    return uV->cur - uV->prev;
}

enum mode
{
  MODE_DEFAULT,
  MODE_RAINBOW,
  MODE_AUDIO,
  MODE_BRIGHTNESS,
  MODE_OFF,
  MODE_FIRST = MODE_DEFAULT,
  MODE_LAST = MODE_OFF,
};

#define KNOB_SWITCH_PIN 1
#define PIEZO_PIN 5
#define NEO_GRBW_PIN 6
#define NEO_GRB_PIN 7
#define KNOB_A_PIN 8
#define KNOB_B_PIN 9
#define AMP_DS_PIN 19
#define MIC_PIN A2

bool asleep = false;

#define AUDIO_READ_INTERVAL_MS 50

int audio_high = 1024;
uint32_t next_audio_read_time_ms = millis();

#define NUM_NEO_GRBW 72
#define NUM_NEO_GRB 38
int num_pixels = NUM_NEO_GRBW + NUM_NEO_GRB;

bool neo_grbw_enabled = true;
bool neo_grb_enabled = true;

double light_factor;
double light_factors[3] = {1.0, 1.0, 1.0};

uint8_t brightness;
uint8_t neo_grbw_brightness;
uint8_t neo_grb_brightness;
double cycle_time_sec;

uint16_t rainbow_pixel = 0;
uint16_t rainbow_pos = 0;

enum mode knob_mode = MODE_FIRST;
bool knob_enabled = false;

struct updatingValue knob_pos, knob_clicks;

unsigned long long_press_start_time_ms;
unsigned long long_press_interval_ms = 2000;

#define FLASH_INTERVAL_MS 500
unsigned long next_flash_time_ms;

#define PRESSURE_HISTORY_DEPTH 18
unsigned long bme_reading_end_time_ms;
double pressure_history[PRESSURE_HISTORY_DEPTH];
int pressure_history_idx;
double pressure_baseline;
int pressure_delta;

// qdec copypasta
::SimpleHacks::QDecoder qdec(KNOB_A_PIN, KNOB_B_PIN, true);
volatile int rotaryCount = 0;
int lastLoopDisplayedRotaryCount = 0;

void IsrForQDEC(void) { // do absolute minimum possible in any ISR ...
  ::SimpleHacks::QDECODER_EVENT event = qdec.update();
  if (event & ::SimpleHacks::QDECODER_EVENT_CW) {
    rotaryCount = rotaryCount + 1;
  } else if (event & ::SimpleHacks::QDECODER_EVENT_CCW) {
    rotaryCount = rotaryCount - 1;
  }
  return;
}
// qdec copypasta

Debouncinator db = Debouncinator();

Adafruit_BME680 bme;
Adafruit_NeoPixel neo_grbw_strip(NUM_NEO_GRBW, NEO_GRBW_PIN, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel neo_grb_strip(NUM_NEO_GRB, NEO_GRB_PIN, NEO_GRB + NEO_KHZ800);


/*
 * 
 * Inputy things
 * 
 */

void update_baseline(uint32_t reading)
{
  double _reading = (double)reading / PRESSURE_HISTORY_DEPTH;
  pressure_baseline -= pressure_history[pressure_history_idx];
  pressure_baseline += _reading;
  pressure_history[pressure_history_idx++] = _reading;
  if (pressure_history_idx >= PRESSURE_HISTORY_DEPTH)
  {
    pressure_history_idx = 0;
  }
}

void update_pressure()
{
  uint32_t pressure = 0;
  
  if (!bme_reading_end_time_ms)
  {
    bme_reading_end_time_ms  = bme.beginReading();
  }
  else if (millis() >= bme_reading_end_time_ms)
  {
    bme.endReading();
    pressure = bme.pressure;
    bme_reading_end_time_ms = 0;
    if (pressure)
    {
      pressure_delta = pressure - (uint32_t)pressure_baseline;
      update_baseline(pressure);
    }
  }
}

void wakeup_isr()
{
  if (asleep)
  {
    powerOn();
  }
}

void long_press()
{
  detachInterrupt(digitalPinToInterrupt(KNOB_A_PIN));
  detachInterrupt(digitalPinToInterrupt(KNOB_B_PIN));
  LowPower.attachInterruptWakeup(digitalPinToInterrupt(KNOB_SWITCH_PIN), wakeup_isr, FALLING);
  setAllPixels(0, 0, 0);
  asleep = true;
  LowPower.deepSleep();
  asleep = false;
  detachInterrupt(digitalPinToInterrupt(KNOB_SWITCH_PIN));
}


bool update_knob_switch()
{
  /*
   * The debouncinator will protect us from bounces in both .read() and .fired()
   * So read() will only mean a fully debounced trigger state, and fired() will
   * only mean a newly entered trigger state (i.e. new since the last time fired()
   * was called). I think we could get away with moving the fired() conditional
   * to the top and just return true/false at the bottom with no rv variable,
   * but this works.
   */
   
  bool rv = false;
  
  if (db.read(KNOB_SWITCH_PIN))
  {
    // switch is pressed
    if (long_press_start_time_ms && (millis() - long_press_start_time_ms > long_press_interval_ms))
    {
      long_press();
      long_press_start_time_ms = 0;
      db.fired(KNOB_SWITCH_PIN); // clear fired state by reading it
    }
  }
  else
  {
    // switch is not pressed
    if (long_press_start_time_ms)
    {
      // we had been waiting for a long press, so click instead
      long_press_start_time_ms = 0;
      incrValue(&knob_clicks);
      knob_enabled = not knob_enabled;
      rv = true;
    }
  }
  if (db.fired(KNOB_SWITCH_PIN))
  {
    long_press_start_time_ms = millis();
  }
  return rv;
}

bool update_knob()
{
  // update knob_pos even if !knob_enabled, because we don't want
  // its position to jump when it does get enabled
  bool rv;
  rv = setValue(&knob_pos, rotaryCount);

  if (!rv)
    return false;

  if (!knob_enabled)
  {
    if (knob_mode == MODE_BRIGHTNESS)
    {
      brightness = max(min(brightness + 25 * diff(&knob_pos), 255), 0);
      return true;
    }
    return false;
  }

  int new_pos = (int)knob_mode + diff(&knob_pos);
  if (new_pos < 0)
  {
    new_pos = (int)MODE_LAST;
  }
  knob_mode = (enum mode)new_pos;

  if (knob_mode > MODE_LAST)
  {
    knob_mode = MODE_FIRST;
  }
  else if (knob_mode < MODE_FIRST)
  {
    knob_mode = MODE_LAST;
  }

  if (knob_mode == MODE_BRIGHTNESS)
  {
    next_flash_time_ms = millis() + FLASH_INTERVAL_MS;
  }

  return true;
}


/*
 * 
 * Outputy Things
 * 
 */

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t neo_grb_wheel(byte WheelPos)
{
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85)
  {
   return neo_grb_strip.Color(255 - WheelPos * 3, 0, WheelPos * 3, 0);
  }
  else if(WheelPos < 170)
  {
    WheelPos -= 85;
   return neo_grb_strip.Color(0, WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else
  {
   WheelPos -= 170;
   return neo_grb_strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
  }
}

// applies brightness to a strip.Color
uint32_t getColor(uint32_t c)
{
  uint16_t r, g, b, w;
  w = 0 >> 24 & 0xFF;
  r = c >> 16 & 0xFF;
  g = c >>  8 & 0xFF;
  b = c & 0xFF;
  r = (brightness * r) / 255;
  g = (brightness * g) / 255;
  b = (brightness * b) / 255;
  w = (brightness * w) / 255;
  return neo_grbw_strip.Color(r, g, b, w);
}

void rainbowCycle()
{
  digitalWrite(AMP_DS_PIN, LOW);
  setPixel(rainbow_pixel, getColor(neo_grb_wheel(((rainbow_pixel * 256 / num_pixels) + rainbow_pos) & 255)));
  rainbow_pixel += 1;
  if (rainbow_pixel >= num_pixels)
  {
    rainbow_pixel = 0;
    rainbow_pos += 1;
    if (rainbow_pos >= 256*5)
    {
      rainbow_pos = 0;
    }
    show_strips();
  }
}

void setPixel(uint16_t pixel, uint8_t red, uint8_t green, uint8_t blue)
{
  if (pixel < 0)
  {
    return;
  }

  if (neo_grbw_enabled)
  {
    if (pixel < NUM_NEO_GRBW)
    {
      red = (uint8_t)((uint16_t) red * neo_grbw_brightness / 255);
      blue = (uint8_t)((uint16_t) blue * neo_grbw_brightness / 255);
      green = (uint8_t)((uint16_t) green * neo_grbw_brightness / 255);
      neo_grbw_strip.setPixelColor(pixel, red, green, blue, 0);
      return;
    }
    pixel -= NUM_NEO_GRBW;
  }

  if (neo_grb_enabled)
  {
    if (pixel < NUM_NEO_GRB)
    {
      red = (uint8_t)((uint16_t) red * neo_grb_brightness / 255);
      blue = (uint8_t)((uint16_t) blue * neo_grb_brightness / 255);
      green = (uint8_t)((uint16_t) green * neo_grb_brightness / 255);
      neo_grb_strip.setPixelColor(pixel, red, green, blue);
      return;
    }
    pixel -= NUM_NEO_GRB;
  }
}

void setPixel(uint16_t pixel, uint32_t c)
{
  uint8_t w = 0 >> 24 & 0xFF;
  uint8_t r = c >> 16 & 0xFF;
  uint8_t g = c >>  8 & 0xFF;
  uint8_t b = c & 0xFF;
  setPixel(pixel, r, g, b);
}

void show_strips()
{
  if (neo_grbw_enabled)
  {
    neo_grbw_strip.show();
  }
  if (neo_grb_enabled)
  {
    neo_grb_strip.show();
  }
}

void setAllPixels(int red, int green, int blue)
{
  for (int i=0; i<num_pixels; i++)
  {
    setPixel(i, red, green, blue);
  }
  show_strips();
}

void update_lights()
{
  switch(knob_mode)
  {
    case MODE_DEFAULT:
      default_lights();
      break;
    case MODE_RAINBOW:
      rainbowLights();
      break;
    case MODE_AUDIO:
      audioLights();
      break;
    case MODE_BRIGHTNESS:
      flashLights();
      break;
    case MODE_OFF:
    default:
      setAllPixels(0, 0, 0);
      break;
  }
}

void flashLights()
{
  if (knob_enabled && millis() > next_flash_time_ms)
  {
    if (neo_grb_brightness == 0)
    {
      neo_grb_brightness = brightness;
      neo_grbw_brightness = brightness;
    }
    else
    {
      neo_grb_brightness = 0;
      neo_grbw_brightness = 0;
    }
    next_flash_time_ms += FLASH_INTERVAL_MS;
  }
  else if (!knob_enabled)
  {
    neo_grb_brightness = brightness;
    neo_grbw_brightness = brightness;
  }
  rainbowCycle();
}

void rainbowLights()
{
  neo_grb_brightness = brightness;
  neo_grbw_brightness = brightness;
  rainbowCycle();
}

void audioLights()
{
  digitalWrite(AMP_DS_PIN, HIGH);
  if (millis() >= next_audio_read_time_ms)
  {
    int audio = analogRead(MIC_PIN);
    if (audio > audio_high)
    {
      audio_high = audio;
    }
    int audioBrightness = brightness * audio / audio_high;
    neo_grb_brightness = audioBrightness;
    neo_grbw_brightness = audioBrightness;
    next_audio_read_time_ms += AUDIO_READ_INTERVAL_MS;
  }
  rainbowCycle();
}

void default_lights()
{
  neo_grb_brightness = brightness;
  neo_grbw_brightness = brightness;

  digitalWrite(AMP_DS_PIN, LOW);

  uint8_t red, green, blue;
  
  double rads;
  uint32_t cycle_time_ms = (uint32_t)(cycle_time_sec * 1000);
  uint32_t pos = (millis() % cycle_time_ms) * num_pixels / cycle_time_ms;

  double diff = (100 * 6.2832) / (double) num_pixels;
  double light_factor = 1.0;
  for (int i=0; i<3; i++)
  {
    light_factors[i] -= (light_factors[i] - light_factor) / 2;
  }

  if (pressure_delta > 1000)
  {
    // baseline not yet established
  }
  else if (pressure_delta < 0)
  {
    light_factors[1] += (double)pressure_delta / 10.0;
  }
  else if (pressure_delta > 0)
  {
    light_factors[0] += (double)(0 - pressure_delta) / 10.0;
  }

  for (double rads = 0.0; rads < 100 * 6.2832; rads += diff)
  {
    if (pos >= num_pixels)
    {
      pos = 0;
    }
    red = min((uint8_t)(sin(rads)*light_factors[0]), 8);
    green = min((uint8_t)(cos(rads)*light_factors[1]), 8);
    blue = min((uint8_t)(tan(rads) * light_factors[2]), 1);
    setPixel(pos++, red, green, blue);
  }
  show_strips();
}

void post()
{
  setAllPixels(32, 0, 0);
  delay(250);
  setAllPixels(0, 32, 0);
  delay(250);
  setAllPixels(0, 0, 32);
  delay(250);
}

/*
 * 
 * setup() and loop()
 * and powerOn()
 * 
 */

void powerOn()
{
  // qdec copypasta
  qdec.begin();
  pinMode(KNOB_A_PIN, INPUT_PULLUP);
  pinMode(KNOB_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KNOB_A_PIN), IsrForQDEC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(KNOB_B_PIN), IsrForQDEC, CHANGE);
  // qdec copypasta
  
  pinMode(A2, INPUT);
  pinMode(AMP_DS_PIN, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  neo_grbw_strip.begin();
  neo_grb_strip.begin();

  // adafruit copypasta
  bme.begin();
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  bme_reading_end_time_ms = bme.beginReading();
  // adafruit copypasta
}

void setup()
{
  powerOn();

  db.init(KNOB_SWITCH_PIN);

  // A little struct that holds both the current and previous values.
  knob_pos = {0, 0};
  knob_clicks = {0, 0};
  
  brightness = 255;
  neo_grbw_brightness = 255;
  neo_grb_brightness = 255;
  next_flash_time_ms = 0;

  post();

  cycle_time_sec = 100.0;
  
  pressure_baseline = 0.0;
  pressure_history_idx = 0;
  memset(pressure_history, 0, sizeof(*pressure_history) * PRESSURE_HISTORY_DEPTH);
}

void loop()
{
  update_pressure();
  update_knob_switch();
  update_knob();
  update_lights();
}
