/*
Controls:

- PTT hold ............ TX mode (LED ON)
- PTT release ......... RX mode (LED blink/VU)

- Encoder turn ........ Volume (0–7)
- Encoder press+turn .. Channel (100–115)
- Encoder click ....... Data Rate (250k/1M/2M)
- Enc turn+PTT hold.... TX Power (MIN/LOW/HIGH/MAX)

LED Indication Map:

TX Power (PTT + Encoder turn)
-------------------------------
- MIN  .......... 1 short flash
- LOW  .......... 2 short flashes
- HIGH .......... 3 short flashes
- MAX  .......... 1 long flash

Data Rate (Encoder click)
----------------------------
- 250 kbps ...... 1 slow flash
- 1 Mbps  ....... 2 medium flashes
- 2 Mbps  ....... 3 fast flashes

Channel (Encoder press + turn)
--------------------------------
Binary code, 4 flashes (short=0, long=1):

100 (0)  .... short short short short
101 (1)  .... long  short short short
102 (2)  .... short long  short short
103 (3)  .... long  long  short short
104 (4)  .... short short long  short
105 (5)  .... long  short long  short
106 (6)  .... short long  long  short
107 (7)  .... long  long  long  short
108 (8)  .... short short short long
109 (9)  .... long  short short long
110 (10) .... short long  short long
111 (11) .... long  long  short long
112 (12) .... short short long  long
113 (13) .... long  short long  long
114 (14) .... short long  long  long
115 (15) .... long  long  long  long
*/

#include <RF24.h>
#include <RF24Audio.h>
#include "EncButton.h"
#include <EEPROM.h>
#include <Vcc.h>

#define DEBUG_eHand

#ifdef DEBUG_eHand
#include "printf.h"
#define DBG(...)         \
  do                     \
  {                      \
    printf(__VA_ARGS__); \
  } while (0)
#else
#define DBG(...) \
  do             \
  {              \
  } while (0)
#endif

// Pin definitions
// A0 - Microphone input
// D9 - Speaker output
// D10 - Speaker output
// D6 - LED output
// D12 - SPI MISO
// D11 - SPI MOSI
// D13 - SPI SCK
// D7 - RF24 CE
// D8 - RF24 CSN
// D2 - PTT button
// D3 - Encoder A
// D4 - Encoder B
// D5 - Encoder switch

#define NAME "eHand"
#define VERSION "1.0.0"

#define LED_SHORT 200
#define LED_LONG 600
#define LED_PAUSE 200
#define LED_END 1000

#define LED_PIN 6

#define PTT_BUTTON_PIN 2
#define ENCODER_A_PIN 3
#define ENCODER_B_PIN 4
#define ENCODER_SWITCH_PIN 5

#define CSN_PIN 8
#define CE_PIN 7

#define CHANNEL_START 100
#define CHANNEL_COUNT 16
#define CH_MIN (CHANNEL_START)
#define CH_MAX (CHANNEL_START + CHANNEL_COUNT - 1)

#define SAVE_DELAY_MS 10000UL

struct Settings
{
  uint8_t channel;
  uint8_t volume;
  uint8_t dataRateIdx;
  uint8_t txPowerIdx;
};

Settings config;
bool isConfigEdited = false;
uint32_t configLastChangeTime = 0;

Button PTT(PTT_BUTTON_PIN);
EncButton encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_SWITCH_PIN);

RF24 radio(CE_PIN, CSN_PIN);
RF24Audio rfAudio(radio, 0);

const float VccCorrection = 1.0/1.0; // Measured Vcc by multimeter / V by reported Vcc
Vcc vcc(VccCorrection);

bool isTx = false;
uint8_t channel = CHANNEL_START;
uint8_t volume = 4;
uint8_t dataRateIdx = 1;
uint8_t txPowerIdx = 2;

int blinkType = -1;
int blinkValue;
uint32_t blinkStart;
uint32_t lastPowerTurnMs;
bool suppressLedDuringPower;

void rogerBeep() {
  // uint8_t _t1a = TCCR1A, _timsk1 = TIMSK1;
  // TIMSK1 = 0;
  // TCCR1A &= ~(_BV(COM1A1) | _BV(COM1B1) | _BV(COM1B0));

  tone(9, 880, 60);  
  delay(80);
  tone(9, 660, 80); 
  delay(100);
  noTone(9);
}

void blinkPower(uint8_t level)
{
  switch (level)
  {
  case 0: // MIN
    digitalWrite(LED_PIN, HIGH);
    delay(LED_SHORT);
    digitalWrite(LED_PIN, LOW);
    delay(LED_END);
    break;
  case 1: // LOW
    for (int i = 0; i < 2; i++)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(LED_SHORT);
      digitalWrite(LED_PIN, LOW);
      delay(LED_PAUSE);
    }
    delay(LED_END);
    break;
  case 2: // HIGH
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(LED_SHORT);
      digitalWrite(LED_PIN, LOW);
      delay(LED_PAUSE);
    }
    delay(LED_END);
    break;
  case 3: // MAX
    digitalWrite(LED_PIN, HIGH);
    delay(LED_LONG);
    digitalWrite(LED_PIN, LOW);
    delay(LED_END);
    break;
  }
}

void blinkRate(uint8_t rate)
{
  // 0=250kbps, 1=1Mbps, 2=2Mbps
  int count = rate + 1;
  int dur = (rate == 0 ? 600 : (rate == 1 ? 300 : 150)); // slow / medium / fast
  for (int i = 0; i < count; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(dur);
    digitalWrite(LED_PIN, LOW);
    delay(dur);
  }
  delay(LED_END);
}

void blinkChannel(uint8_t ch)
{
  for (int i = 3; i >= 0; --i)
  {
    bool bit = (ch >> i) & 0x01;
    digitalWrite(LED_PIN, HIGH);
    delay(bit ? LED_LONG : LED_SHORT);
    digitalWrite(LED_PIN, LOW);
    delay(LED_PAUSE);
  }
  delay(LED_END);
}

void onPowerChanged(uint8_t level)
{
  blinkStart = millis();
  blinkType = 0;
  blinkValue = level;
}

void onRateChanged(uint8_t rate)
{
  blinkStart = millis();
  blinkType = 1;
  blinkValue = rate;
}

void onChannelChanged(uint8_t ch)
{
  blinkStart = millis();
  blinkType = 2;
  blinkValue = ch;
}

void applyChannel()
{
  radio.setChannel(channel);
}

void applyDataRate()
{
  switch (dataRateIdx)
  {
  case 0:
    radio.setDataRate(RF24_250KBPS);
    DBG("Set data rate: 250kbps\n");
    break;
  case 1:
    radio.setDataRate(RF24_1MBPS);
    DBG("Set data rate: 1Mbps\n");
    break;
  case 2:
    radio.setDataRate(RF24_2MBPS);
    DBG("Set data rate: 2Mbps\n");
    break;
  }
}

void applyTxPower()
{
  switch (txPowerIdx)
  {
  case 0:
    radio.setPALevel(RF24_PA_MIN);
    DBG("Set Tx Power: MIN\n");
    break;
  case 1:
    radio.setPALevel(RF24_PA_LOW);
    DBG("Set Tx Power: LOW\n");
    break;
  case 2:
    radio.setPALevel(RF24_PA_HIGH);
    DBG("Set Tx Power: HIGH\n");
    break;
  case 3:
    radio.setPALevel(RF24_PA_MAX);
    DBG("Set Tx Power: MAX\n");
    break;
  }
}

void applyVolume()
{
  rfAudio.setVolume(volume);
}

void markConfigEdited()
{
  isConfigEdited = true;
  configLastChangeTime = millis();
}

void loadSettings()
{
  EEPROM.get(0, config);
  bool ok = (config.volume <= 7) &&
            (config.dataRateIdx <= 2) &&
            (config.channel >= CH_MIN && config.channel <= CH_MAX) &&
            (config.txPowerIdx <= 3);

  if (!ok)
  {
    config.volume = volume;
    config.dataRateIdx = dataRateIdx;
    config.channel = channel;
    config.txPowerIdx = txPowerIdx;
    EEPROM.put(0, config);
  }

  volume = config.volume;
  dataRateIdx = config.dataRateIdx;
  channel = config.channel;
  txPowerIdx = config.txPowerIdx;

  applyTxPower();
  applyDataRate();
  applyChannel();
  applyVolume();
}

void saveSettings()
{
  if (isConfigEdited && (millis() - configLastChangeTime >= SAVE_DELAY_MS))
  {
    EEPROM.put(0, config);
    isConfigEdited = false;
    DBG("Settings saved to EEPROM\n");
  }
}

float readBatteryVoltage()
{
  float volts = vcc.Read_Volts();
  DBG("Battery: %f V\n", volts);

  return volts;
}

void setup()
{
#ifdef DEBUG_eHand
  Serial.begin(115200);
  printf_begin();
  delay(1000);
#endif
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  radio.begin();
  rfAudio.begin();

  loadSettings();

  DBG("Channel: %u, Volume: %u, dataRateIdx: %u, TxPowerIdx: %u\n", channel, volume, dataRateIdx, txPowerIdx);

#ifdef DEBUG_eHand
  radio.printDetails();
#endif

  rfAudio.receive();
  isTx = false;
}

void loop()
{
  static bool ledState;
  static uint32_t blinkTimer;

  encoder.tick();
  PTT.tick();

  // Power toggle when encoder turned while PTT held
  if (PTT.pressing() && encoder.turn())
  {
    if (isTx && !suppressLedDuringPower)
    {
      digitalWrite(LED_PIN, LOW);
      suppressLedDuringPower = true;
      lastPowerTurnMs = millis();
    }

    int step = (encoder.fast() ? 2 : 1) * encoder.dir();
    int idx = (int)txPowerIdx + (step > 0 ? 1 : -1);
    if (idx < 0)
      idx = 3;
    if (idx > 3)
      idx = 0;
    txPowerIdx = (uint8_t)idx;
    config.txPowerIdx = txPowerIdx;
    applyTxPower();
    onPowerChanged(txPowerIdx);
    markConfigEdited();
    DBG("TxPowerIdx: %u\n", txPowerIdx);
  }
  else if (!PTT.pressing() && encoder.turn())
  {
    int step = (encoder.fast() ? 2 : 1) * encoder.dir();

    // Channel toggle when encoder pressed and turned
    if (encoder.pressing())
    {
      int new_channel = (int)channel + step;
      if (new_channel < CH_MIN)
        new_channel = CH_MAX;
      if (new_channel > CH_MAX)
        new_channel = CH_MIN;
      channel = (uint8_t)new_channel;
      config.channel = channel;
      applyChannel();
      onChannelChanged(channel);
      markConfigEdited();
      DBG("Channel: %u\n", channel);
    }
    // Volume toggle when encoder turned
    else
    {
      int new_volume = (int)volume + step;
      if (new_volume < 0)
        new_volume = 0;
      if (new_volume > 7)
        new_volume = 7;
      volume = (uint8_t)new_volume;
      config.volume = volume;
      applyVolume();
      markConfigEdited();
      DBG("Volume: %u\n", volume);
    }
  }

  // Rate toggle on encoder click
  if (encoder.click())
  {
    dataRateIdx = (dataRateIdx + 1) % 3;
    config.dataRateIdx = dataRateIdx;
    applyDataRate();
    onRateChanged(dataRateIdx);
    markConfigEdited();
    DBG("dataRateIdx: %u\n", dataRateIdx);
  }

  // Start Transmitting while PTT hold
  if (PTT.press() && !isTx)
  {
    rfAudio.transmit();
    digitalWrite(LED_PIN, HIGH);
    isTx = true;
    DBG("Transmitting...\n");
  }
  // Stop Transmitting when PTT released
  if (PTT.release() && isTx)
  {
    digitalWrite(LED_PIN, LOW);
    rogerBeep(); //! Test function
    rfAudio.receive();
    isTx = false;
    suppressLedDuringPower = false;
    DBG("Receiving...\n");
  }

  // Blink LED when radio is waiting
  if (!isTx)
  {
    if (!(TCCR0A & _BV(COM0A1)))
    {
      if (millis() - blinkTimer >= 2000)
      {
        bool lowBat = (readBatteryVoltage() <= 3.7);
        if (lowBat)
        {
          digitalWrite(LED_PIN, HIGH);
          delay(120);
          digitalWrite(LED_PIN, LOW);
          delay(180);
          digitalWrite(LED_PIN, HIGH);
          delay(120);
          digitalWrite(LED_PIN, LOW);
        }
        else
        {
          digitalWrite(LED_PIN, HIGH);
          delay(120);
          digitalWrite(LED_PIN, LOW);
        }
        blinkTimer = millis();
      }
    }
  }

  // Handle LED indication after setting changes
  if (blinkType != -1 && millis() - blinkStart >= 1000)
  {
    bool vuActive = (TCCR0A & _BV(COM0A1));
    if (vuActive)
    {
      TCCR0A &= ~_BV(COM0A1);
    }

    switch (blinkType)
    {
    case 0:
      blinkPower(blinkValue);
      break;
    case 1:
      blinkRate(blinkValue);
      break;
    case 2:
      blinkChannel(blinkValue - 100);
      break;
    }

    if (vuActive)
    {
      TCCR0A |= _BV(COM0A1);
    }
    blinkType = -1;
  }

  if (isTx && suppressLedDuringPower && (millis() - lastPowerTurnMs >= 1000))
  {
    digitalWrite(LED_PIN, HIGH);
    suppressLedDuringPower = false;
  }
  saveSettings();
}
