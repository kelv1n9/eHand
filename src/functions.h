#pragma once

#include <RF24.h>
#include <RF24Audio.h>
#include "EncButton.h"
#include <EEPROM.h>
#include <Vcc.h>

Vcc vcc(1);

// #define DEBUG_eHand

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
// D3 - Encoder B
// D4 - Encoder A
// D5 - Encoder switch

#define NAME "eHand"
#define VERSION "1.0.0"

#define LOW_BATT_VOLT 3.8
#define LED_BLINK 10000
#define LED_SHORT 200

#define LED_PIN 6

#define PTT_BUTTON_PIN 2
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 3
#define ENCODER_SWITCH_PIN 5

#define CSN_PIN 8
#define CE_PIN 7

const uint8_t channels[] = {90, 100, 110};
#define CHANNEL_COUNT (sizeof(channels) / sizeof(channels[0]))

#define SAVE_DELAY_MS 10000UL
#define BLINK_DELAY_MS 2000UL

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

bool isTx = false;
uint8_t channelIdx = 0;
uint8_t channel = channels[channelIdx];
uint8_t volume = 4;
uint8_t dataRateIdx = 1;
uint8_t txPowerIdx = 2;

bool blinkPending = false;
uint8_t blinkCountPending = 0;
uint32_t blinkWhenMs = 0;

void scheduleBlink(uint8_t count)
{
  blinkCountPending = count;
  blinkWhenMs = millis() + BLINK_DELAY_MS;
  blinkPending = true;
}

void doBlink(uint8_t count)
{
  bool vuActive = (TCCR0A & _BV(COM0A1));
  if (vuActive)
    TCCR0A &= ~_BV(COM0A1);

  for (uint8_t i = 0; i < count; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(LED_SHORT);
    digitalWrite(LED_PIN, LOW);
    delay(LED_SHORT);
  }

  if (isTx) {
    delay(1000);
    digitalWrite(LED_PIN, HIGH);
  }

  if (vuActive)
    TCCR0A |= _BV(COM0A1);
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

  bool channelOk = false;
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++)
  {
    if (config.channel == channels[i])
    {
      channelIdx = i;
      channelOk = true;
      break;
    }
  }

  bool ok = (config.volume <= 7) &&
            (config.dataRateIdx <= 2) &&
            channelOk &&
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