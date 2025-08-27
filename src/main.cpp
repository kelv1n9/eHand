#include <RF24.h>
#include <RF24Audio.h>
#include "EncButton.h"
#include <EEPROM.h>

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

#define PTT_BUTTON_PIN 2
#define ENCODER_A_PIN 3
#define ENCODER_B_PIN 4
#define ENCODER_SWITCH_PIN 5

#define CSN_PIN 8
#define CE_PIN 7

#define CHANNEL_START 100
#define CHANNEL_COUNT 10
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

bool isTx = false;
uint8_t channel = CHANNEL_START;
uint8_t volume = 4;
uint8_t dataRateIdx = 1;
uint8_t txPowerIdx = 3;

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
    break;
  case 1:
    radio.setDataRate(RF24_1MBPS);
    break;
  case 2:
    radio.setDataRate(RF24_2MBPS);
    break;
  }
}

void applyTxPower()
{
  switch (txPowerIdx)
  {
  case 0:
    radio.setPALevel(RF24_PA_MIN);
    break;
  case 1:
    radio.setPALevel(RF24_PA_LOW);
    break;
  case 2:
    radio.setPALevel(RF24_PA_HIGH);
    break;
  case 3:
    radio.setPALevel(RF24_PA_MAX);
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
  }
}

void setup()
{
#ifdef DEBUG_eHand
  Serial.begin(115200);
  printf_begin();
  delay(1000);
#endif

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
  encoder.tick();
  PTT.tick();

  if (PTT.pressing())
  {
    // Power toggle when encoder turned while PTT held
    if (encoder.turn())
    {
      int step = (encoder.fast() ? 2 : 1) * encoder.dir();
      int idx = (int)txPowerIdx + (step > 0 ? 1 : -1);
      if (idx < 0)
        idx = 3;
      if (idx > 3)
        idx = 0;
      txPowerIdx = (uint8_t)idx;
      config.txPowerIdx = txPowerIdx;
      applyTxPower();
      markConfigEdited();
      DBG("TxPowerIdx: %u\n", txPowerIdx);
    }
    // Transmit while PTT held
    if (PTT.press() && !isTx)
    {
      rfAudio.transmit();
      isTx = true;
      DBG("Transmitting...\n");
    }
  }
  else
  {
    if (encoder.turn())
    {
      int step = (encoder.fast() ? 2 : 1) * encoder.dir();

      // Change channel when encoder pressed and turned
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
        markConfigEdited();
        DBG("Channel: %u\n", channel);
      }
      // Change volume when encoder turned
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

    // Change rate on encoder click
    if (encoder.click())
    {
      dataRateIdx = (dataRateIdx + 1) % 3;
      config.dataRateIdx = dataRateIdx;
      applyDataRate();
      markConfigEdited();
      DBG("dataRateIdx: %u\n", dataRateIdx);
    }
  }

  // Stop transmitting when PTT released
  if (PTT.release() && isTx)
  {
    rfAudio.receive();
    isTx = false;
    DBG("Receiving...\n");
  }

  saveSettings();
}
