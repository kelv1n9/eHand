#include <RF24.h>
#include <RF24Audio.h>
#include "printf.h"
#include "EncButton.h"
#include <EEPROM.h>

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
  uint8_t rateIdx;
};

Settings config;
bool cfgDirty = false;
uint32_t cfgLastChange = 0;

Button PTT(PTT_BUTTON_PIN);
EncButton encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_SWITCH_PIN);

RF24 radio(CE_PIN, CSN_PIN);
RF24Audio rfAudio(radio, 0);

uint8_t channel = CHANNEL_START;
uint8_t volume = 4;
uint8_t rateIdx = 1;

void markCfgDirty()
{
  cfgDirty = true;
  cfgLastChange = millis();
}

void loadSettingsOrDefaults()
{
  EEPROM.get(0, config);
  bool ok = (config.volume <= 7) &&
            (config.rateIdx <= 2) &&
            (config.channel >= CH_MIN && config.channel <= CH_MAX);

  if (!ok)
  {
    config.volume = volume;
    config.rateIdx = rateIdx;
    config.channel = channel;
    EEPROM.put(0, config);
  }

  volume = config.volume;
  rateIdx = config.rateIdx;
  channel = config.channel;
}

void maybeSaveSettings()
{
  if (cfgDirty && (millis() - cfgLastChange >= SAVE_DELAY_MS))
  {
    EEPROM.put(0, config);
    cfgDirty = false;
  }
}

void applyRadio()
{
  radio.setChannel(channel);
  switch (rateIdx)
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

void applyVolume(uint8_t vol)
{
  rfAudio.setVolume(vol);
}

void setup()
{
  Serial.begin(115200);

  printf_begin();

  radio.begin();
  rfAudio.begin();

  loadSettingsOrDefaults();
  applyRadio();
  applyVolume(volume);

  radio.printDetails();
}

void loop()
{
  if (encoder.tick())
  {
    switch (encoder.action())
    {
    case EB_TURN:
    {
      int step = (encoder.fast() ? 2 : 1) * encoder.dir();

      if (encoder.pressing())
      {
        int c = (int)channel + step;
        if (c < CH_MIN)
          c = CH_MAX;
        if (c > CH_MAX)
          c = CH_MIN;
        channel = (uint8_t)c;
        config.channel = channel;
        radio.setChannel(channel);
        markCfgDirty();
      }
      else
      {
        int v = (int)volume + step;
        if (v < 0)
          v = 0;
        if (v > 7)
          v = 7;
        volume = (uint8_t)v;
        config.volume = volume;
        applyVolume(volume);
        markCfgDirty();
      }

      break;
    }
    case EB_CLICK:
      rateIdx = (rateIdx + 1) % 3;
      config.rateIdx = rateIdx;
      applyRadio();
      markCfgDirty();
      break;
    }
  }

  if (PTT.tick())
  {
    switch (PTT.action())
    {
    case EB_PRESS:
    {
      rfAudio.transmit();
      break;
    }
    case EB_RELEASE:
    {
      rfAudio.receive();
      break;
    }
    }
  }

  maybeSaveSettings();
}
