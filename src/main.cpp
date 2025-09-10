/*
Controls:

- PTT hold ............ TX mode
- PTT release ......... RX mode

- Encoder turn ........ Volume
- Encoder press+turn .. Channel
- Encoder click ....... Data Rate
- Enc turn+PTT hold.... TX Power

*/

#include "functions.h"

void setup()
{
#ifdef DEBUG_eHand
  Serial.begin(9600);
  printf_begin();
  delay(1000);
  DBG("Name: %s\n", NAME);
  DBG("Version: %s\n\n", VERSION);
#endif
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  radio.begin();
  rfAudio.begin();

  loadSettings();

  isTx = false;
  blinker.begin(LED_PIN);
}

void loop()
{
  encoder.tick();
  PTT.tick();

  uint32_t now = millis();
  static uint32_t nextBlinkAt;

  // Power toggle when encoder turned while PTT held
  if (PTT.pressing() && encoder.turn())
  {
    if (blinker.active())
      blinker.stop();
    int idx = (int)txPowerIdx + (encoder.dir() > 0 ? 1 : -1);
    if (idx < 0)
      idx = 3;
    if (idx > 3)
      idx = 0;
    txPowerIdx = (uint8_t)idx;
    config.txPowerIdx = txPowerIdx;
    applyTxPower();
    blinker.startEx(txPowerIdx + 1, 200, 200, 200, 200, true);
    markConfigEdited();
  }
  else if (!PTT.pressing() && encoder.turn())
  {
    // Channel toggle when encoder pressed and turned
    if (encoder.pressing() && !isTx)
    {
      int newIndex = (int)channelIdx + encoder.dir();

      if (newIndex < 0)
        newIndex = CHANNEL_COUNT - 1;
      if (newIndex >= (int)CHANNEL_COUNT)
        newIndex = 0;

      channelIdx = (uint8_t)newIndex;
      channel = channels[channelIdx];
      config.channel = channel;
      applyChannel();
      blinker.startEx(channelIdx + 1, 200, 200, 200, 200, false);
      markConfigEdited();
    }
    // Volume toggle when encoder turned
    else if (!isTx)
    {
      int new_volume = (int)volume + encoder.dir();
      if (new_volume < 0)
        new_volume = 0;
      if (new_volume > 7)
        new_volume = 7;

      if (new_volume != volume)
      {
        volume = (uint8_t)new_volume;
        config.volume = volume;
        applyVolume();
        markConfigEdited();
        blinker.startEx(1, 20, 20, 0, 200, false);
      }
    }
  }

  // Rate toggle on encoder click
  if (encoder.click())
  {
    if (PTT.pressing())
    {
      pttLocked = true;
      DBG("PTT Locked\n");
    }
    else if (!isTx)
    {
      dataRateIdx = (dataRateIdx + 1) % 3;
      config.dataRateIdx = dataRateIdx;
      applyDataRate();
      blinker.startEx(dataRateIdx + 1, 200, 200, 200, 200, false);
      markConfigEdited();
    }
  }

  // Start Transmitting while PTT hold
  if (PTT.press())
  {
    if (!isTx && !pttLocked)
    {
      rfAudio.transmit();
      digitalWrite(LED_PIN, HIGH);
      isTx = true;
      DBG("Transmitting...\n");
    }
    else if (pttLocked)
    {
      pttLocked = false;
      DBG("PTT Lock released\n");
      if (isTx)
      {
        blinker.stop();
        rfAudio.receive();
        isTx = false;
        DBG("Receiving...\n");
      }
    }
  }
  // Stop Transmitting when PTT released
  if (PTT.release() && isTx && !pttLocked)
  {
    blinker.stop();
    rfAudio.receive();
    isTx = false;
    DBG("Receiving...\n");
  }

  // Blink LED when radio is waiting
  if (now >= nextBlinkAt && !blinker.active() && !isTx)
  {
    bool isLow = lowBattery(now);
    blinker.startEx(isLow ? 2 : 1, 50, 50, 200, 0, false);
    nextBlinkAt = now + LED_BLINK_MS;
  }

  blinker.tick();
  saveSettings();
}
