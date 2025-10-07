/*
Controls:

- PTT hold ............ TX mode
- PTT release ......... RX mode
- PTT hold+Enc click .. Lock mode

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
  radio.begin();
  loadSettings();
  rfAudio.begin();
  blinker.begin();

  // Invisible mode
  if (encoder.readBtn())
  {
    encoder.reset();
    isInvisibleMode = true;
    maxVolume = 3;
    volume = 1;
    dataRateIdx = 0; // MIN
    txPowerIdx = 3;  // MAX

    applyDataRate();
    applyTxPower();
    applyVolume();
    blinker.setEnabled(!isInvisibleMode);
    DBG("Invisible mode is enabled\n");
  }

  delay(300);
  uint16_t volts = vcc.Read_Volts() * 1000;
  DBG("Battery: %u mV\n", volts);

  if (volts > MID_BATT_VOLT)
    blinker.startEx(1, 50, 50, 0, 2000, false);
  else if (volts > LOW_BATT_VOLT)
    blinker.startEx(2, 50, 50, 0, 2000, false);
  else
    blinker.startEx(3, 50, 50, 0, 2000, false);
}

void loop()
{
  encoder.tick();
  PTT.tick();

  uint32_t now = millis();
  static uint32_t nextBlinkAt;

  if (beacon.enabled)
  {
    if (encoder.hold())
    {
      beacon.exit();
      return;
    }
    beacon.tick();
    blinker.tick();
    return;
  }

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
    config.txPowerIdx = txPowerIdx = (uint8_t)idx;
    applyTxPower();
    markConfigEdited();
    blinker.startEx(txPowerIdx + 1, 200, 200, 200, 200, true);
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
      config.channel = channel = channels[channelIdx];
      applyChannel();
      markConfigEdited();
      blinker.startEx(channelIdx + 1, 200, 200, 200, 200, rfAudio.isStreaming());
    }
    // Volume toggle when encoder turned
    else if (!isTx)
    {
      int new_volume = (int)volume + encoder.dir();
      if (new_volume < 0)
        new_volume = 0;
      if (new_volume > maxVolume)
        new_volume = maxVolume;

      if (new_volume != volume)
      {
        config.volume = volume = (uint8_t)new_volume;
        applyVolume();
        markConfigEdited();
        blinker.startEx(1, 20, 20, 0, 200, rfAudio.isStreaming());
      }
    }
  }

  if (PTT.pressing() && encoder.hasClicks())
  {
    pttLocked = true;
    blinker.startEx(rogerEnabled ? 1 : 2, 120, 120, 0, 0, true);
    DBG("PTT Locked\n");
  }

  if (!PTT.pressing() && !isTx && encoder.hasClicks())
  {
    uint8_t nClicks = encoder.getClicks();

    // Rate toggle on encoder click
    if (nClicks == 1)
    {
      dataRateIdx = (dataRateIdx + 1) % 3;
      config.dataRateIdx = dataRateIdx;
      applyDataRate();
      markConfigEdited();
      blinker.startEx(dataRateIdx + 1, 200, 200, 200, 200, false);
    }
    // Roger Beep toggle on encoder double click
    else if (nClicks == 2)
    {
      rogerEnabled = !rogerEnabled;
      config.rogerEnabled = rogerEnabled ? 1 : 0;
      markConfigEdited();
      blinker.startEx(rogerEnabled ? 1 : 2, 120, 120, 0, 0, false);
      DBG("Roger beep: %s\n", rogerEnabled ? "ON" : "OFF");
    }
    // Beacon mode
    else if (nClicks == 3 && !beacon.enabled)
    {
      beacon.enter();
      return;
    }
  }

  // Start Transmitting while PTT hold
  if (PTT.press())
  {
    // Transmit if not receiving
    if (!rfAudio.isStreaming() && !isTx && !pttLocked)
    {
      rfAudio.transmit();
      digitalWrite(LED_PIN, isInvisibleMode ? LOW : HIGH);
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
  if (now >= nextBlinkAt && !blinker.active() && !isTx && !rfAudio.isStreaming())
  {
    bool isLow = isLowBattery(now);
    blinker.startEx(isLow ? 2 : 1, 50, 50, 200, 0, false);
    nextBlinkAt = now + LED_BLINK_MS;
  }

  // Roger beep
  if (rogerEnabled && prevStreaming && !rfAudio.isStreaming() && !rogerLock)
  {
    rogerLock = true;
    playRogerBeep();
  }
  if (!prevStreaming && rfAudio.isStreaming())
  {
    rogerLock = false;
  }
  prevStreaming = rfAudio.isStreaming();

  blinker.tick();
  saveSettings();
}
