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
  audioBegin();
  loadSettings();
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
    blinker.startEx(1, 100, 100, 0, 2000, false);
  else if (volts > LOW_BATT_VOLT)
    blinker.startEx(2, 100, 100, 0, 2000, false);
  else
    blinker.startEx(3, 100, 100, 0, 2000, false);
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

    if (encoder.turn())
    {
      int period = BEACON_PERIOD + BEACON_ON_MS * encoder.dir();
      if (period <= BEACON_ON_MS)
        period = BEACON_ON_MS;
      if (period >= BEACON_PERIOD_MAX)
        period = BEACON_PERIOD_MAX;

      BEACON_PERIOD = period;
    }

    beacon.tick();
    blinker.tick();
    return;
  }
  else if (parrot.enabled)
  {
    if (encoder.hold())
    {
      parrot.toggle();
      return;
    }
    parrot.tick();
    blinker.tick();
    return;
  }
  else if (scanner.enabled)
  {
    if (encoder.hold())
    {
      scanner.exit();
      return;
    }
    scanner.tick();
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
      blinker.startEx(channelIdx + 1, 200, 200, 200, 200, isStreaming);
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
        blinker.startEx(1, 20, 20, 0, 200, isStreaming);
      }
    }
  }

  if (PTT.pressing() && encoder.hasClicks())
  {
    pttLocked = true;
    DBG("PTT Locked\n");
  }

  if (PTT.hasClicks() && encoder.pressing())
  {
    uint8_t nClicks = PTT.getClicks();

    if (nClicks == 3)
    {
      DBG("Sending SOS...\n");
      uint8_t packet[BUFFER_SIZE] = {0};
      packet[0] = PROTOCOL_HEADER;
      packet[1] = 'S';
      packet[2] = 'O';
      packet[3] = 'S';
      sendMessage(packet);
      blinker.startEx(1, 200, 200, 0, 0, false);
    }
  }

  if (readMessage(receivedPacket))
  {
    if (receivedPacket[0] == PROTOCOL_HEADER)
    {
      if (receivedPacket[1] == 'S' && receivedPacket[2] == 'O' && receivedPacket[3] == 'S')
      {
        DBG("SOS Received\n");
        SOS.start();
      }
    }
  }

  if (!PTT.pressing() && !isTx && encoder.hasClicks())
  {
    uint8_t nClicks = encoder.getClicks();

    // Rate toggle
    if (nClicks == 1)
    {
      dataRateIdx = (dataRateIdx + 1) % 3;
      config.dataRateIdx = dataRateIdx;
      applyDataRate();
      markConfigEdited();
      blinker.startEx(dataRateIdx + 1, 200, 200, 200, 200, false);
    }
    // Scanner mode
    else if (nClicks == 2 && !scanner.enabled)
    {
      scanner.enter();
      return;
    }
    // Roger Beep
    else if (nClicks == 3)
    {
      rogerEnabled = !rogerEnabled;
      config.rogerEnabled = rogerEnabled ? 1 : 0;
      markConfigEdited();
      blinker.startEx(rogerEnabled ? 1 : 2, 120, 120, 0, 0, false);
      DBG("Roger beep: %s\n", rogerEnabled ? "ON" : "OFF");
    }
    // Beacon mode
    else if (nClicks == 4 && !beacon.enabled)
    {
      beacon.enter();
      return;
    }
    // Parrot mode
    else if (nClicks == 5 && !parrot.enabled)
    {
      parrot.toggle();
      return;
    }
  }

  // Start Transmitting while PTT hold
  if (PTT.press() && !encoder.pressing())
  {
    // Transmit if not receiving
    if (!isStreaming && !isTx && !pttLocked)
    {
      transmit();
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
        receive();
        isTx = false;
        DBG("Receiving...\n");
      }
    }
  }

  if (PTT.release() && isTx && !pttLocked)
  {
    releasePending = true;
    releaseAt = now;
  }

  if (releasePending && isTx && !pttLocked)
  {
    if (now - releaseAt >= PTT_RELEASE_MS && !PTT.pressing())
    {
      releasePending = false;
      blinker.stop();
      receive();
      isTx = false;
      DBG("Receiving...\n");
    }
    else if (PTT.pressing())
    {
      releasePending = false;
    }
  }

  // Blink LED when radio is waiting
  if (now >= nextBlinkAt && !blinker.active() && !isTx && !isStreaming)
  {
    bool isLow = isLowBattery(now);
    blinker.startEx(isLow ? 2 : 1, 100, 100, 200, 0, false);
    nextBlinkAt = now + LED_BLINK_MS;
  }

  // Roger beep
  if (rogerEnabled && prevStreaming && !isStreaming && !rogerLock)
  {
    rogerLock = true;
    playRogerBeep();
  }
  if (!prevStreaming && isStreaming)
  {
    rogerLock = false;
  }
  prevStreaming = isStreaming;

  blinker.tick();
  SOS.tick(now);
  saveSettings();
}
