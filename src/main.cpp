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
  DBG("Version: %s\n", VERSION);
#endif
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  radio.begin();
  rfAudio.begin();

  loadSettings();

  DBG("Channel: %u, Volume: %u, dataRateIdx: %u, TxPowerIdx: %u\n", channel, volume, dataRateIdx, txPowerIdx);

  isTx = false;
}

void loop()
{
  static uint32_t blinkTimer;
  uint32_t now = millis();

  encoder.tick();
  PTT.tick();

  // Power toggle when encoder turned while PTT held
  if (PTT.pressing() && encoder.turn())
  {
    if (isTx)
    {
      digitalWrite(LED_PIN, LOW);
    }

    int idx = (int)txPowerIdx + (encoder.dir() > 0 ? 1 : -1);
    if (idx < 0)
      idx = 3;
    if (idx > 3)
      idx = 0;
    txPowerIdx = (uint8_t)idx;
    config.txPowerIdx = txPowerIdx;
    applyTxPower();
    scheduleBlink(txPowerIdx + 1);
    markConfigEdited();
    DBG("TxPowerIdx: %u\n", txPowerIdx);
  }
  else if (!PTT.pressing() && encoder.turn())
  {
    // Channel toggle when encoder pressed and turned
    if (encoder.pressing())
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
      scheduleBlink(channelIdx + 1);
      markConfigEdited();
      DBG("Channel: %u\n", channel);
    }
    // Volume toggle when encoder turned
    else
    {
      int new_volume = (int)volume + encoder.dir();
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
    scheduleBlink(dataRateIdx + 1);
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
    blinkTimer = now;
  }
  // Stop Transmitting when PTT released
  if (PTT.release() && isTx)
  {
    digitalWrite(LED_PIN, LOW);
    rfAudio.receive();
    isTx = false;
    blinkTimer = now;
    DBG("Receiving...\n");
  }

  // Blink LED when radio is waiting
  if (!isTx)
  {
    if (!(TCCR0A & _BV(COM0A1)))
    {
      if (now - blinkTimer >= LED_BLINK)
      {
        float volts = vcc.Read_Volts();
#ifdef DEBUG_eHand
        Serial.print(F("Battery: "));
        Serial.print(volts, 2);
        Serial.println(F(" V"));
#endif
        if (volts <= LOW_BATT_VOLT)
        {
          digitalWrite(LED_PIN, HIGH);
          delay(120);
          digitalWrite(LED_PIN, LOW);
          delay(120);
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

        blinkTimer = now;
      }
    }
  }

  if (blinkPending && (int32_t)(now - blinkWhenMs) >= 0)
  {
    doBlink(blinkCountPending);
    blinkPending = false;
  }
  saveSettings();
}
