#pragma once

#include <RF24.h>
#include <RF24Audio.h>
#include "EncButton.h"
#include <EEPROM.h>
#include <Vcc.h>

// #define DEBUG_eHand

#ifdef DEBUG_eHand
#include "printf.h"
#define DBG(...)             \
    do                       \
    {                        \
        printf(__VA_ARGS__); \
    } while (0)
#else
#define DBG(...) \
    do           \
    {            \
    } while (0)
#endif

/*

Pin definitions
A0 - Microphone input
D9 - Speaker output
D6 - LED output
D12 - SPI MISO
D11 - SPI MOSI
D13 - SPI SCK
D7 - RF24 CE
D8 - RF24 CSN
D2 - PTT button
D3 - Encoder B
D4 - Encoder A
D5 - Encoder switch

*/

#define NAME "eHand"
#define VERSION "1.0.0"

// PINS
#define LED_PIN 6
#define PTT_BUTTON_PIN 2
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 3
#define ENCODER_SWITCH_PIN 5
#define CSN_PIN 8
#define CE_PIN 7

// CONSTANTS
#define LOW_BATT_VOLT 3800
#define BATT_SAMPLE_MS 20000
#define LED_BLINK_MS 10000
#define SAVE_DELAY_MS 10000

const uint8_t channels[] = {90, 100, 110};
#define CHANNEL_COUNT (sizeof(channels) / sizeof(channels[0]))

Vcc vcc(1);

Button PTT(PTT_BUTTON_PIN);
EncButton encoder(ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_SWITCH_PIN);

RF24 radio(CE_PIN, CSN_PIN);
RF24Audio rfAudio(radio);

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

bool isTx = false;
bool pttLocked = false;
uint8_t channelIdx = 0;
uint8_t channel = channels[channelIdx];
uint8_t volume = 4;
uint8_t dataRateIdx = 1;
uint8_t txPowerIdx = 2;

enum BlinkState : uint8_t
{
    BL_IDLE,
    BL_OFF,
    BL_ON,
    BL_POST
};

struct Blinker
{
    uint16_t default_on_ms = 200;
    uint16_t default_off_ms = 200;
    uint16_t default_pre_ms = 0;
    uint16_t default_post_ms = 0;
    bool default_resume_high = false;

    uint8_t pin = 255;
    uint16_t on_ms = 200;
    uint16_t off_ms = 200;
    uint16_t pre_ms = 0;
    uint16_t post_ms = 0;
    bool resume_high = false;

    BlinkState state = BL_IDLE;
    uint8_t remaining = 0;
    uint32_t tmark = 0;
    bool use_pre = false;

    void begin(uint8_t ledPin)
    {
        pin = ledPin;
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        state = BL_IDLE;
        remaining = 0;
        tmark = 0;
        use_pre = false;
    }

    void setDefaults(uint16_t on, uint16_t off, uint16_t pre, uint16_t post, bool resumeHigh)
    {
        default_on_ms = on;
        default_off_ms = off;
        default_pre_ms = pre;
        default_post_ms = post;
        default_resume_high = resumeHigh;
    }

    void start(uint8_t count)
    {
        startEx(count, default_on_ms, default_off_ms, default_pre_ms, default_post_ms, default_resume_high);
    }

    void startEx(uint8_t count, uint16_t on, uint16_t off, uint16_t pre, uint16_t post, bool resumeHigh)
    {
        if (pin == 255)
            return;

        on_ms = on;
        off_ms = off;
        pre_ms = pre;
        post_ms = post;
        resume_high = resumeHigh;

        remaining = count;
        state = BL_OFF;
        use_pre = (pre_ms > 0);

        tmark = millis();
    }

    void stop()
    {
        if (pin != 255)
            digitalWrite(pin, LOW);
        state = BL_IDLE;
        remaining = 0;
        use_pre = false;
    }

    bool active() const { return state != BL_IDLE; }

    void tick()
    {
        if (state == BL_IDLE)
            return;

        uint32_t now = millis();

        switch (state)
        {
        case BL_OFF:
        {
            uint16_t wait = use_pre ? pre_ms : off_ms;
            if ((now - tmark) >= wait)
            {
                digitalWrite(pin, HIGH);
                tmark = now;
                state = BL_ON;
                use_pre = false;
            }
        }
        break;

        case BL_ON:
            if ((now - tmark) >= on_ms)
            {
                digitalWrite(pin, LOW);
                tmark = now;

                if (remaining > 0)
                    remaining--;

                if (remaining == 0)
                {
                    state = (post_ms > 0) ? BL_POST : BL_IDLE;
                    if (state == BL_IDLE && resume_high)
                    {
                        digitalWrite(pin, HIGH);
                    }
                }
                else
                {
                    state = BL_OFF;
                }
            }
            break;

        case BL_POST:
            if ((now - tmark) >= post_ms)
            {
                if (resume_high)
                    digitalWrite(pin, HIGH);
                else
                    digitalWrite(pin, LOW);
                state = BL_IDLE;
            }
            break;

        default:
            break;
        }
    }
} blinker;

bool lowBattery(uint32_t now)
{
    static bool low = false;
    static uint32_t nextSampleAt = 0;

    if (now >= nextSampleAt)
    {
        uint16_t volts = vcc.Read_Volts() * 1000;
        DBG("Battery: %u mV\n", volts);
        low = (volts <= LOW_BATT_VOLT);
        nextSampleAt = now + BATT_SAMPLE_MS;
    }
    return low;
}

void applyChannel()
{
    radio.setChannel(channel);
    DBG("Channel: %u\n", channel);
}

void applyDataRate()
{
    switch (dataRateIdx)
    {
    case 0:
        radio.setDataRate(RF24_250KBPS);
        DBG("Data Rate: 250kbps\n");
        break;
    case 1:
        radio.setDataRate(RF24_1MBPS);
        DBG("Data Rate: 1Mbps\n");
        break;
    case 2:
        radio.setDataRate(RF24_2MBPS);
        DBG("Data Rate: 2Mbps\n");
        break;
    }
}

void applyTxPower()
{
    switch (txPowerIdx)
    {
    case 0:
        radio.setPALevel(RF24_PA_MIN);
        DBG("Tx Power: MIN\n");
        break;
    case 1:
        radio.setPALevel(RF24_PA_LOW);
        DBG("Tx Power: LOW\n");
        break;
    case 2:
        radio.setPALevel(RF24_PA_HIGH);
        DBG("Tx Power: HIGH\n");
        break;
    case 3:
        radio.setPALevel(RF24_PA_MAX);
        DBG("Tx Power: MAX\n");
        break;
    }
}

void applyVolume()
{
    rfAudio.setVolume(volume);
    DBG("Volume: %u\n", volume);
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

    applyChannel();
    applyDataRate();
    applyTxPower();
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