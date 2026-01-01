#include <RF24.h>
#include <RF24Audio.h>
#include "EncButton.h"
#include <TimerFreeTone.h>
#include <EEPROM.h>
#include <Vcc.h>
#include "userConfig.h"

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

#define NAME "eHand"
#define VERSION "2.2.0"

// PINS
#define LED_PIN 6
#define PTT_BUTTON_PIN 2
#define ENCODER_A_PIN 4
#define ENCODER_B_PIN 3
#define ENCODER_SWITCH_PIN 5
#define CSN_PIN 8
#define CE_PIN 7

// CONSTANTS
#define LOW_BATT_VOLT 3600
#define MID_BATT_VOLT 3900
#define BATT_SAMPLE_MS 20000
#define LED_BLINK_MS 10000
#define SAVE_DELAY_MS 10000

// Beacon
#define BEACON_ON_MS 250
#define BEACON_TONE 500
#define BEACON_PERIOD_MAX 5000
uint16_t BEACON_PERIOD = 250;

// SOS
#define SOS_FREQ 800
#define SOS_TIME_UNIT 100

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
    uint8_t rogerEnabled;
};

Settings config;
bool isConfigEdited = false;
uint32_t configLastChangeTime = 0;

bool prevStreaming = false;
bool rogerLock = false;
bool rogerEnabled = true;

bool isTx = false;
bool pttLocked = false;
bool isInvisibleMode = false;
uint8_t channelIdx = 0;
uint8_t channel = channels[channelIdx];
uint8_t volume = 4;
uint8_t maxVolume = 7;
uint8_t dataRateIdx = 1;
uint8_t txPowerIdx = 2;
uint8_t receivedPacket[BUFFER_SIZE];

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

    uint16_t on_ms = 200;
    uint16_t off_ms = 200;
    uint16_t pre_ms = 0;
    uint16_t post_ms = 0;
    bool resume_high = false;

    BlinkState state = BL_IDLE;
    uint8_t remaining = 0;
    uint32_t tmark = 0;
    bool use_pre = false;

    bool enabled = true;

    uint8_t start_count = 0;
    bool repeat = false;

    void begin()
    {
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, LOW);
        state = BL_IDLE;
        remaining = 0;
        tmark = 0;
        use_pre = false;
        start_count = 0;
        repeat = false;
    }

    void setEnabled(bool en)
    {
        if (enabled == en)
            return;

        enabled = en;

        if (!enabled)
        {
            digitalWrite(LED_PIN, LOW);
            state = BL_IDLE;
            remaining = 0;
            use_pre = false;
            repeat = false;
            start_count = 0;
        }
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

    void startEx(uint8_t count, uint16_t on, uint16_t off, uint16_t pre, uint16_t post, bool resumeHigh, bool rep = false)
    {
        if (!enabled)
        {
            return;
        }

        on_ms = on;
        off_ms = off;
        pre_ms = pre;
        post_ms = post;
        resume_high = resumeHigh;

        repeat = rep;
        start_count = count;
        remaining = count;
        state = BL_OFF;
        use_pre = (pre_ms > 0);

        tmark = millis();
    }

    void stop()
    {
        digitalWrite(LED_PIN, LOW);
        state = BL_IDLE;
        remaining = 0;
        use_pre = false;
        repeat = false;
        start_count = 0;
    }

    bool active() { return state != BL_IDLE; }

    void tick()
    {
        if (!enabled)
        {
            if (state != BL_IDLE)
            {
                digitalWrite(LED_PIN, LOW);
                state = BL_IDLE;
                remaining = 0;
                use_pre = false;
            }
            return;
        }

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
                digitalWrite(LED_PIN, HIGH);
                tmark = now;
                state = BL_ON;
                use_pre = false;
            }
        }
        break;

        case BL_ON:
            if ((now - tmark) >= on_ms)
            {
                digitalWrite(LED_PIN, LOW);
                tmark = now;

                if (remaining > 0)
                    remaining--;

                if (remaining == 0)
                {
                    if (repeat)
                    {
                        remaining = start_count; 
                        state = BL_OFF;
                        use_pre = (pre_ms > 0);
                        tmark = now;
                    }
                    else
                    {
                        state = (post_ms > 0) ? BL_POST : BL_IDLE;
                        if (state == BL_IDLE && resume_high)
                            digitalWrite(LED_PIN, HIGH);
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
                    digitalWrite(LED_PIN, HIGH);
                else
                    digitalWrite(LED_PIN, LOW);
                state = BL_IDLE;
            }
            break;

        default:
            break;
        }
    }
} blinker;

bool isLowBattery(uint32_t now)
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
              (config.txPowerIdx <= 3) &&
              (config.rogerEnabled <= 1);

    if (!ok)
    {
        config.volume = volume;
        config.dataRateIdx = dataRateIdx;
        config.channel = channel;
        config.txPowerIdx = txPowerIdx;
        config.rogerEnabled = rogerEnabled ? 1 : 0;
        EEPROM.put(0, config);
    }

    volume = config.volume;
    dataRateIdx = config.dataRateIdx;
    channel = config.channel;
    txPowerIdx = config.txPowerIdx;
    rogerEnabled = (config.rogerEnabled != 0);

    applyChannel();
    applyDataRate();
    applyTxPower();
    applyVolume();
}

void saveSettings()
{
    if (!isInvisibleMode && isConfigEdited && (millis() - configLastChangeTime >= SAVE_DELAY_MS))
    {
        EEPROM.put(0, config);
        isConfigEdited = false;
        DBG("Settings saved to EEPROM\n");
    }
}

void playRogerBeep()
{
    TimerFreeTone(9, 1123, 80, 1);
    delay(10);
    TimerFreeTone(9, 865, 80, 1);
}

struct Beacon
{
    enum Phase : uint8_t
    {
        IDLE,
        BURST,
        GAP
    } phase = IDLE;
    bool enabled = false;
    uint32_t t0 = 0;

    void startBurst()
    {
        RF24Audio_setBeaconTone(BEACON_TONE);
        RF24Audio_setBeaconMode(true);
        rfAudio.transmit();
        isTx = true;
        phase = BURST;
        t0 = millis();
        blinker.startEx(1, BEACON_ON_MS, BEACON_PERIOD, 0, 0, false);
        DBG("Sending beacon...\n");
    }

    void stopBurst()
    {
        rfAudio.receive();
        RF24Audio_setBeaconMode(false);
        isTx = false;
        phase = GAP;
        t0 = millis();
    }

    void enter()
    {
        enabled = true;
        startBurst();
        blinker.startEx(4, 50, 50, 0, 0, false);
        DBG("Started beacon mode\n");
    }

    void exit()
    {
        rfAudio.receive();
        RF24Audio_setBeaconMode(false);
        enabled = false;
        isTx = false;
        blinker.stop();
        phase = IDLE;
        DBG("Exiting beacon mode\n");
    }

    void tick()
    {
        uint32_t dt = millis() - t0;

        if (phase == BURST && dt >= BEACON_ON_MS)
        {
            stopBurst();
        }
        else if (phase == GAP && dt >= BEACON_PERIOD)
        {
            startBurst();
        }
    }
} beacon;

enum ParrotState
{
    P_IDLE,
    P_MEASURE,
    P_COOLDOWN,
    P_PLAY
};

struct Parrot
{
    uint32_t minDur = 50;
    uint32_t cooldown = 500;

    bool enabled = false;
    ParrotState state = P_IDLE;
    uint32_t t_on;
    uint32_t t_play;
    uint32_t t_dur;

    void toggle()
    {
        enabled = !enabled;
        state = P_IDLE;
        rfAudio.receive();
        blinker.startEx(5, 50, 50, 0, 0, false);
        DBG("Parrot %s\n", enabled ? "ON" : "OFF");
    }

    void tick()
    {
        uint32_t now = millis();

        switch (state)
        {
        case P_IDLE:
            if (rfAudio.isStreaming())
            {
                t_on = now;
                state = P_MEASURE;
                digitalWrite(LED_PIN, HIGH);
                DBG("[MEASURE] RX start: %lu ms\n", now);
            }
            break;

        case P_MEASURE:
            if (!rfAudio.isStreaming())
            {
                uint32_t dur = now - t_on;
                DBG("[MEASURE] RX end: %lu ms, dur=%lu ms\n", now, dur);
                if (dur >= minDur)
                {
                    t_dur = dur;
                    t_on = now + cooldown;
                    state = P_COOLDOWN;
                    DBG("[COOLDOWN] wait=%lu ms, TX dur=%lu ms\n", cooldown, t_dur);
                }
                else
                {
                    state = P_IDLE;
                    DBG("[MEASURE] ignored (short)\n");
                }

                digitalWrite(LED_PIN, LOW);
            }
            break;

        case P_COOLDOWN:
            if (now >= t_on)
            {
                rfAudio.transmit();
                t_play = now + t_dur;
                state = P_PLAY;
                digitalWrite(LED_PIN, HIGH);
                DBG("[PLAY] TX start: %lu ms, dur=%lu ms\n", now, t_dur);
            }
            break;

        case P_PLAY:
            if (now >= t_play)
            {
                rfAudio.receive();
                state = P_IDLE;
                digitalWrite(LED_PIN, LOW);
                DBG("[PLAY] TX end: %lu ms\n", now);
            }
            break;
        }
    }
} parrot;

struct Scanner
{
    bool enabled = false;
    uint32_t t_last = 0;
    uint8_t chIdx = 0;
    uint8_t rateIdx = 0;
    uint32_t dwell_ms = 500;
    bool found = false;

    void enter()
    {
        enabled = true;
        chIdx = 0;
        rateIdx = 0;
        found = false;
        applyChannel();
        applyDataRate();
        blinker.startEx(1, 60, 60, 0, 0, false, true);
        DBG("Started scanning mode\n");
    }

    void exit()
    {
        enabled = false;
        found = false;
        blinker.stop();
        DBG("Exited scanning mode\n");
    }

    void tick()
    {
        if (!enabled)
            return;
        uint32_t now = millis();

        if (rfAudio.isStreaming())
        {
            if (millis() - t_last > 500)
            {
                found = true;
                blinker.stop();
                DBG("Stable signal detected! Channel %u, RateIdx %u\n", channel, dataRateIdx);
                exit();
                blinker.startEx(2, 100, 100, 500, 0, true);
            }
            return;
        }

        if (now - t_last >= dwell_ms && !found)
        {
            rateIdx++;
            if (rateIdx >= 3)
            {
                rateIdx = 0;
                chIdx++;
                if (chIdx >= CHANNEL_COUNT)
                    chIdx = 0;
            }
            channelIdx = chIdx;
            dataRateIdx = rateIdx;
            channel = channels[chIdx];
            applyChannel();
            applyDataRate();
            t_last = now;
            DBG("Scanning: channel=%u, rateIdx=%u\n", channel, dataRateIdx);
        }
    }
} scanner;

struct SOSPlayer
{
    bool active;
    uint8_t step = 0;
    uint32_t tNext = 0;

    void start()
    {
        active = true;
        step = 0;
        tNext = 0;
    }

    void tick(uint32_t now)
    {
        if (!active)
            return;
        if (tNext != 0 && now < tNext)
            return;

        switch (step)
        {
        case 0:
            TimerFreeTone(9, SOS_FREQ, SOS_TIME_UNIT, 1);
            tNext = now + SOS_TIME_UNIT + 3 * SOS_TIME_UNIT;
            step = 1;
            break;

        case 1:
            TimerFreeTone(9, SOS_FREQ, 3 * SOS_TIME_UNIT, 1);
            tNext = now + SOS_TIME_UNIT + 3 * SOS_TIME_UNIT;
            step = 2;
            break;

        case 2:
            TimerFreeTone(9, SOS_FREQ, SOS_TIME_UNIT, 1);
            tNext = now + SOS_TIME_UNIT;
            step = 3;
            break;

        default:
            active = false;
            break;
        }
    }
} SOS;