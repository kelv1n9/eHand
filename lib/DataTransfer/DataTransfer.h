#include "RF24.h"
#include <userConfig.h>

#define CSN_PIN 8
#define CE_PIN 7

RF24 radio(CE_PIN, CSN_PIN);

//******* General Variables ************************
// Beacon tone generator state
volatile uint16_t phase = 0;         // Phase accumulator for square-wave beacon tone generation
volatile uint16_t phaseStep = 1;     // Phase increment per sample; controls beacon tone frequency
volatile boolean beaconMode = false; // If true, TX sends generated beacon tone instead of ADC audio samples

// RX/TX isStreaming state
volatile boolean isStreaming = false; // True when audio stream is active (RX playback running); false when idle/command mode

// ADC configuration for TX sampling
byte pin = ANALOG_PIN; // Selected ADC channel index (mapped from Arduino analog pin number)

// RX stream watchdog + scheduling
volatile byte pauseCntr = 0; // Counts consecutive "no payload" checks; if too high, stop isStreaming and mute output
volatile byte bufCtr = 0;    // Divider counter for TIMER1_CAPT ISR; calls handleRadio() every N ticks

// Message/command passing
volatile bool msgReady = false; // Set true when a PROTOCOL_HEADER packet was received and copied into msgRxBuf
volatile bool msgPending = false;
volatile uint8_t msgRxBuf[BUFFER_SIZE]; // Last received command/message payload
uint8_t msgTxBuf[BUFFER_SIZE];

// Double buffer for audio samples (32 bytes per payload)
// intCount/buffCount are indices inside the current buffer, whichBuff selects which of the two buffers is active.
volatile byte intCount = 0;                   // RX playback sample index within buffer[whichBuff] (0..BUFFER_SIZE-1)
volatile byte buffCount = 0;                  // TX capture sample index within buffer[whichBuff] (0..BUFFER_SIZE-1)
volatile boolean buffEmpty[2] = {true, true}; // Per-buffer state: true = empty/free, false = full/ready
volatile boolean whichBuff = false;           // Active buffer selector: 0 or 1 (toggles to implement double buffering)
byte buffer[2][BUFFER_SIZE + 1];              // Two audio payload buffers; each holds one RF payload of audio samples

// Volume control (applied during RX playback when writing to OCR1A)
volatile char volMod = -1; // Volume shift: negative = right shift (quieter), positive = left shift (louder)

/*****************************************************************************************************************************/
/************************************************* General Section ***********************************************************/
void rampDown()
{
    uint16_t step = 400;
    uint16_t target = 0;
    uint16_t cur = OCR1A;

    if (cur > target)
    {
        while (cur > target + step)
        {
            cur -= step;
            OCR1A = cur;
            delayMicroseconds(100);
        }
    }

    OCR1A = target;
}

void receive()
{
    // Start Receiving
    TIMSK1 &= ~(_BV(OCIE1B) | _BV(OCIE1A)); // Disable the transmit interrupts
    ADCSRA = 0;                             // Disable Analog to Digital Converter (ADC)
    ADCSRB = 0;                             // Disable Analog to Digital Converter (ADC)
    buffEmpty[0] = 1;                       // Set the buffers to empty
    buffEmpty[1] = 1;                       // Set the buffers to empty
    ICR1 = F_CPU / SAMPLE_RATE;             // Timer running at normal sample rate speed
    radio.openWritingPipe(pipes[0]);        // Set up writing pipes
    radio.openReadingPipe(1, pipes[1]);     // Set up writing pipes
    radio.startListening();                 // Exit sending mode
    TIMSK1 = _BV(ICIE1);                    // Enable the capture interrupt vector (handles buffering and starting of
}

void transmit()
{
    cli();
    TIMSK1 &= ~((1 << ICIE1) | (1 << TOIE1)); // Disable the receive interrupts
    radio.stopListening();                    // Enter transmit mode on the radio
    radio.openWritingPipe(pipes[1]);          // Set up reading and writing pipes
    radio.openReadingPipe(1, pipes[0]);

    buffCount = 0;
    isStreaming = false;
    buffEmpty[0] = true;
    buffEmpty[1] = true; // Set some variables

    pinMode(SPEAKER_PIN, INPUT);            // Disable the speaker pin
    ADMUX = (pin & 0x07) | (1 << REFS0);    // Enable the ADC PIN and set 5v Analog Reference
    ICR1 = F_CPU / SAMPLE_RATE - 1;         // Timer counts from 0 to this value
    TCCR1A &= ~((1 << COM1A1));             // Disable output to speaker
    ADMUX |= (1 << ADLAR);                  // Left-shift result so only high byte needs to be read
    ADCSRB |= (1 << ADTS0) | (1 << ADTS2);  // Attach ADC start to TIMER1 Capture interrupt flag
    ADCSRA = 1;                             // Adjust sampling rate of ADC depending on sample rate
    ADCSRA |= (1 << ADEN) | (1 << ADATE);   // ADC Enable, Auto-trigger enable
    TIMSK1 = (1 << OCIE1B) | (1 << OCIE1A); // Enable the TIMER1 COMPA and COMPB interrupts
}

void setBeaconMode(bool enabled)
{
    cli();
    beaconMode = enabled;
    phase = 0;
    sei();
}

void setBeaconTone(uint16_t hz)
{
    uint32_t step = (((uint32_t)hz << 16) + (SAMPLE_RATE / 2)) / (uint32_t)SAMPLE_RATE;
    if (step == 0)
        step = 1;
    cli();
    phaseStep = step;
    sei();
}

void sendMessage(const uint8_t *packet)
{
    cli();
    TIMSK1 &= ~((1 << ICIE1) | (1 << TOIE1));
    radio.stopListening();
    radio.openWritingPipe(pipes[1]);
    radio.flush_tx();

    radio.write(packet, BUFFER_SIZE, 1);

    receive();
}

bool readMessage(uint8_t *out)
{
    bool ready = false;

    cli();
    if (msgReady)
    {
        for (uint8_t i = 0; i < BUFFER_SIZE; i++)
            out[i] = msgRxBuf[i];
        msgReady = false;
        ready = true;
    }
    sei();

    return ready;
}

void audioBegin()
{
    pin = (ANALOG_PIN >= 14) ? (ANALOG_PIN - 14) : ANALOG_PIN;

    radio.begin();
    radio.setAutoAck(0);                   // Disable ACKnowledgement packets
    radio.setCRCLength(RF24_CRC_DISABLED); // Set CRC to 1 byte for speed
    // radio.openWritingPipe(pipes[0]);       // Set up reading and writing pipes. All of the radios write via multicast on the same pipe
    // radio.openReadingPipe(1, pipes[1]);    // All of the radios listen by default to the same multicast pipe
    radio.openWritingPipe(pipes[1]); // Set up reading and writing pipes
    radio.openReadingPipe(1, pipes[0]);
    radio.setRetries(0, 0); // Set the number of retry attempts and delay between retry attempts
    radio.startListening(); // NEED to start listening after opening a reading pipe

    TCCR1A = (1 << COM1A1) | (1 << WGM11);              // Enable the timer port/pin as output
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // FPWM, 585.93035 Hz
    ICR1 = F_CPU / SAMPLE_RATE - 1;                     // Timer will count up to this value from 0;

    receive(); // Start listening for transmissions
}

void setVolume(char vol)
{
    volMod = vol - 4;
}

/*****************************************************************************************************************************/
/****************************************** Reception (RX) Section ***********************************************************/

void handleRadio()
{
    if (buffEmpty[!whichBuff] && isStreaming) // If in RX mode and a buffer is empty, load it
    {
        if (radio.available())
        {
            boolean notWhichBuff = !whichBuff;              // Grab the changing value of which buffer is not being read before enabling nested interrupts
            TIMSK1 &= ~(1 << ICIE1);                        // Disable this interrupt so it is not triggered while still running (this may take a while)
            sei();                                          // Enable nested interrupts (other interrupts can interrupt this one)
            radio.read(&buffer[notWhichBuff], BUFFER_SIZE); // Read the payload from the radio
            TIMSK1 |= (1 << ICIE1);                         // Finished, re-enable the interrupt vector that runs this function
            buffEmpty[notWhichBuff] = false;                // Indicate that a buffer is now full and ready to play
            pauseCntr = 0;                                  // For disabling speaker when no data is being received
        }
        else
        {
            pauseCntr++; // No payload available, keep track of how many for disabling the speaker
        }

        if (pauseCntr > 20)
        {                        // If we failed to get a payload 20 times, disable the speaker output
            pauseCntr = 0;       // Reset the failure counter
            isStreaming = false; // Indicate that isStreaming is stopped

            digitalWrite(LED_PIN, LOW); // Disable LED indicator

            TIMSK1 &= ~((1 << TOIE1)); // Disable the TIMER1 overflow vector (playback)
            TCCR1A &= ~(1 << COM1A1);  // Disable speaker output
        }
    }
    else if (!isStreaming) // If not actively reading a stream, read commands instead
    {
        if (radio.available())
        {                                        // If a payload is available
            TIMSK1 &= ~(1 << ICIE1);             // Since this is called from an interrupt, disable it
            sei();                               // Enable global interrupts (nested interrupts) Other interrupts can interrupt this one
            radio.read(&buffer[0], BUFFER_SIZE); // Read the payload into the buffer

            // Additional commands can be added here for controlling other things via radio command
            if (buffer[0][0] == PROTOCOL_HEADER)
            {
                for (uint8_t i = 0; i < BUFFER_SIZE; i++)
                    msgRxBuf[i] = buffer[0][i];
                msgReady = true;
            }
            else
            {
                isStreaming = true;           // If not a command, treat as audio data, enable isStreaming
                digitalWrite(LED_PIN, HIGH);  // Enable LED indicator
                pinMode(SPEAKER_PIN, OUTPUT); // Enable the speaker pin
                TCCR1A |= (1 << COM1A1);      // Enable output to speaker pin
                TIMSK1 |= (1 << TOIE1);       // Enable the overflow vector
            }

            TIMSK1 |= (1 << ICIE1); // Finished: Re-enable the interrupt that runs this function.
        }
    }
}

ISR(TIMER1_CAPT_vect)
{
    // This interrupt checks for data
    bufCtr++;

    if (bufCtr >= 8) // Every N times, do this
    {
        handleRadio(); // Check for incoming radio data if not transmitting
        bufCtr = 0;    // Reset this counter
    }
}

// Receiving interrupt
ISR(TIMER1_OVF_vect)
{
    // This interrupt vector loads received audio samples into the timer register
    if (buffEmpty[whichBuff]) // Return if both buffers are empty
    {
        whichBuff = !whichBuff;
    }
    else
    {
        /*************** Standard 8-Bit Audio Playback ********************/
        // Load an audio sample into the timer compare register
        if (volMod < 0)
        {
            OCR1A = (buffer[whichBuff][intCount] >> volMod * -1); // Output to speaker at a set volume
        }
        else
        {
            OCR1A = buffer[whichBuff][intCount] << volMod;
        }

        intCount++; // Keep track of how many samples have been loaded

        if (intCount >= BUFFER_SIZE)
        {                                // If the buffer is empty, do the following
            intCount = 0;                // Reset the sample count
            buffEmpty[whichBuff] = true; // Indicate which buffer is empty
            whichBuff = !whichBuff;      // Switch buffers to read from
        }
    }
}

/*****************************************************************************************************************************/
/*************************************** Transmission (TX) Section ***********************************************************/

// Transmission sending interrupt
// This interrupt vector sends the samples when a buffer is filled
ISR(TIMER1_COMPA_vect)
{
    if (!buffEmpty[!whichBuff])
    {                                                                // If a buffer is ready to be sent
        boolean notWhichBuff = !whichBuff;                           // Get the buffer # before allowing nested interrupts
        TIMSK1 &= ~(1 << OCIE1A);                                    // Disable this interrupt vector
        sei();                                                       // Enable global interrupts
        radio.startFastWrite(&buffer[notWhichBuff], BUFFER_SIZE, 1); // Do an IRQ friendly radio write
        cli();                                                       // Disable global interrupts
        buffEmpty[notWhichBuff] = 1;                                 // Mark the buffer as empty
        TIMSK1 |= (1 << OCIE1A);
    }
}

// Transmission buffering interrupt
// This interrupt vector captures the ADC values and stores them in a buffer
ISR(TIMER1_COMPB_vect)
{
    // 8-bit samples
    if (beaconMode)
    {
        phase += phaseStep;
        buffer[whichBuff][buffCount] = (phase & 0x8000) ? 255 : 0;
    }
    else
    {
        buffer[whichBuff][buffCount] = ADCH;
    }
    buffCount++;

    // In 8-bit mode, do this every 32 samples
    if (buffCount >= BUFFER_SIZE)
    {
        // Both modes
        buffCount = 0;                 // Reset the sample counter
        buffEmpty[!whichBuff] = false; // Indicate which bufffer is ready to send
        whichBuff = !whichBuff;        // Switch buffers and load the other one
    }
}