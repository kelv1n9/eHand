/**
 * @file RF24Audio.cpp
 *
 * class & function definitions for RF24Audio library
 */

#include <Arduino.h>
#include <stddef.h>
#include "RF24Audio.h"
#include "RF24.h"
#include <userConfig.h>

//******* General Variables ************************
#define RESOLUTION_BASE ((F_CPU) / 10)
volatile boolean buffEmpty[2] = {true, true};
volatile boolean whichBuff = false;
volatile boolean streaming = false;
volatile byte prescaleByte = 0;
volatile byte buffCount = 0;
volatile byte pauseCntr = 0;
volatile byte bufCtr = 0;
byte pin = ANALOG_PIN;
unsigned int intCount = 0;
byte buffer[2][buffSize + 1];
char volMod = -1;

RF24 radi(0, 0);

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || (__AVR_ATmega32U4__) || (__AVR_AT90USB646__) || defined(__AVR_AT90USB1286__) || (__AVR_ATmega128__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
#define rampMega
#endif

/*****************************************************************************************************************************/
/************************************************* General Section ***********************************************************/
bool RF24Audio::isStreaming()
{
    return streaming;
}

RF24Audio::RF24Audio(RF24 &_radio) : radio(_radio)
{
    radi = radio;
}

void RF24Audio::begin()
{
    pinMode(speakerPin, OUTPUT);

    // 32 MHz / 128 / 13 = 19.5kHz
    if (SAMPLE_RATE < 17800)
    {
        prescaleByte = B00000110; // 64
    }
    else if (SAMPLE_RATE < 36000)
    {
        prescaleByte = B00000101; // 32
    }
    else if (SAMPLE_RATE < 54000)
    {
        prescaleByte = B00000100; // 16
    }
    else if (SAMPLE_RATE < 130000)
    {
        prescaleByte = B00000011; // 8
    }
    else
    {
        prescaleByte = B00000010; // 4
    }

    if (pin >= 14)
        pin -= 14; // allow for channel or pin numbers

    radio.setAutoAck(0);                // Disable ACKnowledgement packets
    radio.setCRCLength(RF24_CRC_8);     // Set CRC to 1 byte for speed
    radio.openWritingPipe(pipes[0]);    // Set up reading and writing pipes. All of the radios write via multicast on the same pipe
    radio.openReadingPipe(1, pipes[1]); // All of the radios listen by default to the same multicast pipe
    radio.setRetries(0, 0);

    radio.startListening(); // NEED to start listening after opening a reading pipe
    timerStart();           // Get the timer running
    RX();                   // Start listening for transmissions
}

void RF24Audio::setVolume(char vol)
{
    volMod = vol - 4;
}

void RF24Audio::timerStart()
{
    ICR1 = 10 * (RESOLUTION_BASE / SAMPLE_RATE);  // Timer will count up to this value from 0;
    TCCR1A = _BV(COM1A1);                         // Enable the timer port/pin as output
    TCCR1A |= _BV(WGM11);                         // WGM11,12,13 all set to 1 = fast PWM/w ICR TOP
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // CS10 = no prescaling
}

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

void rampUp(byte nextVal)
{
    unsigned int mod;
    if (volMod > 0)
    {
        mod = OCR1A >> volMod;
    }
    else
    {
        mod = OCR1A << (volMod * -1);
    }
    if (nextVal > mod)
    {
        for (unsigned int i = 0; i < buffSize; i++)
        {
            mod = constrain(mod + 1, mod, nextVal);
            buffer[0][i] = mod;
        }
        for (unsigned int i = 0; i < buffSize; i++)
        {
            mod = constrain(mod + 1, mod, nextVal);
            buffer[1][i] = mod;
        }
    }
    else
    {
        for (unsigned int i = 0; i < buffSize; i++)
        {
            mod = constrain(mod - 1, nextVal, mod);
            buffer[0][i] = mod;
        }
        for (unsigned int i = 0; i < buffSize; i++)
        {
            mod = constrain(mod - 1, nextVal, mod);
            buffer[1][i] = mod;
        }
    }
    whichBuff = 0;
    buffEmpty[0] = 0;
    buffEmpty[1] = 0;
    buffCount = 0;
}

void RF24Audio::transmit()
{
    TX();
}

void RF24Audio::receive()
{
    RX();
}

/*****************************************************************************************************************************/
/****************************************** Reception (RX) Section ***********************************************************/

void handleRadio()
{
    if (buffEmpty[!whichBuff] && streaming) // If in RX mode and a buffer is empty, load it
    {
        if (radi.available())
        {
            boolean n = !whichBuff;    // Grab the changing value of which buffer is not being read before enabling nested interrupts
            TIMSK1 &= ~_BV(ICIE1);     // Disable this interrupt so it is not triggered while still running (this may take a while)
            sei();                     // Enable nested interrupts (Other interrupts can interrupt this one)
            radi.read(&buffer[n], 32); // Read the payload from the radio
            buffEmpty[n] = 0;          // Indicate that a buffer is now full and ready to play
            pauseCntr = 0;             // For disabling speaker when no data is being received

            TIMSK1 |= _BV(ICIE1); // Finished, re-enable the interrupt vector that runs this function
        }
        else
        {
            pauseCntr++; // No payload available, keep track of how many for disabling the speaker
        }

        if (pauseCntr > 10)
        {                            // If we failed to get a payload 250 times, disable the speaker output
            pauseCntr = 0;           // Reset the failure counter
            rampDown();              // Ramp down the speaker (prevention of popping sounds)
            streaming = 0;           // Indicate that streaming is stopped
            TIMSK1 &= ~(_BV(TOIE1)); // Disable the TIMER1 overflow vector (playback)
#if defined(ENABLE_LED)
            digitalWrite(ledPin, LOW);
#endif
            TCCR1A &= ~_BV(COM1A1); // Disable speaker output
        }
    }
    else if (!streaming) // If not actively reading a stream, read commands instead
    {
        if (radi.available())
        {                              // If a payload is available
            TIMSK1 &= ~_BV(ICIE1);     // Since this is called from an interrupt, disable it
            sei();                     // Enable global interrupts (nested interrupts) Other interrupts can interrupt this one
            radi.read(&buffer[0], 32); // Read the payload into the buffer
            switch (buffer[0][0])
            { // Additional commands can be added here for controlling other things via radio command
              //             case 'r':
              //                 if (buffer[0][1] == 'R' && radioIdentifier < 2)
              //                 { // Switch to TX mode if we received the remote tx command and this is radio 0 or 1
              //                     TX();
              //                 }
              //                 break;
            default:
                streaming = 1;         // If not a command, treat as audio data, enable streaming
                TCCR1A |= _BV(COM1A1); // Enable output to speaker pin
                rampUp(buffer[0][31]); // Ramp up the speakers to prevent popping
                TIMSK1 |= _BV(TOIE1);  // Enable the overflow vector
#if defined(ENABLE_LED)
                digitalWrite(ledPin, HIGH);
#endif
                break;
            }
            TIMSK1 |= _BV(ICIE1); // Finished: Re-enable the interrupt that runs this function.
        }
    }
}

void RX()
{
    // Start Receiving
    TIMSK1 &= ~(_BV(OCIE1B) | _BV(OCIE1A)); // Disable the transmit interrupts
    ADCSRA = 0;
    ADCSRB = 0; // Disable Analog to Digital Converter (ADC)
    buffEmpty[0] = 1;
    buffEmpty[1] = 1;                            // Set the buffers to empty
    ICR1 = 10 * (RESOLUTION_BASE / SAMPLE_RATE); // Timer running at normal sample rate speed

    radi.openWritingPipe(pipes[0]); // Set up reading and writing pipes
    radi.openReadingPipe(1, pipes[1]);
    radi.startListening(); // Exit sending mode
    TIMSK1 = _BV(ICIE1);   // Enable the capture interrupt vector (handles buffering and starting of playback)
}

ISR(TIMER1_CAPT_vect)
{
    // This interrupt checks for data at 1/16th the sample rate. Since there are 32 bytes per payload, it gets two chances for every payload
    bufCtr++;

    if (bufCtr >= 16)
    {                  // Every 16 times, do this
        handleRadio(); // Check for incoming radio data if not transmitting
        bufCtr = 0;    // Reset this counter
    }
}

// Receiving interrupt
ISR(TIMER1_OVF_vect)
{
    // This interrupt vector loads received audio samples into the timer register
    if (buffEmpty[whichBuff])
    { // Return if both buffers are empty
        whichBuff = !whichBuff;
    }
    else
    {
        /*************** Standard 8-Bit Audio Playback ********************/
        if (volMod < 0)
        {                                                         // Load an audio sample into the timer compare register
            OCR1A = (buffer[whichBuff][intCount] >> volMod * -1); // Output to speaker at a set volume
        }
        else
        {
            OCR1A = buffer[whichBuff][intCount] << volMod;
        }

        intCount++; // Keep track of how many samples have been loaded

        if (intCount >= buffSize)
        {                                // If the buffer is empty, do the following
            intCount = 0;                // Reset the sample count
            buffEmpty[whichBuff] = true; // Indicate which buffer is empty
            whichBuff = !whichBuff;      // Switch buffers to read from
        }
    }
}

/*****************************************************************************************************************************/
/*************************************** Transmission (TX) Section ***********************************************************/

#if !defined(RX_ONLY) // If TX is enabled:

// Transmission sending interrupt
ISR(TIMER1_COMPA_vect)
{ // This interrupt vector sends the samples when a buffer is filled
    if (buffEmpty[!whichBuff] == 0)
    {                                           // If a buffer is ready to be sent
        boolean a = !whichBuff;                 // Get the buffer # before allowing nested interrupts
        TIMSK1 &= ~(_BV(OCIE1A));               // Disable this interrupt vector
        sei();                                  // Enable global interrupts
        radi.startFastWrite(&buffer[a], 32, 1); // Do an IRQ friendly radio write
        cli();                                  // Disable global interrupts
        buffEmpty[a] = 1;                       // Mark the buffer as empty
        TIMSK1 |= _BV(OCIE1A);
    }
}

// Transmission buffering interrupt
ISR(TIMER1_COMPB_vect) // This interrupt vector captures the ADC values and stores them in a buffer
{
    // 8-bit samples
    buffer[whichBuff][buffCount] = ADCH; // Read the high byte of the ADC register into the buffer for 8-bit samples
    buffCount++;                         // Keep track of how many samples have been loaded

    if (buffCount >= 32) // In 8-bit mode, do this every 32 samples
    {
        // Both modes
        buffCount = 0;             // Reset the sample counter
        buffEmpty[!whichBuff] = 0; // Indicate which bufffer is ready to send
        whichBuff = !whichBuff;    // Switch buffers and load the other one
    }
}

void TX()
{
    TIMSK1 &= ~(_BV(ICIE1) | _BV(TOIE1)); // Disable the receive interrupts
#if defined(ENABLE_LED)
    digitalWrite(ledPin, LOW);
#endif
    radi.stopListening();           // Enter transmit mode on the radio
    radi.openWritingPipe(pipes[1]); // Set up reading and writing pipes
    radi.openReadingPipe(1, pipes[0]);

    streaming = 0;
    buffCount = 0;
    buffEmpty[0] = 1;
    buffEmpty[1] = 1; // Set some variables

#if defined(ADCSRB) && defined(MUX5)
    ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif
#if defined(ADMUX)
    ADMUX = (pin & 0x07) | _BV(REFS0); // Enable the ADC PIN and set 5v Analog Reference
#endif

    ICR1 = 10 * (RESOLUTION_BASE / SAMPLE_RATE);    // Timer counts from 0 to this value
    rampDown();                                     // If disabling/enabling the speaker, ramp it down
    TCCR1A &= ~(_BV(COM1A1));                       // Disable output to speaker
    ADMUX |= _BV(ADLAR);                            // Left-shift result so only high byte needs to be read
    ADCSRB |= _BV(ADTS0) | _BV(ADTS0) | _BV(ADTS2); // Attach ADC start to TIMER1 Capture interrupt flag
    ADCSRA = prescaleByte;                          // Adjust sampling rate of ADC depending on sample rate
    ADCSRA |= _BV(ADEN) | _BV(ADATE);               // ADC Enable, Auto-trigger enable
    TIMSK1 = _BV(OCIE1B) | _BV(OCIE1A);             // Enable the TIMER1 COMPA and COMPB interrupts
}

#endif