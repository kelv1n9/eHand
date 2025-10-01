/**
 * @file userConfig.h
 *
 * macros for customization of RF24Audio
 */

/************  MANDATORY User Variables  ************/
/**
 * @brief The sample rate to use for transferring audio samples
 *
 * Maximum Range: Sample rate 16000, RF_SPEED RF24_250KBPS <br>
 * Maximum Quality: Sample rate 44000, RF_SPEED RF24_2MBPS
 *
 * - RF24_250KBPS will do 13-20khz+ sample rate
 * - RF24_1MBPS up to 24-44khz+
 * - RF24_2MBPS for higher.
 * @note 44khz+ sample rate requires 8-bits per sample
 */
#define SAMPLE_RATE 10000

/** @brief The pin that analog readings will be taken from (microphone pin) */
#define ANALOG_PIN A0

/************  OverRides  ************/

/**
 * @brief Indicator pin
 *
 * Using pin 6 on Uno enables audio visualization. Pin 13 on Mega 2560 (TIMER0 COMPA)
 * @note The pin number cannot be changed.
 */
#define ENABLE_LED

/************  Optional/Advanced User Variables  ************/

/** The size of the memory buffer to use. Not really configurable (set to maximum by default). */
#define buffSize 32

/************  Automated pin selections, override by defining above  ************/

// Speaker selection for Uno,Nano, etc
#if !defined(speakerPin)
/** The pin used to output audio on UNO */
#define speakerPin 9
#endif

#if defined(ENABLE_LED)
#define ledPin 6
#endif

//********Radio Defines ****************************
/** Radio pipe addresses for the 2 nodes to communicate. */
const uint64_t pipes[2] = {0x544d52685FLL, 0x544d526869LL};
