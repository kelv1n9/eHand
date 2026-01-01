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

/************  Optional/Advanced User Variables  ************/

/** The size of the memory buffer to use. Not really configurable (set to maximum by default). */
#define BUFFER_SIZE 32

/** The message protocol header **/
#define PROTOCOL_HEADER 0xA5

/************  Automated pin selections, override by defining above  ************/

#define ANALOG_PIN A0
#define SPEAKER_PIN 9
#define LED_PIN 6

//********Radio Defines ****************************
/** Radio pipe addresses for the 2 nodes to communicate. */
const uint64_t pipes[2] = {0x544d52685FLL, 0x544d526869LL};
