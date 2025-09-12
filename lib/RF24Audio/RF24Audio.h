/**
 * @file RF24Audio.h
 *
 * Class declaration for RF24Audio Library
 *
 * RFAudio Library: by TMRh20 2011-2014
 */

#ifndef __RF24Audio_H__
#define __RF24Audio_H__

class RF24;

/**
 * @brief Arduino Realtime Audio Streaming library
 *
 * This class implements an Audio Streaming library using nRF24L01(+) radios driven
 * by the [Optimized RF24 library](https://github.com/TMRh20/RF24).
 */

class RF24Audio
{

public:
    /**
     * Setup the radio and radio identifier
     * @note Changing radioNum is only required if utilizing private node-to-node communication as
     * opposed to broadcasting to the entire radio group
     *
     * @code
     *	RF24 radio(48, 49);          // Initialize the radio driver
     *	RF24Audio rfAudio(radio, 0); // Initialize the audio driver
     * @endcode
     *
     * @param _radio   The underlying radio driver instance
     */
    RF24Audio(RF24 &_radio);

    /**
     * Initialize the radio and audio library
     *
     * Generally called in setup to initialize the radio
     * @code
     *	rfAudio.begin();
     * @endcode
     */
    void begin();

    /**
     * Volume Control
     * @code
     * rfAudio.volume(1); // Raise the volume
     * @endcode
     * @param upDn Set 0 to lower volume, 1 to raise volume
     *
     */
    void volume(bool upDn);

    /**
     * Volume Control
     * @code
     * rfAudio.setVolume(4); // Set the volume to mid-level
     * @endcode
     * @param vol Set at 0 to 7 for range of volume control
     *
     */
    void setVolume(char vol);

    /**
     * Control transmission through code
     * @code
     * rfAudio.transmit(); // Begin realtime audio streaming
     * @endcode
     * Call this function to begin transmission
     *
     */
    void transmit();

    /**
     * Stop transmission through code
     * @code
     *	rfAudio.receive(); // Stop audio streaming
     * @endcode
     * Call this function to stop transmission
     *
     */
    void receive();

private:
    RF24 &radio;
    void timerStart();
};

/**
 * Global helper function to RF24Audio::transmit(). Do not use this directly.
 * Use RF24Audio::transmit() instead of this function as RF24Audio needs to be
 * properly setup first.
 */
void TX();

/**
 * Global helper function to RF24Audio::receive(). Do not use this directly.
 * Use RF24Audio::receive() instead of this function as RF24Audio needs to be
 * properly setup first.
 */
void RX();

#endif // __RF24Audio_H__

/**
 * @example GettingStarted.ino
 * This sketch is intended to demonstrate the basic functionality of the audio library.
 */

/**
 * @example Minimal.ino
 * This sketch is intended to demonstrate the most basic functionality of the audio library.
 */

/**
 * @example PrivateChannels.ino
 * This sketch is demonstrates the use of private channels (node-to-node) in a multi-radio group.
 */

/**
 * @example PrivateGroups.ino
 * This sketch is demonstrates the use of private groups (node-to-custom_groups) in a multi-radio group.
 */

/**
 * @example USB_Audio.ino
 * This sketch is demonstrates how to interact with the audio library directly using the core
 * RF24 radio library: http://nRF24.github.io/RF24/
 */

/**
 * @page Setup Boards & Wiring
 * @section Board Wiring
 * This page displays different options for wiring/board configuration.
 *
 * @image html "images/NRF1.jpg" height=25% width=25%
 *
 * Wiring diagram for DIY module connector. May not be needed depending on module:
 * @image html "images/RF24AudioBasic_LargeAntenna.jpg" height=45% width=45%
 * @image html "images/NRF2.jpg" height=20% width=20%
 * @image html "images/RF24AudioBasic_SmallAntenna.jpg" height=45% width=45%
 * Wiring diagram for SD streaming/multicast using TMRpcm library:
 * @image html "images/RF24Audio_FullSD.jpg" height=65% width=65%
 */