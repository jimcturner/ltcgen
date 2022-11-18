/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: turnej04
 *
 * Started on 19 September 2018, 13:40
 * 
 * Based on a tutorial here: https://alsa.opensrc.org/HowTo_Asynchronous_Playback
 * 
 * To get this to compile, you need to install:-
 * 
 *      1) sudo apt-get install libasound2-dev (will create and populate /usr/include/alsa/ folder
 *      2) sudo apt-get install build-essential g++
 *      3) Add the entry '-lasound' to the linker 'Additional Additional Options' in the project properties 
 *      
 * 
 * To get Netbeans to resolve all the functions defined in the alsa libraries, add the files into the Header folder of the project
 * 
 * This is a separate git branch freerunWithPeriodicResync
 * Aims: I'm assuming that the regular flashing of the light on my Leith clock is due to the fact that
 * either LTC frames are being skipped or repeated due to timing errors
 * Solution: Make it like my PIC implementation - have the program freerun, but with a periodic sync to the master clock
 * This will involve the calculaion of LTC data being called within the soundcard callback routine itself 
 * 
 * 
 * Known issue:- 
 *  1)Only works at 48kHZ sampling rate (simple solution, don't provide the option
 *  2) Occasional jitter on output causes the Leitch clock to get upset
 * 
 * 
 * 
 * 
 * IMPROVEMENTS:- 
 * 1)Shouldn't printf() the timecode o/p from the timecode thread. Potentially the redrawing will slow the thread down
 * 2)Pass period_size to the callback as part of the private data argument. All the globals are a bit sloppy!
 * 3)Use the input arguments to the program to 
 *      1)select 'silent' mode Where it doesn't echo the timecode to screen
        2)Allows scaling of the LPF output, to permit the output level to be controlled
 *      3)Allow manual setting of sample rate (need to test prog with different sample rates)
 *      4)To set the cutoff freq of the LPF
 *      5)To set debug levels
 * 
 *      This program generates LTC timecode
 * 
 *      It can be controlled by sending the USR1 process to the PID of the program once it's running
 *      eg, from the command line kill -s USR1 8947 (assuming pid of the process is 8947)
 * 
 *
 * 
 * 
 * See here: https://stackoverflow.com/questions/28521857/strange-sine-wave-on-output-of-an-alsa-linux-c-program

 * Sample output (for reference)
Attempting to set period time 40000 uS, with total buffer length of 200000 uS
Actual period size: 1920 stereo 16 bit samples (frames), Buffer size: 9600 stereo 16 bit samples (frames)
Actual period time: 40000uS, Actual buffer length: 200000uS
no_of_samples_per_LTC_symbol : 24
required_array_length_per_LTC_symbol:48
snd_async_add_pcm_handler: Success
Initial Free frames in the buffer: 9600

 */

#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include <alsa/control.h>
#include <errno.h>
#include <poll.h>
#include "alsa/pcm.h"
#include "alsa/error.h"
#include "alsa/global.h"
#include <signal.h>             //For the signal() line)
#include <string>
#include <string.h>
#include <iostream>
#include <memory>

#include <sys/types.h>          //for getpid(void)
#include <unistd.h>             //for getpid(void)

#include <inttypes.h>
#include <math.h>
#include <time.h>

#include <pthread.h>
#include <cmath>        //Remember to add -lpthread to linker options

#define LTC_DATA_AND_CLOCK_FRAME_LENGTH 160
#define SAMPLE_RATE 48000
#define CUTOFF_FREQ 14000       //Specifies the cutoff freq (in Hz) of the LPF of the wavetable samples
#define OUTPUT_LEVEL 100        //Specifies a scaling factor (in %)for the output of the LPF (and therefore the audio output level)
#define RESYNC_INTERVAL 15 * 60     //Specified (in seconds) the time between resyncronisation of freeRunningCurrentTime to the actual system
//3420 seconds =57 minutes
#define MESSAGE_STRING_LENGTH 200          //This is used to display messages on the console 
//time (currentTime). 
using namespace std;
//This holds the error code returned by the various ALSA functions
int err;


//struct tm *prev_time;

//Period size specifies the period in frames between callbacks
//This is required by the callback function so declare it as global.
//An improvement might be to include it in the private data field within the 
//snd_async_handler_t structure passed to the callback when the system calls it

//////////////////Global variables start
int debug = 0; //Used to control printf() debugging statements. Set to >0 to enable d
snd_pcm_uframes_t actual_period_size = 0; //Note this value is calculated, based on a time value
short *audio_data; //array to hold our audio data. Used by the callback function (a short is a signed 16bit integer)

int ltc_data_with_clock[LTC_DATA_AND_CLOCK_FRAME_LENGTH] = {0}; //Array to hold the clock+data biphase encoded LTC data frame.
pthread_mutex_t ltc_data_with_clock_Mutex = PTHREAD_MUTEX_INITIALIZER; //Create a mutex lock for ltc_data_with_clock[]

short *waveTable[4]; //Will hold the wavetables for the four output symbol states 00,01,10 and 11
//This will ultimately be 2D array, but we don't know for sure how many columns will be in that array yet
//This is dependant upon actual_period_size which is a value returned by the hardware for a given 
//sample rate and desired period time (which, in our case is 1/25th of a sec)
int shutdownFlag = 0;

//Create our own struct to hold the current time of day in h:m:s:f

struct CurrentTime {
    //Holds the current time of day in h:m:s:frames where a frame is 1/25th of a second
    int hours;
    int minutes;
    int seconds;
    int frames; //min value=0, max value=24
};

struct CurrentTime currentTime; //The time according to the system clock 
pthread_mutex_t currentTime_Mutex = PTHREAD_MUTEX_INITIALIZER; //Create a mutex lock for the current time struct


struct CurrentTime freeRunningCurrentTime; //The time according to the timing of the soundcard
pthread_mutex_t freeRunningCurrentTime_Mutex = PTHREAD_MUTEX_INITIALIZER; //Create a mutex lock for the non-sync freerun time struct

char messgageString [MESSAGE_STRING_LENGTH]; //Used to display useful messages on the console
int newMessageFlag; //Signals that a new message has arrived               
pthread_mutex_t messageString_Mutex = PTHREAD_MUTEX_INITIALIZER; //Create a mutex lock for for the messageString global char array

int output_level;
int resyncTimer; //Keeps track of the number of seconds between the last re-synchronisation of freeRunningCurrentTime to currentTime
////////////////Global variables end

void setMessageString(char messageToBeDisplayed[]) {
    //This function takes an input string and copies it into the global char array messgageString[] 
    //up to maximum length MESSAGE_STRING_LENGTH
    size_t len = strlen(messageToBeDisplayed);
    pthread_mutex_lock(&messageString_Mutex);

    if (len > MESSAGE_STRING_LENGTH)
        len = MESSAGE_STRING_LENGTH; //Truncate messages longer than the allowed buffer
    memcpy(messgageString, messageToBeDisplayed, len); //Copy new message into global message buffer
    newMessageFlag = 1; //Set the flag
    pthread_mutex_unlock(&messageString_Mutex);
}

void clearMessageFlag() {
    //Clears the globabl message buffer and resets the flag
    pthread_mutex_lock(&messageString_Mutex);
    memset(messgageString, 0, MESSAGE_STRING_LENGTH); //Clear existing messages
    newMessageFlag = 0; //Clear the message flag (acknowledge it)
    pthread_mutex_unlock(&messageString_Mutex);
}

char* getMessageString(char *output, unsigned int length) {
    //Takes a char buffer, and copies the contents of messageString[] into it
    pthread_mutex_lock(&messageString_Mutex);
    if (length > MESSAGE_STRING_LENGTH)
        length = MESSAGE_STRING_LENGTH; //Check to see that output buffer is not longer than the source buffer 
    memcpy(output, messgageString, length); //Copy global messageString[] into output buffer
    pthread_mutex_unlock(&messageString_Mutex);
    return output; //Return a pointer to the supplied char array (so you can printf() the output of this function easily
}

int getMessageFlag() {
    //Returns the value of newMessageFlag to determine whether a new message has been copied into the message buffer
    int retVal;
    pthread_mutex_lock(&messageString_Mutex);
    retVal = newMessageFlag;
    pthread_mutex_unlock(&messageString_Mutex);
    return retVal;

}

void printBits(size_t const size, void const * const ptr) {
    //Debugging function pinched from StackOverflow. Prints the binary representation of an int
    //Arg 1 is the size (in bits) of the int, Arg 2 is a pointer to the 
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    for (i = size - 1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    //puts("");
}

int extract_bit_value_from_char(char input, int bit_to_test) {
    //Takes a char and a bit to test (between 0 and 7)
    //If bit x is 1, returns a 1, else returns a zero, or -1 if bit_to_test is out of bounds
    /*int output;
    switch (bit_to_test) {
        case 0: output = 0b00000001 & input;
            if (output > 0) return 1;
        case 1: output=0b00000010 & input;
            if (output > 0) return 1;
    }
     */
    if (bit_to_test > 7) return -1;
    int bit_mask = 0b00000001;
    if (((bit_mask << bit_to_test) & input) > 0) return 1;
    else return 0;
}

void lowPassFilter(double* input, double* output, int bufferLength, int sampleRate, int cutoffFreq, int outputLevel) {
    //Code stolen directly from https://www.quora.com/Whats-the-C-coding-for-a-low-pass-filter
    //First order low pass filter
    double RC = 1.0 / (cutoffFreq * 2 * 3.14);
    double dt = 1.0 / sampleRate;
    double alpha = dt / (RC + dt);
    double scalingFactor = (double) outputLevel / 100.0;

    //perform LPF transformation
    output[0] = input[0] * scalingFactor; //Scale output
    //printf("[0]%f>>%f\n", input[0], output[0]);
    for (int i = 1; i < bufferLength; ++i) {
        output[i] = output[i - 1] + (alpha * (input[i] - output[i - 1])); //Apply low pass filtering process

        //printf("[%d]%f>>%f\n", i, input[i], output[i]);
    }
    //scale output values
    for (int i = 0; i < bufferLength; ++i) {
        //printf("%d[%f]>>",i,output[i]);
        output[i] = output[i] * scalingFactor; //Scale output
        //printf("[%f]\n",output[i]);
    }
}

void createPCMWaveforms(int required_array_length_per_LTC_symbol, int sample_rate) {
    //This function populates the global waveTable[4][n] array with a pcm representation of 
    //the desired binary output symbols (00,01,10,11) 
    //This function assumes that waveTable[] is a 4x waveform_length byte array

    //Create array to hold the input (pre LPF waveform)
    //Note: The var required_array_length_per_LTC_symbol takes into account 2 channels (stereo)
    //The input waveform only needs to be mono, so can be half the size

    //The normal LPF cutoff freq for LTC should be approx 14kHz. If a low sample rate is specified,
    //such that the freq response of the soundcard is less than that (see Nyquist) then the 
    //cutoff freq will have to modified to accomodate the waveform
    int monoWaveformLength = required_array_length_per_LTC_symbol / 2;
    double *rawWaveform = (double *) malloc(monoWaveformLength * sizeof (double));
    double *output = (double *) malloc(monoWaveformLength * sizeof (double));

    int lpf_cutoff_freq = CUTOFF_FREQ;
    if ((sample_rate / 2) < CUTOFF_FREQ) {
        lpf_cutoff_freq = (sample_rate / 2);
        printf("Sample rate too low for a default %dHz LPF cutoff, using %dHZ instead\n", CUTOFF_FREQ, lpf_cutoff_freq);
    }
    ///////////////////////////////
    //Create raw waveform for the 00 symbol. This will be a perfectly jagged square waveform
    rawWaveform[0] = 32767.0; //Any 0 will have been preceeded by a 1, so create a 1>0 transition at the symbol boundary
    for (int n = 1; n < monoWaveformLength; n++)
        rawWaveform[n] = -32767.0; //Write -ve pcm value

    lowPassFilter(rawWaveform, output, monoWaveformLength, SAMPLE_RATE, lpf_cutoff_freq, output_level); //Now low pass filter rawWaveform[] to lose the jaggedness
    //Now translate output[] (mono) into waveTable[] (stereo)
    //Create waveform 00 (for the 00) symbol
    for (int n = 0; n < required_array_length_per_LTC_symbol; n += 2) {
        int i = n >> 1; //output[] array is half the length of wavetable[]. Divide n by 2 to calculate index i

        //printf("[00]%d:%d:%f:%hi\n",n,i,output[i],(short)output[i]);
        waveTable[0][n] = (short) output[i]; // Write L PCM value
        waveTable[0][n + 1] = (short) output[i]; // Write R PCM value
    }
    /////////////////////////////// 
    //Create waveform for the 01 symbol. This will be a perfectly jagged square waveform
    rawWaveform[0] = 32767.0; //Any 0 will have been preceeded by a 1, so create a 1>0 transition at the symbol boundary
    for (int n = 1; n < (monoWaveformLength / 2); n++)
        rawWaveform[n] = -32767.0; //Write negative portion of waveform (the first half) 
    for (int n = (monoWaveformLength / 2); n < monoWaveformLength; n++)
        rawWaveform[n] = 32767.0; //Write positive portion of waveform (the second half)
    lowPassFilter(rawWaveform, output, monoWaveformLength, SAMPLE_RATE, CUTOFF_FREQ, output_level); //Now low pass filter rawWaveform[] to lose the jaggedness
    //Finally, translate output[] (mono) into waveTable[] (stereo)
    for (int n = 0; n < required_array_length_per_LTC_symbol; n += 2) {
        int i = n >> 1; //output[] array is half the length of wavetable[]. Divide n by 2 to calculate index i

        //printf("[01]%d:%d:%f:%hi\n",n,i,output[i],(short)output[i]);
        waveTable[1][n] = (short) output[i]; // Write L PCM value
        waveTable[1][n + 1] = (short) output[i]; // Write R PCM value
    }
    ///////////////////////////////
    //Create waveform for the 10 symbol
    rawWaveform[0] = -32767.0; //Any 1 will have been preceeded by a 0, so create a 0>1 transition at the symbol boundary
    for (int n = 1; n < (monoWaveformLength / 2); n++)
        rawWaveform[n] = 32767.0; //Write positive portion of waveform (the first half) 
    for (int n = (monoWaveformLength / 2); n < monoWaveformLength; n++)
        rawWaveform[n] = -32767.0; //Write negative portion of waveform (the second half)
    //Now low pass filter rawWaveform[] to lose the jaggedness
    lowPassFilter(rawWaveform, output, monoWaveformLength, SAMPLE_RATE, CUTOFF_FREQ, output_level);
    //Finally, translate output[] (mono) into waveTable[] (stereo)
    for (int n = 0; n < required_array_length_per_LTC_symbol; n += 2) {
        int i = n >> 1; //output[] array is half the length of wavetable[]. Divide n by 2 to calculate index i

        //printf("[10]%d:%d:%f:%hi\n",n,i,output[i],(short)output[i]);
        waveTable[2][n] = (short) output[i]; // Write L PCM value
        waveTable[2][n + 1] = (short) output[i]; // Write R PCM value
    }

    ///////////////////////////////
    //Create raw waveform for the 11 symbol. This will be a perfectly jagged square waveform
    rawWaveform[0] = -32767.0; //Any 0 will have been preceeded by a 0, so create a 0>1 transition at the symbol boundary
    for (int n = 1; n < monoWaveformLength; n++)
        rawWaveform[n] = 32767.0; //Write -ve pcm value
    //Now low pass filter rawWaveform[] to lose the jaggedness
    lowPassFilter(rawWaveform, output, monoWaveformLength, SAMPLE_RATE, CUTOFF_FREQ, output_level);

    //Finally, translate output[] (mono) into waveTable[] (stereo)
    for (int n = 0; n < required_array_length_per_LTC_symbol; n += 2) {
        int i = n >> 1; //output[] array is half the length of wavetable[]. Divide n by 2 to calculate index i

        //printf("[11]%d:%d:%f:%hi\n",n,i,output[i],(short)output[i]);
        waveTable[3][n] = (short) output[i]; // Write L PCM value
        waveTable[3][n + 1] = (short) output[i]; // Write R PCM value
        //printf("[11]%d:%hi,%hi\n",)
    }
    //Now free up memory
    free(rawWaveform);
    free(output);
}

void createLTCDataFrame(int *ltc, int frame_length, int hour, int min, int sec, int frame) {
    //This standalone function takes a pointer to an integer array and also a time in h:m:s:frames
    //It will then fill that buffer with a fully formed LTC data frame encoded with the clock
    //using biphase mark encoding

    //declare an array of char to hold the LTC data as bytes
    char ltc_data_byte_array[10] = {0};

    // Now we have a complete byte array of LTC data for a complete frame (10x 8 bytes).
    // However, for subsequent operations, it would be a lot more convenient if we could
    // just concatenate these bytes into a contiguous 80 element array.
    int ltc_data_raw[80] = {0}; //This holds the same bit values as ltc_data_byte_array but is easier to read/manipulate


    // ***** NOW GENERATE LTC DATA FRAME *****
    //The raw LTC data frame is 80 bits (or 10 bytes). Generally, only the bottom 3 or 4 bits of every byte are defined
    //(so the top 4 can be used for user data...except for certain bits, which are specified).
    //Bytes 8 and 9 (the last two) are a fixed value/pattern which serves as a recognisable sequence
    //to signify the start of a frame. And also the direction of travel (in the case of a VTR)
    //Byte 0: Frames (units
    //Byte 1: Frames (10s)
    //Byte 2: Seconds (units)
    //Byte 3: Seconds (tens)
    //Byte 4: Minutes (units)
    //Byte 5: Minutes (tens)
    //Byte 6: Hours (units)
    //Byte 7: Hours (tens)
    //Byte 8: Sync bit 0b11111100
    //Byte 9: Sync bit 0b10111111


    ltc_data_byte_array[0] = (char) (frame % 10) & 0b00001111; //Frame units. (Modulus discards the 10s)
    ltc_data_byte_array[1] = ((char) frame / 10)&0b00000011; //Frame tens
    ltc_data_byte_array[2] = (char) (sec % 10) & 0b00001111; // Second units
    ltc_data_byte_array[3] = ((char) sec / 10)&0b00000111; //Second tens
    ltc_data_byte_array[4] = (char) (min % 10) & 0b00001111; //Minute unit
    ltc_data_byte_array[5] = ((char) min / 10)&0b00000111; //Minute tens
    ltc_data_byte_array[6] = (char) (hour % 10) & 0b00001111; //Hour units
    ltc_data_byte_array[7] = ((char) hour / 10)&0b00000011; //Hour tens
    ltc_data_byte_array[8] = 0b11111100; //Sync word LSB
    ltc_data_byte_array[9] = 0b10111111; //Sync word MSB

    //Print entire 10 bytes of ltc_data_byte_array[]
    /*
    printf("%d:%d:%d:%d\n", frame, sec, min, hour);
    for (int n = 0; n < 10; n++) {
        printf("%d:", n);
        printBits(sizeof (ltc_data_byte_array[n]), &ltc_data_byte_array[n]);
        printf("\n");
    }
     */
    //Now calcuate parity bit, (bit 59, which is the fourth bit of byte 7 (LTC tens)) 
    //bit 59 is chosen to provide an even no of 0 bits in the total frame

    //Step1: XOR the whole LTC array to arrive at a single char
    char LTC_XORed = 0; //Holds the resultant of all the values XORd together
    for (int n = 0; n < 10; n++) {
        //Iterate over ltc_data_byte_array array XORing all elements together
        //XOR will return a 1 when an odd number of 1's. The evens will cancel out.
        //So, XOR returns true if odd no of 1's
        LTC_XORed = LTC_XORed^ltc_data_byte_array[n];
    }
    /*
    printf("X:");
        printBits(sizeof(LTC_XORed),&LTC_XORed);
        printf("\n");
     */
    //Step 2: XOR the final LTC_XORed value to see that has an odd/even no. of 1's
    int no_of_ones = 0;
    char mask = 0b00000001;
    for (int n = 0; n < 8; n++) {
        //Mask all the bits individually, and if there is a one, increment the count
        if (LTC_XORed & (mask << n)) {
            no_of_ones++;
        }
    }
    //Now see if result is divisible by 2. If it is, there are an even no. of 1's 
    //Mod(2) will return a zero if an even no, or a 1, if an odd number
    if (no_of_ones % 2) {
        //Must be an odd number or ones.
        //If there are an odd number of ones, there must also be an odd number of zeroes,
        //therefore we have to set bit 59=one to 'make even' the number of ones, and therefore zeroes
        ltc_data_byte_array[7] = ltc_data_byte_array[7] | 0b00001000; //Set the fourth bit
        //printf("Odd no of ones - setting 4th bit\n");
    } else {
        //There must be an even number of ones, and therefore an even no of zeroes,
        //so we don't need to do anything, except ensure bit 59 stays a zero
        ltc_data_byte_array[7] = ltc_data_byte_array[7]&0b00000111; //Clear the fourth bit
        //printf("Even no of ones\n");
    }

    // Now we have a complete byte array of LTC data for a complete frame (10x 8 bytes).
    // However, for subsequent operations, it would be a lot more convenient if we could
    // just concatenate these bytes into a contiguous 80 element array
    int k = 0;
    for (int bytes = 0; bytes < 10; bytes++) {
        //Iterate over ltc_data_byte_array[] copying bit values into ltc_data_raw[]
        //Note, we need to copy the bits from each byte out in reverse order to make sure
        //that the LSB is copied first
        //for (int bits = 7; bits>-1; bits--) {
        for (int bits = 0; bits < 8; bits++) {
            ltc_data_raw[k] = extract_bit_value_from_char(ltc_data_byte_array[bytes], bits);
            k++;
        }
    }

    // Print out entire 80 bits of ltc_data_raw[]
    if (debug == 4) {
        printf("\033[1A\033[105D%02d:%02d:%02d:%02d|", frame, sec, min, hour);
        for (k = 0; k < 80; k++) {
            printf("%d", ltc_data_raw[k]);
            if ((k + 1) % 8 == 0)printf("|");
        }
        printf("\n");
    }

    // Now fill the global 160 byte array that will contain both the data and the clock 
    // LTC uses  biphase mark code (also known as FM): a 0 bit has a single 
    // transition at the start of the bit period. A 1 bit has two transitions, 
    // at the beginning and middle of the period


    int prevValue = 0; //Holds the most recent LTC clock or data value.
    for (int n = 0; n < LTC_DATA_AND_CLOCK_FRAME_LENGTH; n += 2) {
        //Generate clock bit first. This will always be the inversion of the previous bit 
        ltc[n] = !prevValue;
        prevValue = ltc[n]; //Capture most recent bit value
        //Now generate data bit. Iterate over ltc_data_raw[] array, testing each bit in turn
        if (ltc_data_raw[(int) floor(n / 2)] > 0) { //if it's a 1, invert the next bit
            ltc[n + 1] = !prevValue;
            prevValue = ltc[n + 1]; //Capture most recent bit value
        } else {
            //If next data bit is a zero, data bit should stay the same as the clock value
            ltc[n + 1] = prevValue;
            prevValue = ltc[n + 1]; //Capture most recent bit value
        }
    }

    /*      
                    //Print out entire 160 bits of the ltc_data_with_clock array
                    for (int n = 0; n < LTC_DATA_AND_CLOCK_FRAME_LENGTH; n += 2) {
                        int k = (int) floor(n / 2);
                        printf("%d:(%d):%d,%d\n", k, ltc_data_raw[k], ltc[n], ltc[n + 1]);
                    }
                    //exit(0);
     */


}

static void refill_buffer_callback(snd_async_handler_t * pcm_callback) {
    //Callback function. This function will be called whenever the
    //soundcard audio buffer needs refilling

    //ALSA passes a snd_async_handler_t structure to this function.
    //We're able to retrieve the the handle for our pcm device from
    //this structure using snd_async_handler_get_pcm()

    //This callback function will fire every 1/25th of a second. However, it is not locked to system 
    //time so will drift. To minimise skipped/duplicate LTC frames caused by these discrepancies
    //the soundcard itself shall be used as a timing reference. At a periodic interval specified by
    //RESYNC_INTERVAL, it will crash lock to the actual system time

    //


    int h, m, s, f;
    if (resyncTimer < RESYNC_INTERVAL) { //Are we still within the free-running period?
        //Increment current freerun time by 1/25th sec
        //Lock access to the global freeRunningCurrentTime struct
        pthread_mutex_lock(&freeRunningCurrentTime_Mutex);
        freeRunningCurrentTime.frames++;
        if (freeRunningCurrentTime.frames > 24) {
            freeRunningCurrentTime.frames = 0;
            freeRunningCurrentTime.seconds++;

            //1 second has elapsed, so increment the resync timer
            resyncTimer++;
        }
        if (freeRunningCurrentTime.seconds > 59) {
            freeRunningCurrentTime.seconds = 0;
            freeRunningCurrentTime.minutes++;
        }
        if (freeRunningCurrentTime.minutes > 59) {
            freeRunningCurrentTime.minutes = 0;
            freeRunningCurrentTime.hours++;
        }
        if (freeRunningCurrentTime.hours > 23) {
            freeRunningCurrentTime.frames = 0;
            freeRunningCurrentTime.seconds = 0;
            freeRunningCurrentTime.minutes = 0;
            freeRunningCurrentTime.hours = 0;
        }
        //Copy global freeRunningCurrentTime into a local var so we can release the mutex lock
        h = freeRunningCurrentTime.hours;
        m = freeRunningCurrentTime.minutes;
        s = freeRunningCurrentTime.seconds;
        f = freeRunningCurrentTime.frames;

        pthread_mutex_unlock(&freeRunningCurrentTime_Mutex);
    } else {
        resyncTimer = 0; //Reset the interval timer

        //Get the current system time and copy into a local variable
        pthread_mutex_lock(&currentTime_Mutex);
        h = currentTime.hours;
        m = currentTime.minutes;
        s = currentTime.seconds;
        f = currentTime.frames;
        pthread_mutex_unlock(&currentTime_Mutex);

        //Now copy the values of the local var back into the global freeRunningCurrentTime struct
        pthread_mutex_lock(&freeRunningCurrentTime_Mutex);
        //char output[MESSAGE_STRING_LENGTH]={0}; //Buffer to hold the resync message
        //printf("Resyncronising to system time: %d:%d:%d:%d\n",h,m,s,f);
        //sprintf(output,"Last resync to system time: %02d:%02d:%02d:%02d",h,m,s,f);   //Format message
        //setMessageString(output);       //Send message to output buffer 
        freeRunningCurrentTime.hours = h;
        freeRunningCurrentTime.minutes = m;
        freeRunningCurrentTime.seconds = s;
        freeRunningCurrentTime.frames = f;
        pthread_mutex_unlock(&freeRunningCurrentTime_Mutex);
    }



    //Get exclusive access to global ltc_data_with_clock[]
    pthread_mutex_lock(&ltc_data_with_clock_Mutex);

    //Create an LTC data frame based on the current system time
    createLTCDataFrame(ltc_data_with_clock, LTC_DATA_AND_CLOCK_FRAME_LENGTH, h, m, s, f);

    //Take a local copy of ltc_data_with_clock[]
    int local_copy_ltc_data_with_clock[LTC_DATA_AND_CLOCK_FRAME_LENGTH] = {0};
    size_t checkArraySizeDst = sizeof (local_copy_ltc_data_with_clock);


    size_t checkArraySizeSrc = sizeof (ltc_data_with_clock);

    //Before we copy the src and dst arrays, check they are the same size (in bytes)    
    if (checkArraySizeSrc != checkArraySizeDst) {
        printf("refill_buffer_callback(): Array size Mismatch between ltc_data_with_clock[%d] and local_copy_ltc_data_with_clock[%d]\n",
                (int) checkArraySizeSrc, (int) checkArraySizeDst);
        exit(1);
    }
    //Actually make the local copy of ltc_data_with_clock[]
    memcpy(local_copy_ltc_data_with_clock, ltc_data_with_clock, sizeof (ltc_data_with_clock));
    //Release the lock on ltc_data_with_clock[]
    pthread_mutex_unlock(&ltc_data_with_clock_Mutex);


    //Now we can deal with the audio.....

    //Retrieve a handle to our pcm device
    snd_pcm_t *pcm_handle = snd_async_handler_get_pcm(pcm_callback);
    snd_pcm_sframes_t avail_frames;

    //Calculate total size (in bytes) of our audio data array
    int audio_data_length = actual_period_size * 2; //x2 because each frame consists of two samples (L and R)

    //The period size is calculated to correspond with a single frame of LTC data (i.e 1/25 of a second)
    //Within that period are 80 LTC symbols (a symbol includes clock and data)
    int no_of_samples_per_LTC_symbol = (int) actual_period_size / 80;
    //Each stereo 16bit sample requires 4 bytes. Calculate required array length in bytes (a short is two bytes)
    int array_length_per_LTC_symbol = no_of_samples_per_LTC_symbol * 2;

    //iterate over local_copy_ltc_data_with_clock[160] to generate the correct pcm data
    int audio_data_bytes_written = 0; //This keeps track of how many audio samples worth of data
    //we've written

    for (int n = 0; n < LTC_DATA_AND_CLOCK_FRAME_LENGTH; n += 2) { //This loop will run 80 times
        //Given the biphase mark coding, there are only four possible values per symbol period
        //(a symbol period consists of clock and data)
        //These are 00, 01, 10 and 11 (or 0,1,2,3 in decimal)

        //Examine local_copy_ltc_data_with_clock[n] and [n+1] to create a composite
        //of the clock+data value (whose possible values will be 0b00,01,10,11
        //We can then select which of our wavetables to write to the soundcard buffer
        int symbolValue = (local_copy_ltc_data_with_clock[n] << 1) | local_copy_ltc_data_with_clock[n + 1];
        //int ltc_bit_index=n>>1;    //Divide n by 2 to get a value between 0 and 79
        //printf("%d:%d:symbolValue: %d\n",n,ltc_bit_index, symbolValue);

        //Depending upon the value of symbolValue, copy one of the four wavetables into audio_data[]
        //Each wavetable is a short[] array of length 96 bytes (24 samples x2 bytes per sample x2 channels)
        //By way of a sanity check, 96 * 80 = 7680
        //7680 bytes is the length of data we need to write to the soundcard every 40mS
        //Why? Because we have a period size of 1920 frames, but an audio frame consists of 
        //Two 16 bit stereo interleaved samples (i.e 4 bytes per frame)
        //Therefore 4*1920=7680

        //With each iteration of the loop, we specify a new starting index for our mem copy
        //This offset is determined by [audio_data_bytes_written]
        //The correct waveTable is chosen by [symbolValue]

        for (int m = 0; m < array_length_per_LTC_symbol; m++) {
            audio_data[m + audio_data_bytes_written] = waveTable[symbolValue][m];
        }

        //Now update the index so that we don't overwrite the same part of the pcm buffer each time
        audio_data_bytes_written += array_length_per_LTC_symbol;

        //Check to make sure that we haven't fallen off the end of thje pcm buffer audio_data[]
        //This could happen if the soundcard driver is unable to create a soundcard buffer with
        //period length of 96 bytes.
        if ((audio_data_bytes_written) > audio_data_length) {
            printf("refill_buffer_callback(). audio_data_index(%d)>audio_data_length(%d), buffer overrun\n",
                    audio_data_bytes_written, audio_data_length);
            break;
        }
    }

    //Poll the soundcard driver to see how much data (in frames) we can send to the buffer
    avail_frames = snd_pcm_avail_update(pcm_handle);

    if (avail_frames < 0) {
        printf("refill_buffer_callback::snd_pcm_avail_update(): %s\n", snd_strerror(avail_frames));
        exit(1);
    }

    //Check to see that a portion of the audio buffer is available to write to (and we're not about to shut down)
    if ((avail_frames >= actual_period_size)&&!shutdownFlag) { //Whilst period_size fits into available space

        //Copying in contents of audio_data until all free space in the buffer has been filled
        err = snd_pcm_writei(pcm_handle, audio_data, actual_period_size);
        if (err < 0) {
            printf("refill_buffer_callback::snd_pcm_writei() %s\n", snd_strerror(err));
            if (int err = snd_pcm_prepare(pcm_handle) < 0) {
                printf("refill_buffer_callback::snd_pcm_prepare(): %s\n", snd_strerror(err));
            }
        }
    }
}

void *displayCurrentTimeThread(void *arg) {
    //This thread runs in the background, and displays the current time on the console
    sleep(1); //Allow time for the program to start running
    //Create local CurrentTime struct to hold copy of data in global CurrentTime struct (holds the current time of day in h:m:s:f)
    //char messageText[MESSAGE_STRING_LENGTH];
    struct CurrentTime ct, frt;
    int secsUntilResync, minsUntilResync;
    int diffSec, diffFrames;
    printf("\n\n\n"); //Move cursor down three lines in preparation
    while (1) {
        pthread_mutex_lock(&currentTime_Mutex); //Gain exclusive access to currentTime global struct
        ct = currentTime; //Take snapshot of global struct
        pthread_mutex_unlock(&currentTime_Mutex); //Release exclusive access to currentTime global struct

        pthread_mutex_lock(&freeRunningCurrentTime_Mutex); //Gain exclusive access to currentTime global struct
        frt = freeRunningCurrentTime; //Take snapshot of global struct
        pthread_mutex_unlock(&freeRunningCurrentTime_Mutex); //Release exclusive access to currentTime global struct

        //Calculate error
        diffSec = frt.seconds - ct.seconds; //Take abs() (non zero vaue)
        diffFrames = abs(frt.frames - ct.frames); //+ve = LTC ahead, -ve LTC behind

        printf("\033[2A\033[50DSystem time: %02d:%02d:%02d:%02d", ct.hours, ct.minutes, ct.seconds, ct.frames);
        printf("\033[1B\033[50D   LTC time: %02d:%02d:%02d:%02d, Error: %02d:%02d   ", frt.hours, frt.minutes, frt.seconds, frt.frames, diffSec, diffFrames);
        //printf("\033[1B\033[100D%s",getMessageString(messageText,MESSAGE_STRING_LENGTH));
        //secondsUntilResync=RESYNC_INTERVAL-resyncTimer;     //Bit naughty. Should really use a mutex lock for this
        minsUntilResync = (RESYNC_INTERVAL - resyncTimer) / 60; //Calculate display mins
        secsUntilResync = (RESYNC_INTERVAL - resyncTimer)-(minsUntilResync * 60); //Calculate display secs

        printf("\033[1B\033[50D%d:%02d  until resync to system time", minsUntilResync, secsUntilResync);


        //delay thread for 100mS 
        struct timespec delay;
        delay.tv_sec = 0;
        delay.tv_nsec = (100 * 100000);
        nanosleep(&delay, NULL);
    }
}

void *getSystemTimeinFramesThread(void *arg) {
    //This thread keeps track of the system time (to 1/25th second resolution and populates  
    //the global currentTime struct
    long systemTimeInMs; // Milliseconds
    time_t elapsedSystemTime; // Seconds
    struct timespec currentSystemTime;
    int currentFrameIndex, prevFrameIndex;


    //Declare structure to contain time information (hr, min sec etc) and pass the current time (s)
    //(calculated in seconds) to localtime() to be converted to something more human-readable
    //by breaking the number of seconds into h:m:s
    struct tm *time_now;

    while (1) { //Sit in an infinite loop that runs every 20mS
        clock_gettime(CLOCK_REALTIME, &currentSystemTime); //Get system time (in seconds since the epoch)



        elapsedSystemTime = currentSystemTime.tv_sec; // Get system time (in seconds since the epoch)
        systemTimeInMs = round(currentSystemTime.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
        if (systemTimeInMs > 999) {
            //If, by chance the ms>999, increment the 'second' count and reset the mS value to zero
            elapsedSystemTime++;
            systemTimeInMs = 0;
            //printf("\033[1B>1000mS!\n");
        }
        time_now = localtime(&elapsedSystemTime); //Pass elapsed system time to extract to h:m:s     
        //printf("Current time: %"PRIdMAX".%03ld seconds since the Epoch\n",
        //        (intmax_t) elapsedSystemTime, systemTimeInMs);


        //printf("updateCurrentTimeThread %d:%d:%d:%03ld\n", time_now->tm_hour, time_now->tm_min, time_now->tm_sec,systemTimeInMs);

        //Now we have the system time to mS resolution, use the mS value to calculate an equivalent value in frames
        //We check for a new frame every 20mS (Nyquist sampling theorum) to give 40mS resolution
        //If the frame hasn't chamged, there's no need to rewrite the values
        currentFrameIndex = floor(systemTimeInMs / 40); //Calculate the frame no by div by 40. 1000ms/40=1/25 sec (or 40mS per frame)
        //debug = 1;
        if (currentFrameIndex != prevFrameIndex) {
            pthread_mutex_lock(&currentTime_Mutex); //Gain exclusive access to the global currentTime struct
            currentTime.hours = time_now->tm_hour;
            currentTime.minutes = time_now->tm_min;
            currentTime.seconds = time_now->tm_sec;
            currentTime.frames = currentFrameIndex; //Calculate the frame no by div by 40. 1000ms/40=1/25 sec (or 40mS per frame)
            if (debug == 3) printf("%d:%d:%d:%d:(%d)\n", currentTime.hours, currentTime.minutes,
                    currentTime.seconds, currentTime.frames, (int) elapsedSystemTime); //print verbose system time
            if (debug == 1)
                printf("\033[1A\033[12D%02d:%02d:%02d:%02d\n", currentTime.hours, currentTime.minutes,
                    currentTime.seconds, currentTime.frames);

            prevFrameIndex = currentFrameIndex;
            pthread_mutex_unlock(&currentTime_Mutex); //Make global currentTime struct available once more
        }
        //delay thread for 20mS 
        struct timespec delay;
        delay.tv_sec = 0;
        delay.tv_nsec = (20 * 1000000);
        nanosleep(&delay, NULL);
    }
}

void shutdownSoundcard(snd_pcm_t *pcm_handle) {
    shutdownFlag = 1;
    //Keep playing until the audio buffer is emptied (blocking function)
    err = snd_pcm_drain(pcm_handle);
    printf("snd_pcm_drain(): %s\n", snd_strerror(err));

    //Free memory allocated for audio_data[]
    //free(audio_data);
    //Free up audio hardware
    err = snd_pcm_hw_free(pcm_handle);
    printf("snd_pcm_free(): %s\n", snd_strerror(err));

    //Close the audio device                
    printf("Attempting to close pcm device\n");
    err = snd_pcm_close(pcm_handle);
    printf("snd_pcm_close(): %s\n", snd_strerror(err));
}


int main(int argc, char *argv[]) {


    output_level = OUTPUT_LEVEL; //Set global output_level to the default (unless an argument supplied at run time)
    int sample_rate = SAMPLE_RATE; //Set sample rate to default (unless an argument supplied at run time)
    int silent_mode = 0; //silent_mode specifies whether the program updates the console in real-time with the current time
    if (argc > 1) {
        //Has a parameter been supplied into the program
        for (int n = 0; n < argc; n++) {
            string realString = argv[n]; //convert to c string
            //printf("Args supplied: %s\n\n", argv[n]);
            if (realString == "?") {
                printf("Use the following args: \n-o output level (0-100)%%, \n-s silent mode (no time update on the console),\n-r sample rate (Hz, 11025-96000)\n"
                        "Note, currently only the default (48kHZ) sample rate actually works");
                exit(0);
            }

            if (realString == "s") {
                printf("Running in silent mode:");
                silent_mode = 1;
            }
            if (realString == "o") {
                if ((argc - 1) < n + 1) //Check that 'o' is followed by another argument
                    printf("no output level supplied, setting to 100%%\n");
                else {
                    //Validate the supplied arg
                    char *ptr;
                    int l = (int) strtol(argv[n + 1], &ptr, 10); //Convert string to integer
                    if (l < 0)l = 0;
                    if (l > 100)l = 100;
                    printf("Setting output level to %d%%\n", l);
                    output_level = l; //Set global output_level var
                }
            }
            if (realString == "r") {
                if ((argc - 1) < n + 1) //Check that 'r' is followed by another argument
                    printf("no sample rate specified, using 44.1kHz\n");
                else {
                    //Validate the supplied arg
                    char *ptr;
                    int l = (int) strtol(argv[n + 1], &ptr, 10); //Convert string to integer
                    if (l < 11025)l = 11025;
                    if (l > 96000)l = 96000;
                    printf("Setting sample rate to %dHz\n", l);
                    sample_rate = l;
                }
            }

        }

    }
    //Main program starts here
    printf("LTC Timecode Generator. (c) James Turner 2019\n");
    //sleep(1);
    //Force audio output to 100% (using ALSA command line command, bit of a bodge.
    system("amixer cset numid=1 -- 100");
    /*
    for (int x = 0; x < 59; x++) {
        print_current_time_with_ms();
        sleep(1);
    }
    exit(0);
     */
    //The pcm device name
    //const char *device_name = "default"; //Default doesn't work, have to specify the name instead
    const char *device_name = "hw:0,0";
    //Our device handle
    snd_pcm_t *pcm_handle = NULL;

    //Set up trapping of Linux signals so that we can trap ctrl-c etc and gracefully close the app
    sigset_t sigsToBlock;
    sigemptyset(&sigsToBlock); //initialise and empty the signal set
    sigaddset(&sigsToBlock, SIGTERM); //Add SIGTERM to our signal set
    sigaddset(&sigsToBlock, SIGINT); //Add SIGINT to our signal set
    sigaddset(&sigsToBlock, SIGUSR1); //Add SIGUSR1 to our signal set
    sigaddset(&sigsToBlock, SIGUSR2); //Add SIGUSR2 to our signal set
    sigaddset(&sigsToBlock, SIGKILL); //Add SIGKILL to our signal set

    sigprocmask(SIG_BLOCK, &sigsToBlock, NULL); //Apply the mask to this program

    //Send signal from terminal using kill -s USR1 [pid]]
    sigset_t sigsToCatch; //Signal set

    sigemptyset(&sigsToCatch); //initialise and empty the signal set
    sigaddset(&sigsToCatch, SIGUSR1); //Add USR1 to our signal set
    sigaddset(&sigsToCatch, SIGUSR2); //Add USR2 to our signal set
    sigaddset(&sigsToCatch, SIGKILL); //Add SIGKILL to our signal set
    sigaddset(&sigsToCatch, SIGTERM); //Add SIGTERM to our signal set (Ctrl T)
    sigaddset(&sigsToCatch, SIGINT); //Add SIGINT to our signal set (Ctrl C)

    int caught; //Will store the returned 'signal number'
    /*
        //Create a separate thread to determine the current time in h:m:s:video frames
        //The global struct currentTime will be accessible by multiple threads so for 
        //safety, use a mutex lock to prevent concurrent read/writes

        pthread_t _createLTCDataFrameThread;
        if (pthread_create(&_createLTCDataFrameThread, NULL, createLTCDataFrameThread, NULL)) {
            printf("main(): Error creating _generateLTCDataFrameThread\n");
        }
     */
    //Create a thread to calculate the system time in frames (and update the global currentTime struct
    pthread_t _getSystemTimeinFramesThread;
    if (pthread_create(&_getSystemTimeinFramesThread, NULL, getSystemTimeinFramesThread, NULL)) {
        printf("main(): Error creating _getSystemTimeinFramesThread\n");
    }
    sleep(1); //Allow time for getSystemTimeinFramesThread to start 

    //Create a separate thread to write the current time to the console 

    pthread_t _displayCurrentTimeThread;
    if (!silent_mode && pthread_create(&_displayCurrentTimeThread, NULL, displayCurrentTimeThread, NULL)) {
        printf("main(): Error creating _displayCurrentTimeThread\n");
    }

    //Force a resync between freerunTime and system time at program startup
    //This will be trapped by the soundcard callback routine
    resyncTimer = RESYNC_INTERVAL;

    ////////////Now deal with the soundcard////////////////

    // Open the pcm device
    err = snd_pcm_open(&pcm_handle, device_name, SND_PCM_STREAM_PLAYBACK, 0);

    //Error check
    if (err < 0) {
        fprintf(stderr, "cannot open audio device %s (%s)\n",
                device_name, snd_strerror(err));
        pcm_handle = NULL;
        exit(1);
    }

    //    Before we can actually feed something to this device handle we just created, 
    //    we have to tell it what we're feeding. We do this using hardware parameters,
    //    which are kept in a structure of type snd_pcm_hw_params_t. 
    //    First, we allocate this structure, and then we fill it with information 
    //    from our device's current state:

    //Step 1: Create and allocate memory for the structure
    snd_pcm_hw_params_t *hw_params;
    snd_pcm_hw_params_malloc(&hw_params);
    snd_pcm_hw_params_any(pcm_handle, hw_params);

    //Step 2: Fill the structure with our desired hardware parameters
    //Stereo (2 channel), 16 bit, 44.1kHz
    unsigned int rrate = (unsigned int) sample_rate;
    snd_pcm_hw_params_set_access(pcm_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_handle, hw_params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_rate_near(pcm_handle, hw_params, &rrate, NULL);
    snd_pcm_hw_params_set_channels(pcm_handle, hw_params, 2);

    //Query soundcard to see if it accepted our supplied sampling rate
    unsigned int actual_sample_rate;
    snd_pcm_hw_params_get_rate(hw_params, &actual_sample_rate, NULL);
    if (sample_rate != actual_sample_rate) {
        printf("Specified sampling rate not supported. Using  %dHz instead\n", (int) actual_sample_rate);
    }
    //Set the buffer/period size
    //We want an interrupt IRQ of 1/25th of a second (0.04) (because LTC counts in video frames)
    //I have guessed and set an overall buffer length of 5 times that figure, or 0.2 secs

    //At 44.1kHz (44100 samples per sec)
    //44100/25 LTC frames per second=1764 samples per LTC frame
    //1764/80 LTC symbols per LTC frame =22.05 samples per LTC symbol (where a symbol
    //includes the clock and data bit) - round down to 22 samples

    //Therefore we would like to have a period size of 1764 samples, and a buffer size
    //of five times that seems reasonable. 1764*5=8820

    int dir = 0; //Not sure what this is for
    /*
    //Set period size to 1764 samples 
    snd_pcm_uframes_t  desired_period_size = 1764;
    snd_pcm_hw_params_set_period_size (pcm_handle, hw_params, desired_period_size, dir);
    
    //Set total size of buffer to to five periods
    unsigned int desired_no_of_periods = 5;
    snd_pcm_hw_params_set_periods_near (pcm_handle, hw_params, &desired_no_of_periods, &dir);
    
    unsigned int resultant_buffer_length=desired_period_size*desired_no_of_periods;
    
    printf("Attempting to set period length %d samples, with total buffer length of %d samples\n",
            desired_period_size,resultant_buffer_length);
      
    
     */
    unsigned int desired_period_time = 40000; //Set (interrupt fire) period time in us (0.04=1/25th sec)
    unsigned int desired_buffer_time = 200000; //Set ring buffer length in us (0.2 seconds)
    printf("Attempting to set period time %d uS, with total buffer length of %d uS\n",
            (int) desired_period_time, (int) desired_buffer_time);

    //Set length (in time) of hardware ring buffer
    snd_pcm_hw_params_set_buffer_time_near(pcm_handle, hw_params, &desired_buffer_time, &dir);
    //Set the period time 
    snd_pcm_hw_params_set_period_time_near(pcm_handle, hw_params, &desired_period_time, &dir);

    //Note the set...near() functions will overwrite &buffer_time and &period_time with 
    //the actual values that were possible
    //printf("Using Buffer time: %duS, Period time: %duS\n", buffer_time, period_time);

    //We can now query the soundcard to determine (in samples) the resultant buffer_size and
    //period_size in (also samples) based on our desired period_size and desired_no_of_periods
    //If the soundcard was unable to create a the desired period size, this throws our calculation off
    snd_pcm_uframes_t actual_buffer_size = 0;
    //snd_pcm_uframes_t  actual_period_size = 0; //This has been declared as a global var
    unsigned int actual_buffer_time = 0;
    unsigned int actual_period_time = 0;

    snd_pcm_hw_params_get_period_size(hw_params, &actual_period_size, &dir);
    snd_pcm_hw_params_get_buffer_size(hw_params, &actual_buffer_size);
    snd_pcm_hw_params_get_period_time(hw_params, &actual_period_time, &dir);
    snd_pcm_hw_params_get_buffer_time(hw_params, &actual_buffer_time, &dir);

    printf("Actual period size: %d stereo 16 bit samples (frames), Buffer size: %d stereo 16 bit samples (frames)\n", (int) actual_period_size, (int) actual_buffer_size);
    printf("Actual period time: %duS, Actual buffer length: %duS\n", (int) actual_period_time, (int) actual_buffer_time);

    //Now calculate the PCM wavetables based on our known period size
    //The period size is calculated to correspond with a single frame of LTC data (i.e 1/25 of a second)
    //Within that period are 80 LTC symbols (a symbol includes clock and data)
    int no_of_samples_per_LTC_symbol = (int) actual_period_size / 80;
    //Each stereo 16bit sample requires 2 shorts. Calculate required array length in shorts (a short is two bytes)
    int required_array_length_per_LTC_symbol = no_of_samples_per_LTC_symbol * 2;
    printf("no_of_samples_per_LTC_symbol : %d\n", no_of_samples_per_LTC_symbol);
    printf("required_array_length_per_LTC_symbol:%d\n", required_array_length_per_LTC_symbol);

    //Now dynamically allocate memory for 4x PCM wavetables
    for (int i = 0; i < 4; i++)
        waveTable[i] = (short *) malloc(required_array_length_per_LTC_symbol * sizeof (short));

    //Generate the wavetables containing our PCM waveforms 
    createPCMWaveforms(required_array_length_per_LTC_symbol, actual_sample_rate);

    //Write the hardware parameters to the actual pcm device
    snd_pcm_hw_params(pcm_handle, hw_params);

    //Now free up memory used by the hw structure (its work is done)
    snd_pcm_hw_params_free(hw_params);

    //Specify necessary software parameters
    //Software parameters define things like how full the buffer needs to be before
    //playback can commence etc.

    //Create a pointer to a structure to hold them
    snd_pcm_sw_params_t *sw_params;

    //Allocate memory for the structure
    snd_pcm_sw_params_malloc(&sw_params);
    snd_pcm_sw_params_current(pcm_handle, sw_params);

    //Specify that soundcard should only start playing once the buffer is full
    snd_pcm_sw_params_set_start_threshold(pcm_handle, sw_params, actual_buffer_size);

    //Specify that the ALSA device should alert us when one 'period size' of data (i.e 0.04s) 
    //has been been played out, so that we can refill the ring buffer with new data
    snd_pcm_sw_params_set_avail_min(pcm_handle, sw_params, actual_period_size);

    //Apply the parameters to the soundcard
    snd_pcm_sw_params(pcm_handle, sw_params);

    //Free up memory used by the soundcard structure
    snd_pcm_sw_params_free(sw_params);



    //Allocate memory for the audio data array
    //We will only ever write [period_size] bytes to the ring buffer in a single write,
    //So that's as much space as we need for our buffer
    //The period_size is calculated from the period_time of 0.04secs which corresponds
    //to a single LTC data frame at 25 video frames/sec

    //Create char array for audio data. Each 16bit stereo audio frame takes 2 shorts
    audio_data = (short *) malloc(actual_period_size * 2);

    //Fill the buffer with silence
    for (int n = 0; n < (actual_period_size * 2); n += 2) {
        audio_data[n] = 0; //Left PCM sample
        audio_data[n + 1] = 0; //Right PCM sample
    }

    //setupSoundcard(pcm_handle);
    //Prepare the soundcard for playback
    if (int err = snd_pcm_prepare(pcm_handle) < 0) {
        printf("snd_pcm_prepare(): %s\n", snd_strerror(err));
        exit(1);
    }

    //Now send some random audio to the soundcard ring buffer
    //We've set the ring buffer to be 5x period size, so to fill the buffer
    //up we need 5 lots of period_size  
    snd_pcm_sframes_t avail_frames; //The amount of frames we can write to the audio buffer
    //Note a frame of stereo audio equiv to 4 bytes

    //Initial check to see if there is space in the soundcard buffer for us to write to
    avail_frames = snd_pcm_avail_update(pcm_handle);
    printf("Initial Free frames in the buffer: %d\n", (int) avail_frames);
    if (avail_frames < 0) {
        printf("snd_pcm_avail_update(): %s\n", snd_strerror(avail_frames));
        exit(1);
    }


    //int frames_written = 0;

    while (avail_frames >= (actual_period_size)) {
        //Check there is still space in the hardware buffer
        avail_frames = snd_pcm_avail_update(pcm_handle);

        //Copy the contents of audio_data[] into the hardware buffer
        //frames_written += snd_pcm_writei(pcm_handle, audio_data, actual_period_size);
        //        printf("Initial frames written %d\n", frames_written);
        if (int err = snd_pcm_writei(pcm_handle, audio_data, actual_period_size) < 0) {
            printf("main()::snd_pcm_writei(): %s\n", snd_strerror(err));
            //exit(1);
            printf("Waiting 1 second before attempting to write to soundcard once more\n");
            sleep(1);

            if (int err = snd_pcm_prepare(pcm_handle) < 0) {
                printf("snd_pcm_prepare(): %s\n", snd_strerror(err));
                exit(1);
            }
        }
    }
    //Create a pointer to a callback function
    snd_async_handler_t *pcm_callback;
    //Register that pointer with the ALSA event handler
    err = snd_async_add_pcm_handler(&pcm_callback, pcm_handle, refill_buffer_callback, NULL);
    printf("snd_async_add_pcm_handler: %s\n", snd_strerror(err));

    //Now sit in endless loop waiting to intercept a system signal]
    //The signal mask imposed earlier should mean that this thread is the only
    //one that can intercept signal messages

    //Get PID of this process
    pid_t pid = getpid();
    printf("'kill -s USR1 %d' to resync LTC output to system time\n", (int) pid);
    while (1) {



        sigwait(&sigsToCatch, &caught); //Blocking call to wait for SIGUSR1
        switch (caught) { //Determine which of the messages has been received
            case SIGUSR1:
                //printf("SIGUSR1\n");
                //USR1 signal used to force resync of LTC time to system time
                resyncTimer = RESYNC_INTERVAL;
                break;
            case SIGUSR2:
                printf("SIGUSR2 - \n");
                shutdownSoundcard(pcm_handle); //Invoke shutdown function
                exit(1);
                break;

            case SIGTERM: case SIGINT: case SIGKILL:
                if (caught == SIGTERM) printf("Recived SIGTERM\n");
                if (caught == SIGINT) printf("Received SIGINT\n");
                if (caught == SIGKILL) printf("Received SIGKILL\n");
                shutdownSoundcard(pcm_handle); //Invoke shutdown function

                /*
                shutdownFlag = 1;
                //Keep playing until the audio buffer is emptied (blocking function)
                err = snd_pcm_drain(pcm_handle);
                printf("snd_pcm_drain(): %s\n", snd_strerror(err));

                //Free memory allocated for audio_data[]
                //free(audio_data);
                //Free up audio hardware
                err = snd_pcm_hw_free(pcm_handle);
                printf("snd_pcm_free(): %s\n", snd_strerror(err));

                //Close the audio device                
                printf("Attempting to close pcm device\n");
                err = snd_pcm_close(pcm_handle);
                printf("snd_pcm_close(): %s\n", snd_strerror(err));
                exit(0);
                break;
                 */
                //Free up memory used by the waveTable[][] array
                for (int i = 0; i < 4; i++)
                    free(waveTable[i]);
                exit(0);
                break;

        }
    }
}
