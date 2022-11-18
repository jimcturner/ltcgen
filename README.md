# ltcgen
This is a standalone C application (ignore the .CPP extension - it's C!) to generate LTC timecode through the standard audio ouput of a Raspberry Pi.

It is still in use (has been running reliably since 2018) but not touched since but this is no longer an active project

This repo is a reconstruction of the original based on a git clone exhumed from my local machine back in 2018. I'm not sure what happened the original repo hosted on githib, it seems to have disappeared. It is believed to be the latest version.
James Turner 18/11/22

Excerpts from the main source file....

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
