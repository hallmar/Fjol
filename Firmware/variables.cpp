#include "settings.h"
#include "variables.h"
#include "daisysp.h"
#include "daisy_seed.h"

using namespace daisysp;
using namespace daisy;



//-------Calibration values------

uint16_t calibration_val[20] = {0, 4095 , 0, 4095 ,0, 4095 ,0, 4095 ,0, 4095 ,0, 4095 ,0, 4095 ,0, 4095 ,};

//gate out status, maybe move to struct or class?
uint8_t gate_out = 0;
uint8_t gate_out_n = 1;

//-------led menu variables-----
bool led_takeover = 0;


//------analog variables-----
float knob_value[8] = {0,0,0,0,0,0,0,0};
float cv_value[2] = {0,0};
float param[8] = {0,0,0,0,0,0,0,0};
float param_sum[8] = {0,0,0,0,0,0,0,0};
bool params_changed[8] = {0,0,0,0,0,0,0,0};
float cv_mod[8] = {0,0,0,0,0,0,0,0};
//-----constant variables------
const float ratioindex[18] = { 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9};
const float octaveindex[5] = {0.25,0.5,1,2,3}; //might be useful
const int releasemult = 3000; //move this to local function
const int attackmult = 2000; //move this to local function
const float pm1amp = 0.75; //move this to local function to save space

//-----Midi notes variables-----
uint8_t voicenote[4] = { 0, 0, 0, 0};
bool voicetrig[4] = { 0, 0, 0, 0};
bool filtenv_trig = 0;
uint8_t voicehist[4] = {4, 3, 2, 1}; //0th place is the oldest voice 3rd place is newest voice
uint8_t midichannel;
float voicefreq[4] = {};
float voicefreqtarget[4] = {};
float pbend = 1;
int glide = 1;
float glidetime = 0;
uint8_t glideon = 0;
bool midi_received = 0;
bool learn_active = 0;
uint8_t incoming_note = 0;
uint8_t incoming_velocity = 0;
uint8_t incoming_channel = 0;

//-----Voice settings variables----

float phasemod = 1; //1 = 2*pi = 360°, move to local function to save space!!
uint8_t octave = 2;

uint8_t algo_select = 1;
uint8_t detunemult = 30;

//encoder variables
uint8_t velocity_low = 15;
uint8_t velocity_mid = 10; //ms
uint8_t velocity_mid_high = 5;
uint8_t velocity_high = 1; //ms


unsigned long ledlastmillis = System::GetNow();
unsigned long lastMillis = ledlastmillis;
unsigned long savelastMillis = ledlastmillis;
unsigned long calibration_lastmillis = ledlastmillis;
unsigned long lastmillis1 = ledlastmillis;
unsigned long lastmillis2 = ledlastmillis;

float brightness = 1;
int pwm_resolution = 255;

float samplerate = 0;