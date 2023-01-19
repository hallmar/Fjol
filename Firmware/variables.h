#ifndef variables_h

#define variables_h

#define PCA9622_I2C_ADDRESS 0x60 // NOTE: Make sure to use the correct I2C address as the PCA9622 can have 128 different addresses
#define voicecount 4
#define AD_curve  50 //-100 to 100, positive creates LOG curve
#define PI 3.14159

//-------output pins--------
#define gate_pin 10
#define gateled_pin 2
#define mux_a_pin 31
#define mux_b_pin 30
#define mux_c_pin 29

//------Input pins---------
#define ctl1_enc_a_pin 3
#define ctl1_enc_b_pin 4
#define ctl2_enc_a_pin 6
#define ctl2_enc_b_pin 5

#define sw2_pin 9
#define sw1_pin 8
#define ctl1_sw_pin 11
#define ctl2_sw_pin 7
#define midi_sw_pin 32

//x0 control3
//x1 control1_cv
//x2 control2_cv
//x3 ratio2
//x4 PM2
//x5 PM3
//x6 release
//x7 attack
#define mux_pin 24 

#define ctrl1cv_pin 22
#define ctrl2cv_pin 23

#define gate1_pin 36
#define gate2_pin 35
#define gate3_pin 34
#define gate4_pin 33

#define voct1_pin 28
#define voct2_pin 27
#define voct3_pin 26
#define voct4_pin 25

//-----i2c pins--------
#define scl1_pin 12
#define sda1_pin 13
#define scl2_pin 14
#define sda2_pin 15

#define green 1
#define red 2

#define load 0
#define save 1

#include "daisysp.h"
#include "daisy_seed.h"


//-------Calibration values------

extern uint16_t calibration_val[20];

//gate out status, maybe move to struct
extern uint8_t gate_out;
extern uint8_t gate_out_n;

//-------led menu variables-----
extern bool led_takeover;


//------analog variables-----
extern float knob_value[8];
float cv_value[2];
extern float param[8];
extern float param_sum[8];
extern bool params_changed[8];
extern float cv_mod[8];

//-----constant variables------
extern const float ratioindex[18];
extern const float octaveindex[5]; //might be useful
extern const int releasemult; //move this to local function
extern const int attackmult; //move this to local function
extern const float pm1amp; //move this to local function to save space

//-----Midi notes variables-----
extern uint8_t voicenote[4];
extern bool voicetrig[4];
extern bool filtenv_trig;
extern uint8_t voicehist[4]; //0th place is the oldest voice 3rd place is newest voice
extern uint8_t midichannel;
extern float voicefreq[4];
extern float voicefreqtarget[4];
extern float pbend;
extern int glide;
extern float glidetime;
extern uint8_t glideon;
extern bool midi_received;
extern bool learn_active;
extern uint8_t incoming_note;
extern uint8_t incoming_velocity;
extern uint8_t incoming_channel;

//-----Voice settings variables----

extern float phasemod; //1 = 2*pi = 360°, move to local function to save space!!
extern uint8_t octave;

//-----FM and osc settings values-----
extern uint8_t algo_select;

extern uint8_t detunemult;

extern uint8_t load_preset;
extern uint8_t save_preset;


extern long lastpos_1;
extern long lastpos_2;

extern uint8_t velocity_low;
extern uint8_t velocity_mid; //ms
extern uint8_t velocity_mid_high;
extern uint8_t velocity_high; //ms


extern unsigned long ledlastmillis;
extern unsigned long lastMillis;
extern unsigned long savelastMillis;
extern unsigned long calibration_lastmillis;
extern unsigned long lastmillis1;
extern unsigned long lastmillis2;

extern float brightness;
extern int pwm_resolution;

extern float samplerate;
#endif // variables_h