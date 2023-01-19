
#include "daisysp.h"
#include "daisy_seed.h"

void voiceON(uint8_t note);
void voiceOFF(uint8_t note);

void HandleMidiMessage(daisy::MidiEvent m);
void controlchange(uint8_t channel, uint8_t number, uint8_t value);
void pitchbend(uint8_t channel, int bend);

//fjol.cpp
void initializeadc();
void initializecontrols();
void initializeaudio();
void updatecontrols();
void updateSwitches();
void updateadc();

void getorsetsettings(bool getorset);

float calcright(float theta);
float calcleft(float theta);
void updatespread();

void update_params(int destination, float value, float mod);
void select_algorithm();
void calibrate();
void glideval();
//all midi functions
void midipanic();
void setpwm(uint8_t channel, uint8_t pwm);
void startPCA9622();

//led-enc.cpp
float encoder_read(int8_t step);
void led_update(uint8_t index);
void led_menu_step(uint8_t number, uint8_t color);



