#include "daisysp.h"
#include "daisy_seed.h"
#include "settings.h"
#include "variables.h"
using namespace daisysp;
using namespace daisy;

static DaisySeed  hw;
I2CHandle i2c;
//struct for saving and loading data between restarts
struct data
  {
    float d_param[8] = {0,0,0,0,0,0,0,0};
    //ctl1dest,ctl1cvdest,ctl2dest,ctl2cvdest,algorithm,midi_channel
    uint8_t d_states[6] = {0,0,4,4,0,1}; //
  };
PersistentStorage<data> storage(hw.qspi);

static constexpr I2CHandle::Config i2c_config
    = {I2CHandle::Config::Peripheral::I2C_1,
       {{DSY_GPIOB, 8}, {DSY_GPIOB, 9}},
       I2CHandle::Config::Speed::I2C_1MHZ};

MidiUartHandler midi; //initialize midi object
//ADC configuration object
AdcChannelConfig adcConfig;

struct envStruc //amplitude envelope
{
  Adsr env;
  float attack;
  float release;
  float envSig;
  bool gate;
};

struct ADStruc //for filter envelope
{
  AdEnv env;
  float attack;
  float release;
  float envSig;
};

struct op
{
  Oscillator op1;
  Oscillator op2;
  Oscillator op3;
  
  //pan of voice
  float panL = 0; 
  float panR = 0;
  //operator routing
  bool op32; //op3 to op2
  bool op31; //op3 to op1
  bool op21; //op2 to op1 
  bool op2out; //op2 to audio output
  //amplitude of operators
  float op1amp = 0.75;
  float op2amp;
  float op3amp;
  //ratio of operators
  float ratio2;
  float ratio3;
  //detune of operator2 and 3
  float op2_detune;
  float op3_detune;
};

struct filter_
{
  Svf filter;
  float freq;
  float res;
  float drive = 0;
};

struct gate_i
{
  GateIn gate;
};

struct knob
{
  AnalogControl input;
  Parameter param;
  float value;
};

struct rotary
{
    Encoder enc;
    uint8_t ctldest = 0;
    uint8_t ctlcvdest = 0;
};

op voice[voicecount]; 
envStruc envelope[voicecount];
ADStruc filtenvelope; //stereo
filter_ stfilter[2]; //stereo

Switch shift_sw;
Switch algo_sw;
Switch midi_sw;
gate_i gate[4];
knob voct[4];
knob controls[8]; //8 knobs
knob cv[2]; //2 cv ins for ctl1 and ctl2
rotary enc[2];

//debug variables
bool debug_pots = 1;
bool debug_midi = 0;


static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size)
{
    for(int i = 0; i < voicecount;i++)
    {
      voice[i].op1.SetFreq(voicefreq[i]);
      voice[i].op2.SetFreq(voicefreq[i]*ratioindex[(int)knob_value[1]]*(1+voice[i].op2_detune*0.2));
      voice[i].op3.SetFreq(voicefreq[i]*ratioindex[(int)knob_value[3]]*(1+voice[i].op3_detune*0.2));
    }

    float left,right,temp1,temp2,temp3;

    for(size_t i = 0; i < size; i += 2)
    {
      for(int k = 0; k < voicecount;i++)
      {
        envelope[k].envSig = envelope[k].env.Process(voicetrig[k]);

        voice[k].op1.SetAmp(envelope[k].envSig);
        voice[k].op2.SetAmp(envelope[k].envSig*controls[0].value);
        voice[k].op3.SetAmp(envelope[k].envSig*controls[1].value);

        //operator 3 processing
        temp1 = voice[k].op3.Process();
        //operator 2 processing
        voice[k].op2.PhaseAdd(temp1*voice[k].op32);
        temp2 = voice[k].op2.Process();
        voice[k].op2.PhaseAdd(param[7]*temp2); //operator 2 feedback
        temp2 = voice[k].op2.Process(); 
        //operator 1 processing
        voice[k].op1.PhaseAdd(temp1*voice[k].op31 + temp2*voice[k].op21);
        temp3 = voice[k].op1.Process();
        voice[k].op1.PhaseAdd(temp3*param[6]); //operator 1 feedback

        //stereo mix
        left+=voice[k].op1.Process()*voice[k].panL;
        left+=(temp2*voice[k].op2out*voice[k].panL)*pm1amp;
        right+=voice[k].op1.Process()*voice[k].panR;
        right+=(temp2*voice[k].op2out*voice[k].panR)*pm1amp;

      }

      //stereo filter with envelope
      if(filtenv_trig){filtenvelope.env.Trigger();}
      filtenvelope.envSig = filtenvelope.env.Process();
      float sum = (filtenvelope.envSig*20000) + controls[6].value;
      sum = DSY_CLAMP(sum,20,20000);
      
      for(int j = 0; i<2;i++)
      {
        stfilter[j].filter.SetFreq(sum);
        stfilter[j].filter.SetFreq(param[1]);
      }
      stfilter[0].filter.Process(left);
      stfilter[0].filter.Process(right);

      //main out
      // left out
      out[i] = stfilter[0].filter.Low();
      // right out
      out[i + 1] = stfilter[1].filter.Low();
    }
}


int main(void)
{
  hw.PrintLine("Initialize started");

  i2c.Init(i2c_config);

  initializeadc();
  //Initialize the adc with the config we just made
  hw.adc.Init(&adcConfig, 1);

  hw.Init();
  hw.SetAudioBlockSize(2);
  initializecontrols();

  shift_sw.Debounce();
  if(shift_sw.Pressed())
  {
    calibrate();
  }
  
  updatecontrols();

  //getorsetsettings(load); //get settings from EEPROM

  samplerate = hw.AudioSampleRate();
  initializeaudio();
  hw.StartAudio(AudioCallback);

  startPCA9622();

  for(int i = 0; i<15;i++)
  {
    setpwm(i,128);
    System::Delay(25);
    setpwm(i,0);
    System::Delay(25);
  }
  led_update(8);
  MidiUartHandler::Config midi_config;
  midi.Init(midi_config);
  midi.StartReceive();
  while(1)
  {
    updatecontrols();
    /** Listen to MIDI for new changes */
    midi.Listen();
    // Handle MIDI Events
    while(midi.HasEvents())
    {
      HandleMidiMessage(midi.PopEvent());
    }
    if(led_takeover == 0)
    {
      for(int i = 0;i<8;i++)
      {
        if(params_changed[i] == 1 && !(midi_sw.Pressed()) && !(algo_sw.Pressed()) && !(shift_sw.Pressed()))
        {
          led_update(i);
          params_changed[i] = 0; 
        }
      } //for ends
    } //if led takeover ends
        
    //if any other UI feature has taken over the LED's then count up to 2 seconds and go back to the normal UI for LED's
    if( led_takeover == 1 && ((System::GetNow()- ledlastmillis) >= 500) )
    {
       led_takeover = 0;
      //Serial.println(led_takeover);
       led_update(8); //update all leds
    } 
    
      //---------glide-------
    if ((System::GetNow() - lastMillis) >= glidetime)
    {
      lastMillis = System::GetNow(); //get ready for the next iteration
      glideval();
    }
  } //while ends
 hw.PrintLine("OH SHIT, WHILE ENDED!!");
} //main ends


float calcright(float theta) //calculate -4.5dB pan law
{

  float right = sqrt((theta) * (2 / PI) * sin(theta));
  //Serial.println(String("Right:") + right);
  return right;
}

float calcleft(float theta) //calculate -4.5dB pan law
{
  float left = sqrt((PI / 2 - (theta)) * (2 / PI) * cos(theta));
  //Serial.println(String("Left:") + left);
  return left;
}

void updatespread()
{
   //spread algorithm starts
  float righttemp = calcright( ((param[0]/2)+0.5)*PI/2 );
  float lefttemp = calcleft( ((param[0]/2)+0.5)*PI/2 );
  voice[0].panL = lefttemp;
  voice[0].panR = righttemp;
  voice[3].panL = lefttemp;
  voice[3].panR = righttemp;

  righttemp = calcright( (( (1-param[0])/400 ) +0.25)*PI/2 );
  lefttemp = calcleft( (( (1-param[0])/400 )+0.25)*PI/2 );
  voice[1].panL = lefttemp;
  voice[1].panR = righttemp;
  voice[2].panL = lefttemp;
  voice[2].panR = righttemp;
}

void getorsetsettings(bool getorset)
{
  uint8_t eeaddress = 0;

  data qspidata;
  if (getorset == load)
  {
   
    for(int i = 0; i<8; i++)
    {
      param[i] = qspidata.d_param[i];
    }
    enc[0].ctldest = qspidata.d_states[0];
    enc[0].ctlcvdest = qspidata.d_states[1];
    enc[1].ctldest = qspidata.d_states[2];
    enc[1].ctlcvdest = qspidata.d_states[3];
    algo_select = qspidata.d_states[4];
    midichannel = qspidata.d_states[5];
    return;
  }
  else if (getorset == save)
  {
    // Serial.println(String("saved data to point:")+ eeaddress);
    // EEPROM.put(0, midichannel);
    for(int i = 0; i<8; i++)
    {
       qspidata.d_param[i] = param[i];
    }
    qspidata.d_states[0] = enc[0].ctldest;
    qspidata.d_states[1] = enc[0].ctlcvdest;
    qspidata.d_states[2]= enc[1].ctldest;
    qspidata.d_states[3] = enc[1].ctlcvdest;
    qspidata.d_states[4] = algo_select;
    qspidata.d_states[5] = midichannel;
    //** Erase qspi and then write our new data
    // for(int i = 0; i<8;i++)
    // {
    //   hw.qspi.Erase(eeaddress, eeaddress);
    //   hw.qspi.Write(eeaddress, sizeof(qspidata.d_param[i]), qspidata.d_param[i]);
    //   eeaddress += sizeof(qspidata.d_param[i]);
    //   System::Delay(1);
    // }
    // for(int i = 0; i<6;i++)
    // {
    //   hw.qspi.Erase(eeaddress, eeaddress);
    //   hw.qspi.Write(eeaddress, sizeof(qspidata.d_param[i]), qspidata.d_states[i]);
    //   eeaddress += sizeof(qspidata.d_param[i]);
    // }
    return;
  }
} //get or set ends

void glideval()
{
  int voicenumb = 4;
  if (glidetime > 0)
  {

    for (int i = 0; i < voicenumb; i++)
    {
      if (voicefreqtarget[i] > int(voicefreq[i]))
      {
        voicefreq[i] = voicefreq[i] + glide;
        if (int(voicefreq[i]) > voicefreqtarget[i])
          voicefreq[i] = voicefreqtarget[i];
      }
      else if (voicefreqtarget[i] < int(voicefreq[i]))
      {
        voicefreq[i] = voicefreq[i] - glide;
        if (int(voicefreq[i]) < voicefreqtarget[i])
          voicefreq[i] = voicefreqtarget[i];
      }
      else
      {
        voicefreq[i] = voicefreqtarget[i];
      }
    }
  } //if glidetime > 0 ends
  else if (glidetime == 0)
  {
    for (int i = 0; i < voicenumb; i++)
    {
      voicefreq[i] = voicefreqtarget[i];
    }
  }
} //glideval() ends

void midipanic()
{
  voicetrig[0] = 0;
  voicetrig[1] = 0;
  voicetrig[2] = 0;
  voicetrig[3] = 0;
  gate_out = 0;
}

void update_params(int destination, float value, float mod)
{
  
  params_changed[destination] = 1;
  for(int i = 0; i<8; i++)
  {
    if(i == destination)
    {
      param[i]+= value;
      if(param[i] > 1){param[i] = 1; params_changed[i] = 0; led_update(i);}
      if(param[i] > 0){param[i] = 0; params_changed[i] = 0; led_update(i);}
      param_sum[i] = param[i] + cv_mod[i];
      if(destination == 0){updatespread();}
    }
  }
}

void select_algorithm()
{
  switch(algo_select)
    {
      case 0:
      for(int i = 0; i<4; i++)
      {
        voice[i].op21 = 1;
        voice[i].op31 = 1;
        voice[i].op32 = 0;
        voice[i].op2out = 0;
      }
      return;
      break;
      case 1:
      for(int i = 0; i<4; i++)
      {
        voice[i].op21 = 1;
        voice[i].op31 = 0;
        voice[i].op32 = 1;
        voice[i].op2out = 0;
      }
      return; 
      break;
      case 2:
      for(int i = 0; i<4; i++)
      {
        voice[i].op21 = 0;
        voice[i].op31 = 1;
        voice[i].op32 = 0;
        voice[i].op2out = 1;
      }
      return;
      break;
    }
}

void updatecontrols()
{
  float temp = 0;
  if(led_takeover == 0)
  {
   for(int i= 0; i<2; i++)
   {
     enc[i].enc.Debounce();
     if(enc[i].enc.Pressed() == 0)
     {temp = encoder_read(enc[i].enc.Increment());}
     if(temp != 0)
     {update_params(enc[i].ctldest,temp,0);}
   }
  }
  updateSwitches();
  updateadc();
}

void updateadc()
{

  for(int i = 0; i<8; i++)
  {
    controls[i].value = controls[i].param.Process();
    //this is to prevent unnecessary changes
    if(controls[i].value != knob_value[i])
    {
      knob_value[i] = controls[i].value;
    }
  }
  for(int i = 0; i<2; i++)
  {
    cv_value[i] = cv[i].input.Process();
    if(cv_mod[enc[i].ctlcvdest] != cv_value[i])
    {
      cv_mod[enc[i].ctlcvdest] = cv_value[i];
      params_changed[i] = 1;
    }
  }
  
  //reset values that cv destination isn't mapped to
  for(int j = 0; j<1; j++)
  {
   for(int i = 0; i<7; i++)
   {
     if(i != enc[j].ctlcvdest)
     {
       cv_mod[enc[j].ctlcvdest] = 0;
     }
   }
  }//reset cv array ends
  //set attack decay for envelope
  for(int i = 0; i<4; i++)
  {
    envelope[i].env.SetAttackTime(knob_value[4]);
    envelope[i].env.SetDecayTime(knob_value[5]);
  }
  filtenvelope.env.SetMax(knob_value[7]); //set amplitude of envelope for filter
}

void updateSwitches()
{
  
  shift_sw.Debounce();
  //get encoder switches
  for(int i=0;i<2;i++)
  {
    enc[i].enc.Debounce();
    if((shift_sw.Pressed() == false) && enc[i].enc.RisingEdge())
    {
      enc[i].ctldest = enc[i].ctldest+1;
      if(enc[i].ctldest > 3 && i == 0){enc[i].ctldest = 0;}
      if(enc[i].ctldest > 7 && i == 1){enc[i].ctldest = 0;}
      led_takeover = 1;
      ledlastmillis = System::GetNow();
      led_menu_step(enc[i].ctldest,green);
      //getorsetsettings(1,0); 
    }
    //ctl1 cv destination
    if((shift_sw.Pressed() == true) && enc[i].enc.RisingEdge())
    {
      enc[i].ctlcvdest = enc[i].ctlcvdest+1;
      if(enc[i].ctlcvdest > 3 && i == 0){enc[i].ctlcvdest = 0;}
      if(enc[i].ctlcvdest > 7 && i == 1){enc[i].ctlcvdest = 0;}
      led_takeover = 1;
      ledlastmillis = System::GetNow();
      led_menu_step(enc[i].ctlcvdest,red);
      //getorsetsettings(1,0); 
    }
  }
  //get algorithm switch
  algo_sw.Debounce();
  if(algo_sw.RisingEdge() && shift_sw.Pressed() == false)
  {
    algo_select++;
    algo_select = DSY_CLAMP(algo_select,0,3);
     //PM algorithm select
    //AudioNoInterrupts();
    select_algorithm();
    led_takeover = 1;
    ledlastmillis = System::GetNow();
    led_menu_step(algo_select,red);
    //getorsetsettings(save); //save settings
  }
  else if(algo_sw.RisingEdge() && shift_sw.Pressed() == true)
  {
    getorsetsettings(save); //save settings
  }
  midi_sw.Debounce();
  if(midi_sw.RisingEdge())
  {
    while(midi_sw.Pressed())
    {
      if(midi_sw.TimeHeldMs() >= 2000)
      {
        learn_active = 1;
      }
      else
      {
        midipanic();
      }
    }
  } //midi switch ends
  //gate expander
  for(int i = 0; i<4;i++)
  {
    if(gate[i].gate.State())
    {
      //trigger envelope i
    }
    else
    {
      //stop envelope i
    }
  }
}

void initializeaudio()
{

  //initialize operators and VCA envelopes
  for(int i = 0;i<voicecount;i++)
  {
    voice[i].op1.Init(samplerate);
    voice[i].op2.Init(samplerate);
    voice[i].op3.Init(samplerate);
    voice[i].op1.SetWaveform(Oscillator::WAVE_SIN);
    voice[i].op2.SetWaveform(Oscillator::WAVE_SIN);
    voice[i].op3.SetWaveform(Oscillator::WAVE_SIN);
    voice[i].op1.SetAmp(pm1amp);
    envelope[i].env.Init(samplerate);
    envelope[i].env.SetSustainLevel(1);
    envelope[i].env.SetDecayTime(0);
  }

  filtenvelope.env.Init(samplerate);
  filtenvelope.env.SetCurve(50); //set curve to inbetween lin and log
  filtenvelope.env.SetMin(0.001); //set it to minimum 20hz

  //initialize filters
  for(int i = 0;i<2;i++)
  {
    stfilter[i].filter.Init(samplerate);
    stfilter[i].filter.SetDrive(0);
  }
  
}
void initializeadc()
{
  //init ADC
  adcConfig.InitSingle(hw.GetPin(ctrl1cv_pin));
  adcConfig.InitSingle(hw.GetPin(ctrl2cv_pin));
  adcConfig.InitMux(hw.GetPin(mux_pin),8,hw.GetPin(mux_a_pin),hw.GetPin(mux_b_pin),hw.GetPin(mux_c_pin));
  adcConfig.InitSingle(hw.GetPin(voct1_pin));
  adcConfig.InitSingle(hw.GetPin(voct2_pin));
  adcConfig.InitSingle(hw.GetPin(voct3_pin));
  adcConfig.InitSingle(hw.GetPin(voct4_pin));
}
void initializecontrols()
{
  uint8_t knob_mapping[8] = {4,2,5,3,7,6,1,0};
  float callbackrate = hw.AudioCallbackRate();
  //how the fuck does this init work?? what object does it want??
  // gate[0].gate.Init(&(hw.GetPin(gate1_pin)));
  // gate[1].gate.Init(&(hw.GetPin(gate2_pin)));
  // gate[2].gate.Init(&(hw.GetPin(gate3_pin)));
  // gate[3].gate.Init(&(hw.GetPin(gate4_pin)));

  algo_sw.Init(hw.GetPin(sw2_pin),callbackrate);
  shift_sw.Init(hw.GetPin(sw1_pin),callbackrate);
  midi_sw.Init(hw.GetPin(midi_sw_pin),callbackrate);

  enc[0].enc.Init(hw.GetPin(ctl1_enc_a_pin),hw.GetPin(ctl1_enc_b_pin),hw.GetPin(ctl1_sw_pin),callbackrate);
  enc[1].enc.Init(hw.GetPin(ctl2_enc_a_pin),hw.GetPin(ctl2_enc_b_pin),hw.GetPin(ctl2_sw_pin),callbackrate);

  for(int i = 0; i<8; i++)
  {
    if(i<2){cv[i].input.Init(hw.adc.GetPtr(i),callbackrate,false,true,0.002);}
    controls[i].input.Init(hw.adc.GetMuxPtr(2,knob_mapping[i]),callbackrate,false,true,0.002);
    
  }
  for(int i=3;i<7;i++)
  {
    voct[i-3].input.Init(hw.adc.GetPtr(i),callbackrate,false,true,0);
  }
  controls[0].param.Init(controls[0].input,0,1,Parameter::LOGARITHMIC); //pm2
  controls[1].param.Init(controls[1].input,0,1,Parameter::LINEAR); //ratio2
  controls[2].param.Init(controls[2].input,0,1,Parameter::LOGARITHMIC); //pm3
  controls[3].param.Init(controls[3].input,0,1,Parameter::LINEAR); //ratio3
  controls[4].param.Init(controls[4].input,0,3,Parameter::LINEAR); //attack
  controls[5].param.Init(controls[5].input,0,2,Parameter::LINEAR); //decay
  controls[6].param.Init(controls[6].input,20,20000,Parameter::LOGARITHMIC); //cutoff
  controls[7].param.Init(controls[7].input,0,1,Parameter::LOGARITHMIC); //envfilter amplitude
}

void setpwm(uint8_t channel, uint8_t pwm)
{
  i2c.WriteDataAtAddress(PCA9622_I2C_ADDRESS,channel+2,2,&pwm,2,10);
}

void startPCA9622()
{
  uint8_t generalsettings = 0;
  uint8_t pwmsettings = 255;
  //start PWM oscillator
  i2c.WriteDataAtAddress(PCA9622_I2C_ADDRESS,PCA9622_I2C_ADDRESS,1,&generalsettings,1,10);
  System::Delay(1);
  //set mode2 register to zero
  i2c.WriteDataAtAddress(PCA9622_I2C_ADDRESS,PCA9622_I2C_ADDRESS+1,1,&generalsettings,1,10);
  System::Delay(1);
  //Set LEDs to enable PWM output
  for(int i = 0; i<4; i++)
  {
    i2c.WriteDataAtAddress(PCA9622_I2C_ADDRESS,0x14+i,1,&pwmsettings,1,10);
    System::Delay(1);
  }
}

void shiftvoices()
{
  uint8_t temp;
  temp = voicehist[3];
  voicehist[3] = voicehist[2];
  voicehist[2] = voicehist[1];
  voicehist[1] = voicehist[0];
  voicehist[0] = temp;
}

void voiceOFF(uint8_t note)
{
  for(int i = 0; i<4; i++)
  {
    if((voicetrig[i] == 1) && (note == voicenote[i]))
    {
      voicetrig[i] = 0;
    }
  }
  if (gate_out && voicetrig[0] == 0 && voicetrig[1] == 0 && voicetrig[2] == 0 && voicetrig[3] == 0 )
  {
    gate_out = 0;
    filtenv_trig = 0;
  }

}


void voiceON(uint8_t note)
{
  float freq_note = mtof(note - 1) * pbend * octaveindex[octave];
  
  if (!gate_out)
  {
    gate_out = 1;
    filtenv_trig = 1;
  }
  for(int i = 0; i<4; i++)
  {
    if(voicehist[0] == i+1 && !voicetrig[i])
    {
      voicenote[i] = note;
      voicetrig[i] = 1;
      shiftvoices();
      voicefreqtarget[i] = freq_note;
      if(envelope[i].env.IsRunning())
      {
        envelope[i].env.Retrigger(0);
      }
      return;
     }
  }
 
}