#include <cmath>
#include "daisy_seed.h"
#include "daisysp.h"
#include "dev/bela_trill.h"
#include "dev/oled_ssd130x.h"

#define PIN_MUX_SEL_2 19
#define PIN_MUX_SEL_1 20
#define PIN_MUX_SEL_0 21
#define PIN_ADC_POT_MUX 16
#define PIN_ADC_POT_MUX2 15
#define PIN_Z 22

#define PIN_OLED_DC 9
#define PIN_OLED_RESET 30

#define PIN_ENC_A 1
#define PIN_ENC_B 2
#define PIN_ENC_CLICK 3

#define ADC_CHANNELS 4

using namespace daisy;
using namespace daisysp;

DaisySeed               hw;

daisy::UI ui;
UiEventQueue    eventQueue;
UI::SpecialControlIds specialControlIds;
UiCanvasDescriptor oledDisplayDescriptor;

Trill touchSensor;
bool gate;
bool sustainNote;
//bool syncOnOff;
Switch panicButton, backButton, functionButton;
MidiUsbHandler midi;

OledDisplay<SSD130x4WireSpi128x64Driver> display;
FullScreenItemMenu mainMenu;
FullScreenItemMenu lfoMenu;
FullScreenItemMenu lfoMapMenu;
FullScreenItemMenu controlMapMenu;
FullScreenItemMenu paramLimitMenu;
FullScreenItemMenu pitchEnvMenu;
FullScreenItemMenu xMapMenu;
FullScreenItemMenu yMapMenu;
FullScreenItemMenu zMapMenu;
FullScreenItemMenu fxMenu;
FullScreenItemMenu reverbMenu;
FullScreenItemMenu chorusMenu;
FullScreenItemMenu overdriveMenu;

const int                kNumMainMenuItems = 6;
AbstractMenu::ItemConfig mainMenuItems[kNumMainMenuItems];
const int                kNumLfoMenuItems = 3;
AbstractMenu::ItemConfig lfoMenuItems[kNumLfoMenuItems];
const int                kNumLfoMapMenuItems = 7;
AbstractMenu::ItemConfig lfoMapMenuItems[kNumLfoMapMenuItems];
const int                kNumControlMapMenuItems = 4;
AbstractMenu::ItemConfig controlMapMenuItems[kNumControlMapMenuItems];
const int                kNumParamLimitMenuItems = 9;
AbstractMenu::ItemConfig paramLimitMenuItems[kNumParamLimitMenuItems];

const int                kNumPitchEnvMenuItems = 6;
AbstractMenu::ItemConfig pitchEnvMenuItems[kNumPitchEnvMenuItems];

const int                kNumXMapMenuItems = 15;
AbstractMenu::ItemConfig xMapMenuItems[kNumXMapMenuItems];
const int                kNumYMapMenuItems = 15;
AbstractMenu::ItemConfig yMapMenuItems[kNumYMapMenuItems];
const int                kNumZMapMenuItems = 15;
AbstractMenu::ItemConfig zMapMenuItems[kNumZMapMenuItems];
const int                kNumFxMenuItems = 4;
AbstractMenu::ItemConfig fxMenuItems[kNumFxMenuItems];
const int                kNumReverbMenuItems = 4;
AbstractMenu::ItemConfig reverbMenuItems[kNumReverbMenuItems];
const int                kNumChorusMenuItems = 6;
AbstractMenu::ItemConfig chorusMenuItems[kNumChorusMenuItems];
const int                kNumOverdriveMenuItems = 3;
AbstractMenu::ItemConfig overdriveMenuItems[kNumOverdriveMenuItems];

Encoder         encoder; 

float lfoFreq, lfoAmp, mod;
float ampAttack, ampDecay, ampSustain, ampRelease, amp;
float filterFreq, filterRes, filterAttack, filterDecay, filterSustain, filterRelease, filterEnvAmount, fltEnv;
float waveshape, syncFreq, PW;
float freq, velocity;
float pitchEnv, pitchXYZ;
float vol;

MappedFloatValue freqLowerLimit(50.f, 20000.f, 200.f, MappedFloatValue::Mapping::log, "Hz" ,0, false);
MappedFloatValue freqUpperLimit(50.f, 20000.f, 20000.f, MappedFloatValue::Mapping::log, "Hz" ,0, false);
MappedFloatValue lfoFreqLowerLimit(0.05f, 50.f, 0.1f, MappedFloatValue::Mapping::pow2, "Hz" , 2 ,false);
MappedFloatValue lfoFreqUpperLimit(0.05f, 50.f, 50.f, MappedFloatValue::Mapping::pow2, "Hz" , 2 ,false);
MappedFloatValue pitchLowerLimit(-24.f, 24.f, 0.f, MappedFloatValue::Mapping::lin, "st", 1, false);
MappedFloatValue pitchUpperLimit(-24.f, 24.f, 12.f, MappedFloatValue::Mapping::lin, "st", 1, false);
bool invFreqLimit;
bool invLfoFreqLimit;


MappedFloatValue reverbDryWet(0.0f, 100.0f, 35.f, MappedFloatValue::Mapping::lin, "%" ,0, false);
MappedFloatValue reverbFeedback(0.0f, 100.0f, 20.f, MappedFloatValue::Mapping::lin, "%" ,0, false);
MappedFloatValue reverbLp(500.f, 20000.f, 20000.f, MappedFloatValue::Mapping::log, "Hz" ,0, false);

MappedFloatValue chorusDryWet(0.0f, 100.0f, 20.f, MappedFloatValue::Mapping::lin, "%" ,0, false);
MappedFloatValue chorusLfoDepth(0.0f, 1.0f, 0.2f, MappedFloatValue::Mapping::lin, "", 2, false);
MappedFloatValue chorusLfoFreq(0.1f, 15.f, 1.f, MappedFloatValue::Mapping::lin, "Hz", 1, false);
MappedFloatValue chorusDelayMs(0.0f, 50.f, 0.5f, MappedFloatValue::Mapping::pow2, "ms", 1, false);
MappedFloatValue chorusFeedback(0.0f, 100.0f, 20.f, MappedFloatValue::Mapping::lin, "%" ,0, false);

MappedFloatValue pitchAttack(0.f, 5.f, 0.0f, MappedFloatValue::Mapping::pow2, "s", 2, false);
MappedFloatValue pitchDecay(0.f, 5.f, 0.2f, MappedFloatValue::Mapping::pow2, "s", 2, false);
MappedFloatValue pitchSustain(0.f, 1.f, 0.0f, MappedFloatValue::Mapping::lin, " ", 2, false);
MappedFloatValue pitchRelease(0.f, 2.f, 0.0f, MappedFloatValue::Mapping::pow2, "s", 2, false);
MappedFloatValue pitchEnvAmount(-24.f, 24.f, 0.f, MappedFloatValue::Mapping::lin, "st", 1, false);

MappedFloatValue overdriveDryWet(0.0f, 100.0f, 20.f, MappedFloatValue::Mapping::lin, "%" ,0, false);
MappedFloatValue overdriveDrive(0.f, 100.0f, 0.f, MappedFloatValue::Mapping::lin, "%", 0, false);

MappedFloatValue masterVolume(0.0f, 100.f, 50.f, MappedFloatValue::Mapping::lin, "%", 0, false);

const char* lfoShapeValues[]
    = {"Sine", "Tri", "Saw", "Ramp", "Square"};
MappedStringListValue lfoShapeValue(lfoShapeValues, 5, 0);
bool lfoMapCutoff;
bool lfoMapPW;
bool lfoMapSync;
bool lfoMapShape;
bool lfoMapAmp;
bool lfoMapPitch;

bool xMapCutoff, yMapCutoff, zMapCutoff;
bool xMapPW, yMapPW, zMapPW;
bool xMapSync, yMapSync, zMapSync;
bool xMapShape, yMapShape, zMapShape;
bool xMapLfoRate, yMapLfoRate, zMapLfoRate;
bool xMapLfoAmount, yMapLfoAmount, zMapLfoAmount;
bool xMapAmp, yMapAmp, zMapAmp;
bool xMapFltRes, yMapFltRes, zMapFltRes;
bool xMapFltAttack, yMapFltAttack, zMapFltAttack;
bool xMapFltDecay, yMapFltDecay, zMapFltDecay;
bool xMapFltEnv, yMapFltEnv, zMapFltEnv;
bool xMapPitch, yMapPitch, zMapPitch;
bool xMapRevFeedback, yMapRevFeedback, zMapRevFeedback;
bool xMapRevDryWet, yMapRevDryWet, zMapRevDryWet;
bool xMapOverdrive, yMapOverdrive, zMapOverdrive;

// Horizontal and vertical position for Trill sensor
float gTouchPosition[2] = { 0.0 , 0.0 };
// Touch size
float gTouchSize = 0.0;

float xPos, yPos;

float sigScope;

static MoogLadder flt;
static Oscillator lfo;
static ReverbSc  rev;
static Chorus chorus;
static Overdrive od;


uint8_t lfowWaveforms[5] = {
    Oscillator::WAVE_SIN,
    Oscillator::WAVE_TRI,
    Oscillator::WAVE_SAW,
    Oscillator::WAVE_RAMP,
    Oscillator::WAVE_SQUARE,
};

class Voice
{
  public:
    Voice() {}
    ~Voice() {}
    void Init(float samplerate)
    {
        active_ = false;
        variosc.Init(samplerate);
        adsr.Init(samplerate);
	    fltAdsr.Init(samplerate);
        pitchAdsr.Init(samplerate);
    }

    void UpdateEnvelopes()
    {
        adsr.SetTime(ADSR_SEG_ATTACK, ampAttack);
        adsr.SetTime(ADSR_SEG_DECAY , ampDecay);
        adsr.SetSustainLevel(ampSustain);
        adsr.SetTime(ADSR_SEG_RELEASE, ampRelease);

        //Update pitch envelope parameters
        pitchAdsr.SetTime(ADSR_SEG_ATTACK, pitchAttack);
        pitchAdsr.SetTime(ADSR_SEG_DECAY, pitchDecay);
        pitchAdsr.SetSustainLevel(pitchSustain);
        pitchAdsr.SetTime(ADSR_SEG_RELEASE, pitchRelease);

        //Update filter envelope parameters
        fltAdsr.SetTime(ADSR_SEG_ATTACK, filterAttack);
        fltAdsr.SetTime(ADSR_SEG_DECAY, filterDecay);
        fltAdsr.SetSustainLevel(filterSustain);
        fltAdsr.SetTime(ADSR_SEG_RELEASE, filterRelease);
    }

    float Process()
    {
        if(active_)
        {
            float sig, pitchMod;
			amp = adsr.Process(env_gate_);
            fltEnv = fltAdsr.Process(env_gate_);
            pitchEnv = pitchAdsr.Process(env_gate_);

            //handle pitch modulation sources
            if (lfoMapPitch == true){
            pitchMod = mod + pitchXYZ + (pitchEnv*pitchEnvAmount);
            variosc.SetFreq(mtof(note_ + pitchMod));
            }
            else if(pitchXYZ != 0 || pitchEnvAmount != 0){
                pitchMod = pitchXYZ + (pitchEnv*pitchEnvAmount);
                variosc.SetFreq(mtof(note_ + pitchMod));
            }
            
            
			if(!adsr.IsRunning())
                active_ = false;
            sig = variosc.Process();
            return (sig * (velocity_ / 127.f) * amp);
        }
        return 0.f;
    }

    void OnNoteOn(float note, float velocity)
    {
        note_     = note;
        velocity_ = velocity;
        variosc.SetFreq(mtof(note_));
        UpdateEnvelopes();
        active_   = true;
        env_gate_ = true;
    }

    void OnNoteOff() {env_gate_ = false;}

    void SetSyncFreq(float val) { variosc.SetSyncFreq(val); }
    void SetPW(float val) { variosc.SetPW(val); }
    void SetSync(bool val) { variosc.SetSync(val); }
    void SetWaveshape(float val) { variosc.SetWaveshape(val); }
 
    
    void SetFilterCutoff(float val) { flt.SetFreq(val); }
    void SetFilterRes(float val) { flt.SetRes(val); }

    inline bool  IsActive() const { return active_; }
    inline float GetNote() const { return note_; }

  private:
    VariableShapeOscillator variosc;
    Adsr       pitchAdsr, fltAdsr, adsr;
    float      note_, velocity_;
    bool       active_;
	bool 	   env_gate_;
};


template <size_t max_voices>
class VoiceManager
{
  public:
    VoiceManager() {}
    ~VoiceManager() {}

    void Init(float samplerate)
    {
        for(size_t i = 0; i < max_voices; i++)
        {
            voices[i].Init(samplerate);
        }
    }

    float Process()
    {
        float sum;
        sum = 0.f;
        for(size_t i = 0; i < max_voices; i++)
        {
            sum += voices[i].Process();
        }
        return sum;
    }

    void OnNoteOn(float notenumber, float velocity)
    {
        Voice *v = FindFreeVoice();
        if(v == NULL)
            return;
        v->OnNoteOn(notenumber, velocity);
    }

    void OnNoteOff(float notenumber, float velocity)
    {   if(sustainNote == false){
            for(size_t i = 0; i < max_voices; i++)
            {
                Voice *v = &voices[i];
                if(v->IsActive() && v->GetNote() == notenumber)
                {
                    v->OnNoteOff();
                }
            }}
        else{}
    }

    void FreeAllVoices()
    {
        for(size_t i = 0; i < max_voices; i++)
        {
            voices[i].OnNoteOff();
        }
    }


    void SetSyncFreq(float all_val)
    {
        for(size_t i = 0; i < max_voices; i++)
        {
            voices[i].SetSyncFreq(all_val);
        }
    }

    void SetPW(float all_val)
    {
        for(size_t i = 0; i < max_voices; i++)
        {
            voices[i].SetPW(all_val);
        }
    }
    
    void SetSync(bool all_val)
    {
        for(size_t i = 0; i < max_voices; i++)
        {
            voices[i].SetSync(all_val);
        }
    }

    void SetWaveshape(float all_val)
    {
        for(size_t i = 0; i < max_voices; i++)
        {
            voices[i].SetWaveshape(all_val);
        }
    }

    void SetFilterCutoff(float all_val)
    {
        for(size_t i = 0; i < max_voices; i++)
        {
            voices[i].SetFilterCutoff(all_val);
        }
    }

    void SetFilterRes(float all_val)
    {
        for(size_t i = 0; i < max_voices; i++)
        {
            voices[i].SetFilterRes(all_val);
        }
    }

  private:
    Voice  voices[max_voices];
    Voice *FindFreeVoice()
    {
        Voice *v = NULL;
        for(size_t i = 0; i < max_voices; i++)
        {
            if(!voices[i].IsActive())
            {
                v = &voices[i];
                break;
            }
        }
        return v;
    }
};

static VoiceManager<16> voice_handler;

void UpdateButtons(){
    //Debounce buttons
    panicButton.Debounce();
    functionButton.Debounce();
    backButton.Debounce();
    encoder.Debounce();

    //Update button parameters
    if(panicButton.RisingEdge() )
    {
        voice_handler.FreeAllVoices();
    }

}

void UpdateADC(){

    //Maped so z goes from 0 to 1
    float zPos = fmap(hw.adc.GetFloat(0), -0.34f, 1.20f, Mapping::LINEAR);
    //zPos = zPos *
    
    //Update parameters
    ampAttack = fmap(hw.adc.GetMuxFloat(1,0), 0.f, 5.f, Mapping::EXP);
    ampDecay = fmap(hw.adc.GetMuxFloat(1,1), 0.f, 5.f, Mapping::EXP);
    ampSustain = hw.adc.GetMuxFloat(1,2);
    ampRelease = fmap(hw.adc.GetMuxFloat(1,3), 0.f, 2.f, Mapping::EXP);

    if (xMapShape == false && yMapShape == false && zMapShape == false){
        waveshape = hw.adc.GetMuxFloat(1,4);}
        else if (xMapShape == true)
        {
            waveshape = xPos;
        }
        else if (yMapShape == true)
        {
            waveshape = yPos;
        }
        else if (zMapShape == true)
        {
            waveshape = zPos;
        }

    if (xMapSync == false && yMapSync == false && zMapSync == false){
        syncFreq = mtof(hw.adc.GetMuxFloat(1,5) * 127);}
        else if (xMapSync == true)
        {
            syncFreq = mtof(xPos * 127);
        }
        else if (yMapSync == true)
        {
            syncFreq = mtof(yPos * 127);
        }
        else if (zMapSync == true)
        {
            syncFreq = mtof(zPos * 127);
        }

    if (xMapPW == false && yMapPW == false && zMapPW == false){
        PW = hw.adc.GetMuxFloat(1,6);}
        else if (xMapPW == true)
        {
            PW = xPos;
        }
        else if (yMapPW == true)
        {
            PW = yPos;
        }
        else if (zMapPW == true)
        {
            PW = zPos;
        }

    if (xMapLfoRate == false && yMapLfoRate == false && zMapLfoRate == false){
        lfoFreq = fmap(hw.adc.GetMuxFloat(1,7), lfoFreqLowerLimit , lfoFreqUpperLimit, Mapping::EXP);}
        else if (xMapLfoRate == true)
        {
            lfoFreq = fmap(xPos , lfoFreqLowerLimit, lfoFreqUpperLimit, Mapping::EXP);
        }
        else if (yMapLfoRate == true)
        {
            lfoFreq = fmap(yPos , lfoFreqLowerLimit, lfoFreqUpperLimit, Mapping::EXP);
        }
        else if (zMapLfoRate == true)
        {
            lfoFreq = fmap(zPos , lfoFreqLowerLimit, lfoFreqUpperLimit, Mapping::EXP);
        }

    if (xMapLfoAmount == false && yMapLfoAmount == false && zMapLfoAmount == false){
        lfoAmp = hw.adc.GetMuxFloat(2,0);}
        else if (xMapLfoAmount == true)
        {
            lfoAmp = xPos;
        }
        else if (yMapLfoAmount == true)
        {
            lfoAmp = yPos;
        }
        else if (zMapLfoAmount == true)
        {
            lfoAmp = zPos;
        }
    
    if (freqUpperLimit > freqLowerLimit || invFreqLimit == false){
        if (xMapCutoff == false && yMapCutoff == false && zMapCutoff == false){
            filterFreq = fmap(hw.adc.GetMuxFloat(2,1), freqLowerLimit, freqUpperLimit, Mapping::LOG);}
            else if (xMapCutoff == true)
            {
                filterFreq = fmap(xPos, freqLowerLimit, freqUpperLimit, Mapping::LOG);
            }
            else if (yMapCutoff == true)
            {
                filterFreq = fmap(yPos, freqLowerLimit, freqUpperLimit, Mapping::LOG);
            }
            else if (zMapCutoff == true)
            {
                filterFreq = fmap(zPos, freqLowerLimit, freqUpperLimit, Mapping::LOG);
            }
    }
    else{
        float invL = freqUpperLimit;
        float invU = freqLowerLimit;
        if (xMapCutoff == false && yMapCutoff == false && zMapCutoff == false){
            filterFreq = fmap(hw.adc.GetMuxFloat(2,1), invL, invU, Mapping::LOG);}
            else if (xMapCutoff == true)
            {
                filterFreq = fmap(xPos, invL, invU, Mapping::LOG);
            }
            else if (yMapCutoff == true)
            {
                filterFreq = fmap(yPos, invL, invU, Mapping::LOG);
            }
            else if (zMapCutoff == true)
            {
                filterFreq = fmap(zPos, invL, invU, Mapping::LOG);
            }

    }
    //filterRes = fmap(hw.adc.GetMuxFloat(2,2), 0, 0.9, Mapping::LINEAR);
    if (xMapFltRes == false && yMapFltRes == false && zMapFltRes == false)
        filterRes = fmap(hw.adc.GetMuxFloat(2,2), 0, 0.9, Mapping::LINEAR);
        else if (xMapFltRes == true)
        {
            filterRes = fmap(xPos, 0, 0.9f, Mapping::LINEAR);
        }
        else if (yMapFltRes == true)
        {
            filterRes = fmap(yPos, 0, 0.9f, Mapping::LINEAR);
        }
        else if (zMapFltRes == true)
        {
            filterRes = fmap(zPos, 0, 0.9f, Mapping::LINEAR);
        }
    
    if (xMapFltAttack == false && yMapFltAttack == false && zMapFltAttack == false)
        filterAttack = fmap(hw.adc.GetMuxFloat(2,3), 0, 5, Mapping::EXP);
        else if (xMapFltAttack == true)
        {
            filterAttack = fmap(xPos, 0.f, 5.f, Mapping::EXP);
        }
        else if (yMapFltAttack == true)
        {
            filterAttack = fmap(yPos, 0.f, 5.f, Mapping::EXP);
        }
        else if (zMapFltAttack == true)
        {
            filterAttack = fmap(zPos, 0.f, 5.f, Mapping::EXP);
        }

    if (xMapFltDecay == false && yMapFltDecay == false && zMapFltDecay == false)
        filterDecay = fmap(hw.adc.GetMuxFloat(2,4), 0.f, 5.f, Mapping::EXP);
        else if (xMapFltDecay == true)
        {
            filterDecay = fmap(xPos, 0.f, 5.f, Mapping::EXP);
        }
        else if (yMapFltDecay == true)
        {
            filterDecay = fmap(yPos, 0.f, 5.f, Mapping::EXP);
        }
        else if (zMapFltDecay == true)
        {
            filterDecay = fmap(zPos, 0.f, 5.f, Mapping::EXP);
        }

    filterSustain = hw.adc.GetMuxFloat(2,5);
    filterRelease = fmap(hw.adc.GetMuxFloat(2,6), 0.f, 2.f, Mapping::EXP);

    if (xMapFltEnv == false && yMapFltEnv == false && zMapFltEnv == false)
        filterEnvAmount = fmap(hw.adc.GetMuxFloat(2,7), 0, 10000, Mapping::EXP);
        else if (xMapFltEnv == true)
        {
            filterEnvAmount = fmap(xPos, 0, 10000, Mapping::EXP);
        }
        else if (yMapFltEnv == true)
        {
            filterEnvAmount = fmap(yPos, 0, 10000, Mapping::EXP);
        }
        else if (zMapFltEnv == true)
        {
            filterEnvAmount = fmap(zPos, 0, 10000, Mapping::EXP);
        }
        
        if (xMapPitch == true)
        {
            pitchXYZ = fmap(xPos, pitchLowerLimit, pitchUpperLimit, Mapping::LINEAR);
        }
        else if (yMapPitch == true)
        {
            pitchXYZ = fmap(yPos, pitchLowerLimit, pitchUpperLimit, Mapping::LINEAR);
        }
        else if (zMapPitch == true)
        {
            pitchXYZ = fmap(zPos, pitchLowerLimit, pitchUpperLimit, Mapping::LINEAR);
        }    

    if (xMapAmp == false && yMapAmp == false && zMapAmp == false)
        vol =1;
        else if (xMapAmp == true){
            vol = xPos;
        }
        else if (yMapAmp == true){
            vol = yPos;
        }
        else if (zMapAmp == true){
            vol = zPos;
        }

    //Update LFO
    lfo.SetAmp(lfoAmp);
    lfo.SetFreq(lfoFreq);


    //Update oscillator parameters
	voice_handler.SetSync(true);

    if (lfoMapShape == true)
        voice_handler.SetWaveshape(waveshape * mod);
    else{
        voice_handler.SetWaveshape(waveshape);
    }
    if (lfoMapSync == true){
	    voice_handler.SetSyncFreq(syncFreq * ((mod*2.f)+ 3));}
    else{
        voice_handler.SetSyncFreq(syncFreq * 3);
    }
    if (lfoMapPW == true){
	    voice_handler.SetPW(mod*PW);}
    else{
        voice_handler.SetPW(PW);
    }


    if (lfoMapAmp == true){
	    vol = vol * mod;}
    else{
    }

   

    //Update overdriveParameters
    if(xMapOverdrive == true){
        overdriveDrive = xPos*100.f;
    }
    else if(yMapOverdrive == true){
        overdriveDrive = yPos*100.f;
    }
    else if(zMapOverdrive == true){
        overdriveDrive = zPos*100.f;
    }
    od.SetDrive(overdriveDrive/100.f);

    //Update reverb parameters
    if(xMapRevFeedback == true){
        reverbFeedback = xPos*100.f;
    }
    else if(yMapRevFeedback == true){
        reverbFeedback = yPos*100.f;
    }
    else if(zMapRevFeedback == true){
        reverbFeedback = zPos*100.f;
    }
    rev.SetFeedback(reverbFeedback/100.f);
    rev.SetLpFreq(reverbLp);

    //Update chorus parameters
    chorus.SetLfoDepth(chorusLfoDepth);
    chorus.SetLfoFreq(chorusLfoFreq);
    chorus.SetDelayMs(chorusDelayMs);
    chorus.SetFeedback(chorusFeedback/100.f);
}

void GenerateUiEvents()
{
    if(encoder.RisingEdge())
        eventQueue.AddButtonPressed(0, 1);

    if(encoder.FallingEdge())
        eventQueue.AddButtonReleased(0);

    if(backButton.RisingEdge())
        eventQueue.AddButtonPressed(1,1);

    if(backButton.FallingEdge())
        eventQueue.AddButtonReleased(1);

    if(functionButton.RisingEdge())
        eventQueue.AddButtonPressed(2,1);

    if(functionButton.FallingEdge())
        eventQueue.AddButtonReleased(2);
    

    const auto increments = encoder.Increment();
    if(increments != 0)
        eventQueue.AddEncoderTurned(0, increments, 12);
}

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    float odOut, choL, choR, outL, outR;
    float sum = 0.f;
    UpdateButtons();
    UpdateADC();
    GenerateUiEvents();
    int lfoShape = DSY_CLAMP(lfoShapeValue.GetIndex(), 0, 5);
    lfo.SetWaveform(lfoShape);
    if (filterEnvAmount < 5.f){
        if (lfoMapCutoff == true){
            voice_handler.SetFilterCutoff(filterFreq+(mod*500.f));}
        else{
            voice_handler.SetFilterCutoff(filterFreq);
            }
        }
        else{
            if (lfoMapCutoff == true){
                voice_handler.SetFilterCutoff(filterFreq+(mod*500.f) + (filterEnvAmount * fltEnv));}
            else{
                voice_handler.SetFilterCutoff(filterFreq + (filterEnvAmount * fltEnv));
            }
        }
            voice_handler.SetFilterRes(filterRes);
    
    for(size_t i = 0; i < size; i += 2)
    { 
        mod = lfo.Process();
        sum        = 0.f;
        sum        = flt.Process(voice_handler.Process() * 0.5f) * vol;
       odOut = (overdriveDryWet/100.f) * od.Process(sum) + (1.f - (overdriveDryWet/100.f))*sum;

        chorus.Process(odOut);
        choL = (chorusDryWet/100.f) * chorus.GetLeft() + (1.f - (chorusDryWet/100.f)) * odOut ;
        choR = (chorusDryWet/100.f) * chorus.GetRight() + (1.f - (chorusDryWet/100.f)) * odOut ;

        rev.Process(choL, choR, &outL, &outR);
        outL = (reverbDryWet/100.f) * outL + (1.f - (reverbDryWet/100.f)) * choL;
        outR = (reverbDryWet/100.f) * outR + (1.f - (reverbDryWet/100.f)) * choR;

        out[i]     = outL * (masterVolume/100.f);
        out[i + 1] = outR * (masterVolume/100.f);
    }
}



// Typical Switch case for Message Type.
void HandleMidiMessage(MidiEvent m)
{
    switch(m.type)
    {
        case NoteOn:
        {
            NoteOnEvent p = m.AsNoteOn();
            // Note Off can come in as Note On w/ 0 Velocity
            if(p.velocity == 0.f)
            {
                voice_handler.OnNoteOff(p.note, p.velocity);
            }
            else
            {
                voice_handler.OnNoteOn(p.note, p.velocity);
            }
        }
        break;
        case NoteOff:
        {
            NoteOnEvent p = m.AsNoteOn();
            voice_handler.OnNoteOff(p.note, p.velocity);
        }
        break;
        case ControlChange:
        {
            ControlChangeEvent p = m.AsControlChange();
            switch(p.control_number)
            {
                //sustain pedal cc is 64
                case 64:
                    if((float)p.value == 127){
                        sustainNote = true;
                    }
                    else{
                        sustainNote = false;
                        voice_handler.FreeAllVoices();
                    }
                    break;
                default: break;
            }
            break;
        }       
        default: break;
    }
}

//Initialise the OLED display
void InitDisplay()
{
    OledDisplay<SSD130x4WireSpi128x64Driver>::Config display_config;

    display_config.driver_config.transport_config.pin_config.dc
        = hw.GetPin(PIN_OLED_DC);
    display_config.driver_config.transport_config.pin_config.reset
        = hw.GetPin(PIN_OLED_RESET);

    display.Init(display_config);
}



using OledDisplayType = decltype(display);

void FlushCanvas(const daisy::UiCanvasDescriptor& canvasDescriptor)
{
    if(canvasDescriptor.id_ == 0)
    {
        OledDisplayType& display
            = *((OledDisplayType*)(canvasDescriptor.handle_));
        display.Update();
    }
}
void ClearCanvas(const daisy::UiCanvasDescriptor& canvasDescriptor)
{
    if(canvasDescriptor.id_ == 0)
    {
        OledDisplayType& display
            = *((OledDisplayType*)(canvasDescriptor.handle_));
        display.Fill(false);
    }
}



void InitUiPages()
{
    // The main menu
    mainMenuItems[0].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    mainMenuItems[0].text = "LFO";
    mainMenuItems[0].asOpenUiPageItem.pageToOpen = &lfoMenu;

    mainMenuItems[1].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    mainMenuItems[1].text = "Map XYZ";
    mainMenuItems[1].asOpenUiPageItem.pageToOpen = &controlMapMenu;

    mainMenuItems[2].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    mainMenuItems[2].text = "Param Lim";
    mainMenuItems[2].asOpenUiPageItem.pageToOpen = &paramLimitMenu;

    mainMenuItems[3].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    mainMenuItems[3].text = "FX";
    mainMenuItems[3].asOpenUiPageItem.pageToOpen = &fxMenu;

    mainMenuItems[4].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    mainMenuItems[4].text = "Pitch Env";
    mainMenuItems[4].asOpenUiPageItem.pageToOpen = &pitchEnvMenu;

    mainMenuItems[5].type = daisy::AbstractMenu::ItemType::valueItem;
    mainMenuItems[5].text = "Volume";
    mainMenuItems[5].asMappedValueItem.valueToModify = &masterVolume;

    mainMenu.Init(mainMenuItems, kNumMainMenuItems);

    // The "lfo" menu
    lfoMenuItems[0].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    lfoMenuItems[0].text = "Map LFO";
    lfoMenuItems[0].asOpenUiPageItem.pageToOpen = &lfoMapMenu;

    lfoMenuItems[1].type = daisy::AbstractMenu::ItemType::valueItem;
    lfoMenuItems[1].text = "LFO shape";
    lfoMenuItems[1].asMappedValueItem.valueToModify = &lfoShapeValue;
    

    lfoMenuItems[2].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    lfoMenuItems[2].text = "Back";

    lfoMenu.Init(lfoMenuItems, kNumLfoMenuItems);

    // The "lfo map" menu
    lfoMapMenuItems[0].type = daisy::AbstractMenu::ItemType::checkboxItem;
    lfoMapMenuItems[0].text = "Cutoff";
    lfoMapMenuItems[0].asCheckboxItem.valueToModify = &lfoMapCutoff;

    lfoMapMenuItems[1].type = daisy::AbstractMenu::ItemType::checkboxItem;
    lfoMapMenuItems[1].text = "PW";
    lfoMapMenuItems[1].asCheckboxItem.valueToModify = &lfoMapPW;

    lfoMapMenuItems[2].type = daisy::AbstractMenu::ItemType::checkboxItem;
    lfoMapMenuItems[2].text = "Sync freq";
    lfoMapMenuItems[2].asCheckboxItem.valueToModify = &lfoMapSync;

    lfoMapMenuItems[3].type = daisy::AbstractMenu::ItemType::checkboxItem;
    lfoMapMenuItems[3].text = "Waveshape";
    lfoMapMenuItems[3].asCheckboxItem.valueToModify = &lfoMapShape;

    lfoMapMenuItems[4].type = daisy::AbstractMenu::ItemType::checkboxItem;
    lfoMapMenuItems[4].text = "Amp";
    lfoMapMenuItems[4].asCheckboxItem.valueToModify = &lfoMapAmp;

    lfoMapMenuItems[5].type = daisy::AbstractMenu::ItemType::checkboxItem;
    lfoMapMenuItems[5].text = "Pitch";
    lfoMapMenuItems[5].asCheckboxItem.valueToModify = &lfoMapPitch;

    lfoMapMenuItems[6].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    lfoMapMenuItems[6].text = "Back";

    lfoMapMenu.Init(lfoMapMenuItems, kNumLfoMapMenuItems);

    // The "xyz map edit" menu
    controlMapMenuItems[0].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    controlMapMenuItems[0].text = "X map";
    controlMapMenuItems[0].asOpenUiPageItem.pageToOpen = &xMapMenu;

    controlMapMenuItems[1].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    controlMapMenuItems[1].text = "Y map";
    controlMapMenuItems[1].asOpenUiPageItem.pageToOpen = &yMapMenu;

    controlMapMenuItems[2].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    controlMapMenuItems[2].text = "Z map";
    controlMapMenuItems[2].asOpenUiPageItem.pageToOpen = &zMapMenu;
    
    controlMapMenuItems[3].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    controlMapMenuItems[3].text = "Back";

    controlMapMenu.Init(controlMapMenuItems, kNumControlMapMenuItems);

    // The "parameter limit" menu
    paramLimitMenuItems[0].type = daisy::AbstractMenu::ItemType::valueItem;
    paramLimitMenuItems[0].text = "cutoff lo";
    paramLimitMenuItems[0].asMappedValueItem.valueToModify =  &freqLowerLimit;

    paramLimitMenuItems[1].type = daisy::AbstractMenu::ItemType::valueItem;
    paramLimitMenuItems[1].text = "cutoff hi";
    paramLimitMenuItems[1].asMappedValueItem.valueToModify =  &freqUpperLimit;

    paramLimitMenuItems[2].type = daisy::AbstractMenu::ItemType::checkboxItem;
    paramLimitMenuItems[2].text = "inv f lim";
    paramLimitMenuItems[2].asCheckboxItem.valueToModify =  &invFreqLimit;

    paramLimitMenuItems[3].type = daisy::AbstractMenu::ItemType::valueItem;
    paramLimitMenuItems[3].text = "lfo lo";
    paramLimitMenuItems[3].asMappedValueItem.valueToModify =  &lfoFreqLowerLimit;

    paramLimitMenuItems[4].type = daisy::AbstractMenu::ItemType::valueItem;
    paramLimitMenuItems[4].text = "lfo hi";
    paramLimitMenuItems[4].asMappedValueItem.valueToModify =  &lfoFreqUpperLimit;

    paramLimitMenuItems[5].type = daisy::AbstractMenu::ItemType::checkboxItem;
    paramLimitMenuItems[5].text = "inv l lim";
    paramLimitMenuItems[5].asCheckboxItem.valueToModify =  &invLfoFreqLimit;

    paramLimitMenuItems[6].type = daisy::AbstractMenu::ItemType::valueItem;
    paramLimitMenuItems[6].text = "pitch lo";
    paramLimitMenuItems[6].asMappedValueItem.valueToModify =  &pitchLowerLimit;

    paramLimitMenuItems[7].type = daisy::AbstractMenu::ItemType::valueItem;
    paramLimitMenuItems[7].text = "pitch hi";
    paramLimitMenuItems[7].asMappedValueItem.valueToModify =  &pitchUpperLimit;
    
    paramLimitMenuItems[8].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    paramLimitMenuItems[8].text = "Back";

    paramLimitMenu.Init(paramLimitMenuItems, kNumParamLimitMenuItems);

 
    // The "pitch envelope" menu
    pitchEnvMenuItems[0].type = daisy::AbstractMenu::ItemType::valueItem;
    pitchEnvMenuItems[0].text = "Attack";
    pitchEnvMenuItems[0].asMappedValueItem.valueToModify =  &pitchAttack;

    pitchEnvMenuItems[1].type = daisy::AbstractMenu::ItemType::valueItem;
    pitchEnvMenuItems[1].text = "Decay";
    pitchEnvMenuItems[1].asMappedValueItem.valueToModify =  &pitchDecay;

    pitchEnvMenuItems[2].type = daisy::AbstractMenu::ItemType::valueItem;
    pitchEnvMenuItems[2].text = "Sustain";
    pitchEnvMenuItems[2].asMappedValueItem.valueToModify =  &pitchSustain;

    pitchEnvMenuItems[3].type = daisy::AbstractMenu::ItemType::valueItem;
    pitchEnvMenuItems[3].text = "Release";
    pitchEnvMenuItems[3].asMappedValueItem.valueToModify =  &pitchRelease;

    pitchEnvMenuItems[4].type = daisy::AbstractMenu::ItemType::valueItem;
    pitchEnvMenuItems[4].text = "Env Amount";
    pitchEnvMenuItems[4].asMappedValueItem.valueToModify =  &pitchEnvAmount;

    pitchEnvMenuItems[5].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    pitchEnvMenuItems[5].text = "Back";

    pitchEnvMenu.Init(pitchEnvMenuItems, kNumPitchEnvMenuItems);

    // The "x map" menu
    xMapMenuItems[0].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[0].text = "Cutoff";
    xMapMenuItems[0].asCheckboxItem.valueToModify = &xMapCutoff;

    xMapMenuItems[1].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[1].text = "PW";
    xMapMenuItems[1].asCheckboxItem.valueToModify = &xMapPW;

    xMapMenuItems[2].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[2].text = "Sync freq";
    xMapMenuItems[2].asCheckboxItem.valueToModify = &xMapSync;

    xMapMenuItems[3].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[3].text = "Waveshape";
    xMapMenuItems[3].asCheckboxItem.valueToModify = &xMapShape;

    xMapMenuItems[4].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[4].text = "LFO Rate";
    xMapMenuItems[4].asCheckboxItem.valueToModify = &xMapLfoRate;

    xMapMenuItems[5].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[5].text = "LFO Amount";
    xMapMenuItems[5].asCheckboxItem.valueToModify = &xMapLfoAmount;

    xMapMenuItems[6].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[6].text = "Pitch";
    xMapMenuItems[6].asCheckboxItem.valueToModify = &xMapPitch;

    xMapMenuItems[7].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[7].text = "Amp";
    xMapMenuItems[7].asCheckboxItem.valueToModify = &xMapAmp;

    xMapMenuItems[8].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[8].text = "Flt Res";
    xMapMenuItems[8].asCheckboxItem.valueToModify = &xMapFltRes;

    xMapMenuItems[9].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[9].text = "Flt Attack";
    xMapMenuItems[9].asCheckboxItem.valueToModify = &xMapFltAttack;

    xMapMenuItems[10].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[10].text = "Flt Decay";
    xMapMenuItems[10].asCheckboxItem.valueToModify = &xMapFltDecay;

    xMapMenuItems[11].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[11].text = "Flt Env";
    xMapMenuItems[11].asCheckboxItem.valueToModify = &xMapFltEnv;

    xMapMenuItems[12].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[12].text = "Overdrive";
    xMapMenuItems[12].asCheckboxItem.valueToModify = &xMapOverdrive;

    xMapMenuItems[13].type = daisy::AbstractMenu::ItemType::checkboxItem;
    xMapMenuItems[13].text = "Rev FB";
    xMapMenuItems[13].asCheckboxItem.valueToModify = &xMapRevFeedback;

    xMapMenuItems[14].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    xMapMenuItems[14].text = "Back";

    xMapMenu.Init(xMapMenuItems, kNumXMapMenuItems);

    // The "y map" menu
    yMapMenuItems[0].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[0].text = "Cutoff";
    yMapMenuItems[0].asCheckboxItem.valueToModify = &yMapCutoff;

    yMapMenuItems[1].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[1].text = "PW";
    yMapMenuItems[1].asCheckboxItem.valueToModify = &yMapPW;

    yMapMenuItems[2].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[2].text = "Sync freq";
    yMapMenuItems[2].asCheckboxItem.valueToModify = &yMapSync;

    yMapMenuItems[3].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[3].text = "Waveshape";
    yMapMenuItems[3].asCheckboxItem.valueToModify = &yMapShape;

    yMapMenuItems[4].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[4].text = "LFO Rate";
    yMapMenuItems[4].asCheckboxItem.valueToModify = &yMapLfoRate;

    yMapMenuItems[5].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[5].text = "LFO Amount";
    yMapMenuItems[5].asCheckboxItem.valueToModify = &yMapLfoAmount;

    yMapMenuItems[6].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[6].text = "Pitch";
    yMapMenuItems[6].asCheckboxItem.valueToModify = &yMapPitch;

    yMapMenuItems[7].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[7].text = "Amp";
    yMapMenuItems[7].asCheckboxItem.valueToModify = &yMapAmp;

    yMapMenuItems[8].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[8].text = "Flt Res";
    yMapMenuItems[8].asCheckboxItem.valueToModify = &yMapFltRes;

    yMapMenuItems[9].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[9].text = "Flt Attack";
    yMapMenuItems[9].asCheckboxItem.valueToModify = &yMapFltAttack;

    yMapMenuItems[10].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[10].text = "Flt Decay";
    yMapMenuItems[10].asCheckboxItem.valueToModify = &yMapFltDecay;

    yMapMenuItems[11].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[11].text = "Flt Env";
    yMapMenuItems[11].asCheckboxItem.valueToModify = &yMapFltEnv;

    yMapMenuItems[12].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[12].text = "Overdrive";
    yMapMenuItems[12].asCheckboxItem.valueToModify = &yMapOverdrive;

    yMapMenuItems[13].type = daisy::AbstractMenu::ItemType::checkboxItem;
    yMapMenuItems[13].text = "Rev FB";
    yMapMenuItems[13].asCheckboxItem.valueToModify = &yMapRevFeedback;


    yMapMenuItems[14].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    yMapMenuItems[14].text = "Back";

    yMapMenu.Init(yMapMenuItems, kNumYMapMenuItems);

    // The "z map" menu
    zMapMenuItems[0].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[0].text = "Cutoff";
    zMapMenuItems[0].asCheckboxItem.valueToModify = &zMapCutoff;

    zMapMenuItems[1].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[1].text = "PW";
    zMapMenuItems[1].asCheckboxItem.valueToModify = &zMapPW;

    zMapMenuItems[2].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[2].text = "Sync freq";
    zMapMenuItems[2].asCheckboxItem.valueToModify = &zMapSync;

    zMapMenuItems[3].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[3].text = "Waveshape";
    zMapMenuItems[3].asCheckboxItem.valueToModify = &zMapShape;

    zMapMenuItems[4].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[4].text = "LFO Rate";
    zMapMenuItems[4].asCheckboxItem.valueToModify = &zMapLfoRate;

    zMapMenuItems[5].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[5].text = "LFO Amount";
    zMapMenuItems[5].asCheckboxItem.valueToModify = &zMapLfoAmount;

    zMapMenuItems[6].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[6].text = "Pitch";
    zMapMenuItems[6].asCheckboxItem.valueToModify = &zMapPitch;

    zMapMenuItems[7].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[7].text = "Amp";
    zMapMenuItems[7].asCheckboxItem.valueToModify = &zMapAmp;

    zMapMenuItems[8].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[8].text = "Flt Res";
    zMapMenuItems[8].asCheckboxItem.valueToModify = &zMapFltRes;

    zMapMenuItems[9].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[9].text = "Flt Attack";
    zMapMenuItems[9].asCheckboxItem.valueToModify = &zMapFltAttack;

    zMapMenuItems[10].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[10].text = "Flt Decay";
    zMapMenuItems[10].asCheckboxItem.valueToModify = &zMapFltDecay;

    zMapMenuItems[11].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[11].text = "Flt Env";
    zMapMenuItems[11].asCheckboxItem.valueToModify = &zMapFltEnv;

    zMapMenuItems[12].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[12].text = "Overdrive";
    zMapMenuItems[12].asCheckboxItem.valueToModify = &zMapOverdrive;

    zMapMenuItems[13].type = daisy::AbstractMenu::ItemType::checkboxItem;
    zMapMenuItems[13].text = "Rev FB";
    zMapMenuItems[13].asCheckboxItem.valueToModify = &zMapRevFeedback;

    zMapMenuItems[14].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    zMapMenuItems[14].text = "Back";

    zMapMenu.Init(zMapMenuItems, kNumZMapMenuItems);

    // The "fx edit" menu
    fxMenuItems[0].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    fxMenuItems[0].text = "Reverb";
    fxMenuItems[0].asOpenUiPageItem.pageToOpen = &reverbMenu;

    fxMenuItems[1].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    fxMenuItems[1].text = "Chorus";
    fxMenuItems[1].asOpenUiPageItem.pageToOpen = &chorusMenu;

    fxMenuItems[2].type = daisy::AbstractMenu::ItemType::openUiPageItem;
    fxMenuItems[2].text = "Overdrive";
    fxMenuItems[2].asOpenUiPageItem.pageToOpen = &overdriveMenu;

    fxMenuItems[3].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    fxMenuItems[3].text = "Back";

    fxMenu.Init(fxMenuItems, kNumFxMenuItems);

    // The "reverb" menu
    reverbMenuItems[0].type = daisy::AbstractMenu::ItemType::valueItem;
    reverbMenuItems[0].text = "Dry/Wet";
    reverbMenuItems[0].asMappedValueItem.valueToModify = &reverbDryWet;
    
    reverbMenuItems[1].type = daisy::AbstractMenu::ItemType::valueItem;
    reverbMenuItems[1].text = "feedback";
    reverbMenuItems[1].asMappedValueItem.valueToModify = &reverbFeedback;

    reverbMenuItems[2].type = daisy::AbstractMenu::ItemType::valueItem;
    reverbMenuItems[2].text = "LP";
    reverbMenuItems[2].asMappedValueItem.valueToModify = &reverbLp;

    reverbMenuItems[3].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    reverbMenuItems[3].text = "Back";

    reverbMenu.Init(reverbMenuItems, kNumReverbMenuItems);

    // The "chorus" menu
    chorusMenuItems[0].type = daisy::AbstractMenu::ItemType::valueItem;
    chorusMenuItems[0].text = "Dry/Wet";
    chorusMenuItems[0].asMappedValueItem.valueToModify = &chorusDryWet;

    chorusMenuItems[1].type = daisy::AbstractMenu::ItemType::valueItem;
    chorusMenuItems[1].text = "LFO am";
    chorusMenuItems[1].asMappedValueItem.valueToModify = &chorusLfoDepth;

    chorusMenuItems[2].type = daisy::AbstractMenu::ItemType::valueItem;
    chorusMenuItems[2].text = "LFO fr";
    chorusMenuItems[2].asMappedValueItem.valueToModify = &chorusLfoFreq;

    chorusMenuItems[3].type = daisy::AbstractMenu::ItemType::valueItem;
    chorusMenuItems[3].text = "Delay";
    chorusMenuItems[3].asMappedValueItem.valueToModify = &chorusDelayMs;

    chorusMenuItems[4].type = daisy::AbstractMenu::ItemType::valueItem;
    chorusMenuItems[4].text = "feedback";
    chorusMenuItems[4].asMappedValueItem.valueToModify = &chorusFeedback;

    chorusMenuItems[5].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    chorusMenuItems[5].text = "Back";

    chorusMenu.Init(chorusMenuItems, kNumChorusMenuItems);

    // The "overdrive" menu
    overdriveMenuItems[0].type = daisy::AbstractMenu::ItemType::valueItem;
    overdriveMenuItems[0].text = "Dry/Wet";
    overdriveMenuItems[0].asMappedValueItem.valueToModify = &overdriveDryWet;
    
    overdriveMenuItems[1].type = daisy::AbstractMenu::ItemType::valueItem;
    overdriveMenuItems[1].text = "Drive";
    overdriveMenuItems[1].asMappedValueItem.valueToModify = &overdriveDrive;

    overdriveMenuItems[2].type = daisy::AbstractMenu::ItemType::closeMenuItem;
    overdriveMenuItems[2].text = "Back";

    overdriveMenu.Init(overdriveMenuItems, kNumOverdriveMenuItems);
}

void InitTrill(){
    //Init I2C and Trill sensor
    static constexpr I2CHandle::Config i2c_config = {
        I2CHandle::Config::Peripheral::I2C_1,
        {{DSY_GPIOB, 8},
         {DSY_GPIOB, 9}}, 
        I2CHandle::Config::Speed::I2C_400KHZ};
    //Trill square I2C adress
    uint8_t addr = 0b00101000; // 0x28 - TRILL SQUARE

    I2CHandle i2c;
    i2c.Init(i2c_config);

    touchSensor.setup(i2c, Trill::device::SQUARE, addr);
}

void UpdateTrill(){
    unsigned int touches;
    touchSensor.readI2C();
    gTouchSize = touchSensor.compoundTouchSize();

    gTouchPosition[0] = touchSensor.compoundTouchHorizontalLocation();
    gTouchPosition[1] = touchSensor.compoundTouchLocation();
    touches = touchSensor.getNumTouches();

    if (touches != 0){
        xPos = gTouchPosition[0];
    }

    if (touches != 0){
        yPos = gTouchPosition[1];
    }
}


int main(void)
{
    hw.Configure();
    hw.Init();
    MidiUsbHandler::Config midi_config;
    midi.Init(midi_config);
    hw.usb_handle.Init(UsbHandle::FS_INTERNAL);
    System::Delay(250);
    InitDisplay();
    specialControlIds.okBttnId = 0;
    specialControlIds.menuEncoderId = 0; 
    specialControlIds.cancelBttnId = 1;
    specialControlIds.funcBttnId = 2;
    
    oledDisplayDescriptor.id_     = 0;
    oledDisplayDescriptor.handle_ = &display;
    oledDisplayDescriptor.updateRateMs_  = 50;   // 50ms == 20Hz
    oledDisplayDescriptor.clearFunction_ = &ClearCanvas;
    oledDisplayDescriptor.flushFunction_ = &FlushCanvas;
    ui.Init(eventQueue, specialControlIds, {oledDisplayDescriptor}, 0);

    InitUiPages();
    ui.OpenPage(mainMenu);

    float sample_rate = hw.AudioSampleRate();
    flt.Init(sample_rate);
	lfo.Init(sample_rate);
    od.Init();
    chorus.Init(sample_rate);
    rev.Init(sample_rate);
    voice_handler.Init(sample_rate);
    sustainNote = false;

    xMapRevFeedback = true;
    yMapLfoRate = true;
    zMapCutoff = true;
    lfoMapPW = true;
    lfoMapShape = true;
    lfoMapPitch = true;
    lfoMapSync = true;
    invFreqLimit = false;

    InitTrill();

    panicButton.Init(hw.GetPin(27), sample_rate / 48.f);
    backButton.Init(hw.GetPin(28), sample_rate / 48.f);
    functionButton.Init(hw.GetPin(29), sample_rate / 48.f);

    encoder.Init(hw.GetPin(PIN_ENC_A), hw.GetPin(PIN_ENC_B), hw.GetPin(PIN_ENC_CLICK), sample_rate / 48.f);
   
    //ADC Init
    AdcChannelConfig adcConfig[ADC_CHANNELS];
    adcConfig[0].InitSingle(hw.GetPin(PIN_Z));
    adcConfig[1].InitMux(hw.GetPin(PIN_ADC_POT_MUX),
                             8,
                             hw.GetPin(PIN_MUX_SEL_0),
                             hw.GetPin(PIN_MUX_SEL_1),
                             hw.GetPin(PIN_MUX_SEL_2));
    
    adcConfig[2].InitMux(hw.GetPin(PIN_ADC_POT_MUX2),
                             8,
                             hw.GetPin(PIN_MUX_SEL_0),
                             hw.GetPin(PIN_MUX_SEL_1),
                             hw.GetPin(PIN_MUX_SEL_2));
    adcConfig[3].InitSingle(hw.GetPin(23));
    hw.adc.Init(adcConfig, ADC_CHANNELS);
    hw.adc.Start();
    System::Delay(100);

    hw.StartAudio(AudioCallback);

    midi.StartReceive();

    while(1) {
	UpdateTrill();
    ui.Process();

    midi.Listen();
    while(midi.HasEvents())
        {
            HandleMidiMessage(midi.PopEvent());
        }
    }
}
