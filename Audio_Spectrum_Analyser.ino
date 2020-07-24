// Yasteer Sewpersad
// Audio Spectrum LED Analyser

/* Materials Required
 * 1x W9534 Sound Sensor
 * 3x WS2812B - 5050 RGB LED Sticks
 * 1x Arduino Uno
 * 1x External Power Supply For RGBs
 */

#include <arduinoFFT.h>
#include <FastLED.h>
#include <EEPROM.h>

//#define Led_Type WS2812B
#define Colour_Order GRB // According to datasheet. 
#define Number_Of_LED 24
#define Data_Pin 7
CRGB LED_Vector[Number_Of_LED]; // This structure (unique to FastLED lib) is sort of like a vector which stores RGB data for each LED. 
unsigned long Start_Time, Current_Time = 0;
bool Refresh = false;
int BASS_Base_Value, MID_Base_Value, TREBLE_Base_Value;

static const uint16_t samples = pow(2,7); // 128 Samples (MAX!!) -> 64 Bins   ----- Constraint on bin size is due to SRAM size. ATMEGA328P has 2K Bytes and an element of type DOUBLE takes 4 Bytes.
static const double samplingFrequency = 19000; // The FFT can only detect frequencies up to half of the sampling frequency of the ADC. 
double vReal[samples]; // Arrays to store time domain samples. 
double vImag[samples]; // -> Probably takes around 512 Bytes of SRAM each leaving 976 Bytes left for the UNO to do other operations. 

static const double BASS = 250; // Bass is 250Hz and lower.
static const double MID = 2000;
static const double TREBLE = 20000;
int Intensity = 0; // Will be used to track signal strength.
bool Sound_Detected = false;
 
void setup() {
  Serial.begin(115200); // Recommended  baud rate for sound sensor. -> Won't bottleneck the measured samples.
  
  // ADC Setup -> Audible sound has a frequency between 20-20kHz.
  DDRD = DDRD | B00000000; // Digital Input on pin PD2 
  ADMUX =  (0 << REFS1) | (1 << REFS0); // Use 5V reference, left adjust ADCH register,  and use default Channel 0 for input
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (0 << ADPS1) | (1 << ADPS0); // Turn on the ADC & use a prescaler of 32. 
  // The crystal oscillator uses a frequency = 16Mz so ADC Clock Speed = 16MHz/32 = 500kHz
  // Since each conversion in the AVR takes 13 clock cycles, Sample Rate = 50kHz/13 = 38.5kHz. --> Sampling frequency needs to double the frequency of audible sound according to the Nyquist Theorem.  

  // LED Setup
  FastLED.addLeds<WS2812B, Data_Pin>(LED_Vector, Number_Of_LED);
  BASS_Base_Value = EEPROM.read(2);
  MID_Base_Value  = EEPROM.read(2);
  TREBLE_Base_Value = EEPROM.read(3);
  
  Start_Time = millis();
}

void loop() {
  
  Current_Time = millis();
  if((Current_Time - Start_Time) >= 100)
  {
    for(int position=0; position < Number_Of_LED; position++) LED_Vector[position] = CRGB(0,0,0); // Perform a refresh on the LEDS.
    Start_Time = Current_Time;
  }
  
  // Wait until sound sensor registers digital input --> LOW = Input & HIGH = Noise
  while((PIND & B00000100) == true){ 
    // Wait until sound is registered or else, the FFT will try to process the base value of the sensor.
    Sound_Detected = false;
  }
  
  if((PIND & B00000100) == false)
  {
    Sound_Detected = true;
  }
  
  // Sample Signal
  for(int count = 0; count < samples; count++){
    vReal[count] = Read_ADC(); 
    vImag[count] = 0; // A real signal is being measured so there is no imaginary component.
    Intensity = (vReal[count]/4) - 1; // Factor converts 10-bit ADC into a 0-255 valued range for the RGBs.
  }
  
  // Compute the Fast Fourier Transform --> Decomposes a signal into its frequencies.
  // Follow this link to check function implementations: https://github.com/kosme/arduinoFFT/blob/master/src/arduinoFFT.cpp
  arduinoFFT FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); // Intialize an FFT object from the library. Each object contains new data to sample.
  FFT.DCRemoval(); // Should remove the first element in the fourier transform array.
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
  double Peak_Frequency = FFT.MajorPeak(); // According to the documentation: "Looks for and returns the frequency of the biggest spike in the analyzed signal."
  
  // Return frequency with highest amplitude:
  // Serial.println(Peak_Frequency);

  // View a graphic display of which bins contains the highest frequencies:
  for(int disp1 = 0; disp1 < (samples/2); disp1++){ // Since the data gets repeated, we can only draw useful results from one half of the array. 
    //Serial.println(vReal[disp1],1);
  }

  //Serial.println("Bass");
  //Serial.println(vReal[1],vReal[8]);
  //Serial.println(",");
  //Serial.println("MID");
  //Serial.println(vReal[8]);
  //Serial.println(",");
 // Serial.println("TREBLE");
  //Serial.println(vReal[1]);

  /*
  Bin resolution = Sampling Frequency/2/64 -> +- 150 Hz per bin
  Bin 0           -> DC
  Bin 1           -> BASS   Frequency Range. 
  Bin 2  - Bin 13 -> MID    Frequency Range.
  Bin 14 - Bin 64 -> TREBLE Frequency Range.  
  */
  
  // RGB Display // --> BASS Monitor (Due to the lack of RGBs, the bin representing the midpoint of each range is used.) 
  // RED Stick
  if(vReal[1] > BASS_Base_Value) 
  {
    LED_Vector[0] = CRGB(0,255,0);
  }
  if((vReal[1] > BASS_Base_Value * 20) && (Sound_Detected == true))
  {
    LED_Vector[1] = CRGB(0,255,0);
  }
  if((vReal[1] > BASS_Base_Value * 40) && (Sound_Detected == true))
  {
    LED_Vector[2] = CRGB(0,255,0);
  }
  if((vReal[1] > BASS_Base_Value * 60) && (Sound_Detected == true))
  {
    LED_Vector[3] = CRGB(0,255,0);
  }
  if((vReal[1] > BASS_Base_Value * 80) && (Sound_Detected == true))
  {
    LED_Vector[4] = CRGB(0,255,0);
  }
  if((vReal[1] > BASS_Base_Value * 100) && (Sound_Detected == true))
  {
    LED_Vector[5] = CRGB(0,255,0);
  }
  if((vReal[1] > BASS_Base_Value * 120) && (Sound_Detected == true))
  {
    LED_Vector[6] = CRGB(0,255,0);
  }
  if((vReal[1] > BASS_Base_Value * 140) && (Sound_Detected == true))
  {
    LED_Vector[7] = CRGB(0,255,0);
  }


  // RGB Display // --> MID Monitor
  if(vReal[8] > MID_Base_Value)
  {
    LED_Vector[8] = CRGB(255,0,0);
  }
  if((vReal[8] > MID_Base_Value * 2) && (Sound_Detected == true))
  {
    LED_Vector[9] = CRGB(255,0,0);
  }
  if((vReal[8] > MID_Base_Value * 3) && (Sound_Detected == true))
  {
    LED_Vector[10] = CRGB(255,0,0);
  }
  if((vReal[8] > MID_Base_Value * 4) && (Sound_Detected == true))
  {
    LED_Vector[11] = CRGB(255,0,0);
  }
  if((vReal[8] > MID_Base_Value * 5) && (Sound_Detected == true))
  {
    LED_Vector[12] = CRGB(255,0,0);
  }
  if((vReal[8] > MID_Base_Value * 6) && (Sound_Detected == true))
  {
    LED_Vector[13] = CRGB(255,0,0);
  }
  if((vReal[8] > MID_Base_Value * 7) && (Sound_Detected == true))
  {
    LED_Vector[14] = CRGB(255,0,0);
  }
  if((vReal[8] > MID_Base_Value * 8) && (Sound_Detected == true))
  {
    LED_Vector[15] = CRGB(255,0,0);
  }

  // RGB Display // --> TREBLE Monitor (BLUE)
  if(vReal[37] > TREBLE_Base_Value)
  {
    LED_Vector[16] = CRGB(0,0,255);
  }
  if((vReal[37] > TREBLE_Base_Value * 2) && (Sound_Detected == true))
  {
    LED_Vector[17] = CRGB(0,0,255);
  }
  if((vReal[37] > TREBLE_Base_Value * 3) && (Sound_Detected == true))
  {
    LED_Vector[18] = CRGB(0,0,255);
  }
  if((vReal[37] > TREBLE_Base_Value * 4) && (Sound_Detected == true))
  {
    LED_Vector[19] = CRGB(0,0,255);
  }
  if((vReal[37] > TREBLE_Base_Value * 5) && (Sound_Detected == true))
  {
    LED_Vector[20] = CRGB(0,0,255);
  }
  if((vReal[37] > TREBLE_Base_Value * 6) && (Sound_Detected == true))
  {
    LED_Vector[21] = CRGB(0,0,255);
  }
  if((vReal[37] > TREBLE_Base_Value * 7) && (Sound_Detected == true))
  {
    LED_Vector[22] = CRGB(0,0,255);
  }
  if((vReal[37] > TREBLE_Base_Value * 8) && (Sound_Detected == true))
  {
    LED_Vector[23] = CRGB(0,0,255);
  }

  FastLED.show();
  
  delay(200);
  //while(1);

}

unsigned int Read_ADC(){
  ADCSRA |= (1 << ADSC);// Start ADC conversion. Use OR operator to prevent overwriting register, we still want the other settings. 
  while(ADIF == 0){
     //Wait until flag is raised indicating conversion complete and registers updated. 
  }
  return ADC;
}
