// Yasteer Sewpersad
// Audio Spectrum LED Analyser

/* Materials Required
 * 1x W9534 Sound Sensor
 * 3x WS2812B - 5050 RGB LED Sticks
 * 1x Arduino Uno
 */

#include <arduinoFFT.h>
#include <FastLED.h>

//#define Led_Type WS2812B
#define Colour_Order GRB // According to datasheet. 
#define Number_Of_LED 8
#define Data_Pin 7
CRGB LED_Vector[Number_Of_LED]; // This structure (unique to FastLED lib) is sort of like a vector which stores RGB data for each LED. 

static const uint16_t samples = pow(2,7); // 128 Samples (MAX!!) -> 64 Bins     Reference: https://www.norwegiancreations.com/2017/08/what-is-fft-and-how-can-you-implement-it-on-an-arduino/
static const double samplingFrequency = 20000; // The FFT can only detect frequencies up to half of the sampling frequency. 
double vReal[samples]; // Arrays to store values from the FFT evaluation. 
double vImag[samples];

static const double BASS = 250; // Bass is 250Hz and lower.
static const double MID = 2000;
static const double TREBLE = 20000;
int Intensity = 0; // Will be used to track signal strength.
 
void setup() {
  Serial.begin(115200);
  
  // ADC Setup -> Audible sound has a frequency between 20-20kHz.
  DDRD = DDRD | B00000000; // Digital Input on pin PD2 
  ADMUX =  (0 << REFS1) | (1 << REFS0); // Use 5V reference, left adjust ADCH register,  and use default Channel 0 for input
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (0 << ADPS1) | (1 << ADPS0); // Turn on the ADC & use a prescaler of 32. 
  // The crystal oscillator uses a frequency = 16Mz so Sampling Frequency = 16MHz/32 = 500kHz
  // Since each conversion takes 13 clock cycles, Sample Rate = 50kHz/13 = 38.5kHz. --> Sampling frequency needs to double the frequency we're measuring according to the Nyquist Theorem.  

  // LED Setup
  FastLED.addLeds<WS2812B, Data_Pin>(LED_Vector, Number_Of_LED);
}

void loop() {
  // Wait until sound sensor registers digital input.
  while((PIND & B00000100) == false){ 
    // Wait
  };

  // Sample Signal
  for(int count = 0; count < samples; count++){
    vReal[count] = Read_ADC(); 
    vImag[count] = 0; // A real signal is being measured so there is no imaginary component.
    Intensity = (vReal[count]/4) - 1; // Factor converts 10-bit ADC into a 0-255 valued range for the RGBs.
  }
  
  // Compute the Fast Fourier Transform --> Decomposes a signal into its frequencies.
  // Follow this link to check function implementations: https://github.com/kosme/arduinoFFT/blob/master/src/arduinoFFT.cpp
  arduinoFFT FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); // Intialize an FFT object from the library. Each object contains new data to sample.
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
  double Peak_Frequency = FFT.MajorPeak(); // According to the documentation: "Looks for and returns the frequency of the biggest spike in the analyzed signal."
  
  // Display Results
  Serial.println(Peak_Frequency);

  // View a Graphic Display
  //for(int disp1 = 0; disp1 < (samples/2); disp1++){
  //  Serial.println(vReal[disp1],1);
  //}

  // RGB Display --> 3 Frequency Bands: BASS, MID, TREBLE
  if(Peak_Frequency > 0 && Peak_Frequency < BASS){
    LED_Vector[0] = CRGB(0,255 - Intensity, 0);
    LED_Vector[1] = CRGB(0,255 - Intensity, 0);
    LED_Vector[2] = CRGB(0,0, 255 - Intensity);
    LED_Vector[3] = CRGB(0,0, 255 - Intensity);
    LED_Vector[4] = CRGB(0,0, 255 - Intensity);
    LED_Vector[5] = CRGB(255 - Intensity,255 - Intensity, 0);
    LED_Vector[6] = CRGB(255 - Intensity,0, 0);
    LED_Vector[7] = CRGB(255 - Intensity,0, 0);
  }

  else if(Peak_Frequency > BASS && Peak_Frequency < MID){
    LED_Vector[8] = CRGB(0,255 - Intensity, 0);
    LED_Vector[9] = CRGB(0,255 - Intensity, 0);
    LED_Vector[10] = CRGB(0,0, 255 - Intensity);
    LED_Vector[11] = CRGB(0,0, 255 - Intensity);
    LED_Vector[12] = CRGB(0,0, 255 - Intensity);
    LED_Vector[13] = CRGB(255 - Intensity,255 - Intensity, 0);
    LED_Vector[14] = CRGB(255 - Intensity,0, 0);
    LED_Vector[15] = CRGB(255 - Intensity,0, 0);
  }

  else if(Peak_Frequency > MID && Peak_Frequency < TREBLE){
    LED_Vector[16] = CRGB(0,255 - Intensity, 0);
    LED_Vector[17] = CRGB(0,255 - Intensity, 0);
    LED_Vector[18] = CRGB(0,0, 255 - Intensity);
    LED_Vector[19] = CRGB(0,0, 255 - Intensity);
    LED_Vector[20] = CRGB(0,0, 255 - Intensity);
    LED_Vector[21] = CRGB(255 - Intensity,255 - Intensity, 0);
    LED_Vector[22] = CRGB(255 - Intensity,0, 0);
    LED_Vector[23] = CRGB(255 - Intensity,0, 0);
  }

  FastLED.show();
  
  delay(1000);
}

unsigned int Read_ADC(){
  ADCSRA |= (1 << ADSC);// Start ADC conversion. Use OR operator to prevent overwriting register, we still want the other settings. 
  while(ADIF == 0){
     //Wait until flag is raised indicating conversion complete and registers updated. 
  }
  return ADC;
}
