// Yasteer Sewpersad
// Audio Spectrum LED Analyser

/* Materials Required
 * 1x W9534 Sound Sensor
 * 5x WS2812B - 5050 RGB LEDs
 * 1x Arduino Uno
 */

#include <arduinoFFT.h>
static const uint16_t samples = 128;
static const double samplingFrequency = 1000; 
double vReal[samples]; // Arrays to store values from the FFT evaluation. 
double vImag[samples];

int SS_Analogue_Reading = 0;
bool SS_Digital_Reading = 0;
 
void setup() {
  Serial.begin(115200);
  // ADC Setup
  DDRD = DDRD | B00000000; // Digital Input on pin PD2 
  ADMUX =  (0 << REFS1) | (1 << REFS0); // Use 5V reference, left adjust ADCH register,  and use default Channel 0 for input
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Turn on the ADC & use a prescaler of 128. The crystal oscillator uses a frequency = 16Mz so Sampling Frequency = 16Mz/128 = 125kHz
   
}

void loop() {
  // Check that sound sensor is functional.
  if(PIND & B00000100){ // Check if Pin PD2 is set. 
    SS_Digital_Reading = true;
  }

  // Sample Signal
  for(int count = 0; count < samples; count++){
    if(SS_Digital_Reading == true){ // Only record an analogue reading if there is a suitable level of audio available. 
      SS_Analogue_Reading = Read_ADC();
      vReal[count] = SS_Analogue_Reading; 
      vImag[count] = 0; // A real signal is being measured so there is no imaginary component.
    }
  }
  
  // Compute the Fast Fourier Transform --> Decomposes a signal into its frequencies.
  // Follow this link to check function implementations: https://github.com/kosme/arduinoFFT/blob/master/src/arduinoFFT.cpp
  arduinoFFT FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); // Intialize an FFT object from the library. Each object contains new data to sample.
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
  FFT.MajorPeak();
  
  // Display Results
  for(int disp1 = 0; disp1 < (samples/2); disp1++){
    Serial.println(vReal[disp1],1);
  }

  delay(1000);
}

unsigned int Read_ADC(){
  ADCSRA |= (1 << ADSC);// Start ADC conversion. Use OR operator to prevent overwriting register, we still want the other settings. 
  while(ADIF == 0){
     //Wait until flag is raised indicating conversion complete and registers updated. 
  }
  return ADC;
}
