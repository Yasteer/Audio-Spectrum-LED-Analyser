// Yasteer Sewpersad
// Audio Spectrum LED Analyser

/* Materials Required
 * 1x W9534 Sound Sensor
 * 5x WS2812B - 5050 RGB LEDs
 * 1x Arduino Uno
 */

int SS_Analogue_Reading = 0;
bool SS_Digital_Reading = 0;
 
void setup() {
  Serial.begin(9600);
  // ADC Setup
  DDRD = DDRD | B00000000; // Digital Input on pin PD2 
  ADMUX =  (0 << REFS1) | (1 << REFS0); // Use 5V reference, left adjust ADCH register,  and use default Channel 0 for input
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Turn on the ADC & use a prescaler of 128. The crystal oscillator uses a frequency = 16Mz so Sampling Frequency = 16Mz/128 = 125kHz
   
}

void loop() {

  if(PIND & B00000100){ // Check if Pin PD2 is set. 
    SS_Digital_Reading = true;
  }
  
  if(SS_Digital_Reading == true){ // Only record an analogue reading if there is a suitable level of audio available. 
    SS_Analogue_Reading = Read_ADC();
  }

  // Compute the Fast Fourier Transform --> Decomposes a signal into its frequencies.
  

}

unsigned int Read_ADC(){
  ADCSRA |= (1 << ADSC);// Start ADC conversion. Use OR operator to prevent overwriting register, we still want the other settings. 
  while(ADIF == 0){
     //Wait until flag is raised indicating conversion complete and registers updated. 
  }
  return ADC;
}
