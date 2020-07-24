// Yasteer Sewpersad

#include <EEPROM.h>
#include <arduinoFFT.h>

static const uint16_t samples = pow(2,7); // 128 Samples (MAX!!) -> 64 Bins   ----- Constraint on bin size is due to SRAM size. ATMEGA328P has 2K Bytes and an element of type DOUBLE takes 4 Bytes.
static const double samplingFrequency = 19000; // The FFT can only detect frequencies up to half of the sampling frequency of the ADC. 
double vReal_Calibration[samples];
double vImag_Calibration[samples];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Calibrate_FFT_Array();

  Serial.println(EEPROM.read(0));
  Serial.println(EEPROM.read(1));
  Serial.println(EEPROM.read(2));
  Serial.println(EEPROM.read(3));
}

void loop() {
  // put your main code here, to run repeatedly:

}

void Calibrate_FFT_Array()
{
  // Clear Current State Of EEPROM
  for(int position = 0; position < EEPROM.length(); position++)
  {
    EEPROM.write(position, 0);
  }
  
  Serial.println("Recording Audio!");
  for(int position=0; position < samples; position++)
  {
      vReal_Calibration[position] = Read_ADC(); 
      vImag_Calibration[position] = 0;
  } 
  arduinoFFT FFT = arduinoFFT(vReal_Calibration, vImag_Calibration, samples, samplingFrequency); // Intialize an FFT object from the library. Each object contains new data to sample.
  FFT.DCRemoval(); // Should remove the first element in the fourier transform array.
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();
  
  EEPROM.write(0, vReal_Calibration[0]); // DC Component
  EEPROM.write(1, vReal_Calibration[1]); // BASS
  EEPROM.write(2, vReal_Calibration[8]); // MID
  EEPROM.write(3, vReal_Calibration[37]);// TREBLE
  Serial.println("Calibration Completed.");
  return;
}

unsigned int Read_ADC(){
  ADCSRA |= (1 << ADSC);// Start ADC conversion. Use OR operator to prevent overwriting register, we still want the other settings. 
  while(ADIF == 0){
     //Wait until flag is raised indicating conversion complete and registers updated. 
  }
  return ADC;
}
