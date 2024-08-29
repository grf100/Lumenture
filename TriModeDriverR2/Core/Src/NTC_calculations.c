#include "main.h"
#include "math.h"

#include "NTC_calculations.h"


float thermistorResistance(uint16_t ntc_filt_reading) {
float ratio = (float)ntc_filt_reading/4096.0;
float resistance = 10000 * ratio/ (1-ratio);
return (resistance);
}

float thermistorTemperature(float R) {
//  float T25 = 25+273.15; // Convert to Kelvin
 float T =  1 / ((1 / T25) + ((log(R / R25)) / BETA));
 float Tc = T - 273.15; // Converting kelvin to celsius
  return (Tc);
}
