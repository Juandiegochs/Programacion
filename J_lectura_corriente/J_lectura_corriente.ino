////////////amperio////////
//Setup variables
int numberOfSamples = 4000;
double ICAL = 0.625; //0.625

//Set Voltage and current input pins
int inPinI1 = 0;

// CT: Voltage depends on current, burden resistor, and turns
#define CT_BURDEN_RESISTOR    62
#define CT_TURNS              1800
#define VOLTAGE               115

// Initial gueses for ratios, modified by VCAL/ICAL tweaks
double I_RATIO = (long double)CT_TURNS / CT_BURDEN_RESISTOR * 5.50 / 1023 * ICAL;

//Filter variables 1
double lastFilteredI, filteredI;
double sqI, sumI;
//Sample variables
int lastSampleI, sampleI;
double Irms1;
unsigned long timer;
//////////////////////////
void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(get_corriente());
  delay(300);
}
float get_corriente()
{
  timer = millis();
  //**************************************************************************
  //Phase1
  for (int n = 0; n < numberOfSamples; n++)
  {

    //Used for offset removal
    lastSampleI = sampleI;

    //Read in voltage and current samples.
    sampleI = analogRead(inPinI1);

    //Used for offset removal
    lastFilteredI = filteredI;

    //Digital high pass filters to remove 1.6V DC offset.
    filteredI = 0.9989 * (lastFilteredI + sampleI - lastSampleI);

    //Root-mean-square method current
    //1) square current values
    sqI = filteredI * filteredI;
    //2) sum
    sumI += sqI;
    delay(0.0002);
  }

  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coeficients applied.
  Irms1 = (I_RATIO * sqrt(sumI / numberOfSamples)) - 0.06;
  if (Irms1 < 0) {
    Irms1 = 0;
  }; //Set negative Current to zero
  sumI = 0;

  //Serial.println("Irms1:" + String(Irms1));
  return Irms1;
}

/*
// Include Emon Library
#include "EmonLib.h"
 
// Crear una instancia EnergyMonitor
EnergyMonitor energyMonitor;
 
// Voltaje de nuestra red eléctrica
float voltajeRed = 120.0;
 
void setup()
{
  Serial.begin(9600);
 
  // Iniciamos la clase indicando
  // Número de pin: donde tenemos conectado el SCT-013
  // Valor de calibración: valor obtenido de la calibración teórica
  energyMonitor.current(0, 0.036);
}
 
void loop()
{
  // Obtenemos el valor de la corriente eficaz
  // Pasamos el número de muestras que queremos tomar
  double Irms = energyMonitor.calcIrms(1480);
 
  // Calculamos la potencia aparente
  double potencia =  Irms * voltajeRed;
 
  // Mostramos la información por el monitor serie
  Serial.print("Potencia = ");
  Serial.print(potencia);
  Serial.print("    Irms = ");
  Serial.println(Irms);
}
*/
