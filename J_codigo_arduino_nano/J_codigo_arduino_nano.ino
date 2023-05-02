#include <Wire.h>
#include "RTClib.h"
RTC_DS3231 rtc;
#include <SoftwareSerial.h>

const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial mySerial (rxPin, txPin);

/////////******** variables para medicion volatje**///////////

unsigned long lastmillis = millis();
int adc_max = 704;      //Reemplazar por valor adc_max entregado por el sketch: volt_ac_cal
int adc_min = 356;      //Reemplazar por valor adc_min entregado por el sketch: volt_ac_cal
float volt_multi = 116.6; //Reemplazar por el "voltaje ac rms" entregado por un multimetro
float volt_multi_p;
float volt_multi_n;
///////////////////////////////////////////////////////////////

/////////******** variables para medicion corriente**///////////

int numberOfSamples = 4000;
double ICAL = 0.625; //0.625
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
///////////////////////////////////////////////////////////////

void setup() {
  mySerial.begin(9600);
  pinMode(A0, INPUT);  // entrada voltaje
  pinMode(A1, INPUT); //entrada corriente
  volt_multi_p = volt_multi * 1.4142;   //Voltaje pico= Voltaje RMS * 1.4142 (Corriente Monofasica)
  volt_multi_n = volt_multi_p * -1;
  if (!rtc.begin()) {
   // mySerial.println(F("Couldn't find RTC"));
    while (1);
  }
  // Si se ha perdido la corriente, fijar fecha y hora
  if (rtc.lostPower()) {
    // Fijar a fecha y hora de compilacion
     //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // Fijar a fecha y hora específica. En el ejemplo, 21 de Enero de 2016 a las 03:00:00
    // rtc.adjust(DateTime(2016, 1, 21, 3, 0, 0));
  }
}

float volt_rms;
float Irms;
float sum_volt = 0;
float sum_corr = 0;
int cont = 0;
boolean estado = true;
int tiempo_contador=1;
void loop() {
 // mySerial.println("Entro");
  // Obtener fecha actual y mostrar por Serial
   //DateTime now = rtc.now();
    //String c=String(now.day())+"/"+String(now.month())+"/"+String(now.year())+" "+String(now.hour())+":"+String(now.minute())+":"+String(now.second())+","+String(get_voltaje())+","+String(get_corriente());
  //mySerial.println(c);

 // Serial.println(get_voltaje());
  condiciones_lectura();
  DateTime now = rtc.now();
  if (now.second() >= 0 && now.second() <= 4 && estado == true) {    /// cada minuto  guarda un dato del valor de voltaje y corriente
   // Serial.print("calculando: "); Serial.println( String(now.minute()) + ":" + String(now.second()));
    //String cadena=String(now.day())+"/"+String(now.month())+"/"+String(now.year())+" "+String(now.hour())+":"+String(now.minute())+":"+String(now.second())+","+String(get_voltaje())+","+String(get_corriente());

    sum_volt += get_voltaje();//volt_rms;//get_voltaje();
    sum_corr += Irms;
    cont ++;
    estado = false;
    //Serial.print("contador: "); Serial.println(cont);
    //Serial.print("voltaje: "); Serial.println(volt_rms);
    //Serial.print("suma vol: "); Serial.println(sum_volt);
    //delay(1000);
  } else if (now.second() >= 5  ) {
    estado = true;
  }

  if (cont == tiempo_contador ) {  // cada 2 min envia una cedena con los valores para ser almacenados en MongoBD
    String fecha = String(now.month()) + "/" + String(now.day()) + "/" + String(now.year()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
    // Serial.print(fecha);
    // String cadena=String(now.day())+"/"+String(now.month())+"/"+String(now.year())+" "+String(now.hour())+":"+String(now.minute())+":"+String(now.second())+","+String(get_voltaje())+","+String(get_corriente());

    float tot_vol = sum_volt / tiempo_contador;
    float tot_corr = sum_corr / tiempo_contador;
    String cadena = fecha + "," + tot_vol + "," + tot_corr;
    //Serial.print("Cadena: ");
    mySerial.println(cadena);
    // enviar_mongobd(cadena);
    sum_volt = 0;
    sum_corr = 0;
    cont = 0;
  } else {
    //enviar_datos_node(volt_rms, Irms);
  }
//   Serial.println(cadena);
//mySerial.println("673,65,999");
}


void condiciones_lectura() {
  for (int i = 0; i <= 3; i++) {
    volt_rms = get_voltaje();
  }
  volt_rms += volt_rms / 4;
  Irms = get_corriente(); //Corriente eficaz (A)
  if ( volt_rms > 90) {
    Serial.println(String( volt_rms) + "," + String(Irms));
  } else {
    volt_rms = 0;
    Irms = 0;
  }
}

float get_voltaje()
{
  float adc_sample;
  float volt_inst = 0;
  float Sumatoria = 0;
  float volt;
  long tiempo_init = millis();
  int N = 0;

  while ( (millis() - tiempo_init) < 500) //Duración 0.5 segundos(Aprox. 30 ciclos de 60Hz)
  {
    //adc_sample = analogRead(A0) - 510;////voltaje del sensor
    //volt_inst = map(adc_sample,-188,188,-310,310);

    adc_sample = analogRead(A1);    //voltaje del sensor
    volt_inst = map(adc_sample, adc_min, adc_max, volt_multi_n, volt_multi_p);
    Sumatoria = Sumatoria + sq(volt_inst);  //Sumatoria de Cuadrados
    N = N + 1;
    delay(1);
  }

  //Serial.print("N: ");
  //Serial.println(N);

  volt = sqrt((Sumatoria) / N); //ecuación del RMS
  //Serial.print("V: ");
  //Serial.println(volt);
  if(volt < 90){
    volt = 0;
    sum_volt = 0;
  }
  return (volt);
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
