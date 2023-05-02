unsigned long lastmillis = millis();
int adc_max = 704;      //Reemplazar por valor adc_max entregado por el sketch: volt_ac_cal
int adc_min = 356;      //Reemplazar por valor adc_min entregado por el sketch: volt_ac_cal
float volt_multi = 140.6; //Reemplazar por el "voltaje ac rms" entregado por un multimetro
float volt_multi_p;
float volt_multi_n;
void setup(){
  Serial.begin(9600);
   volt_multi_p = volt_multi * 1.4142;   //Voltaje pico= Voltaje RMS * 1.4142 (Corriente Monofasica)
  volt_multi_n = volt_multi_p * -1;
}
void loop(){
  Serial.println(get_voltaje());
}
float get_voltaje(void)
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
  return (volt);
}
/*double sensorValue1 = 0;
double sensorValue2 = 0;
int crosscount = 0;
int climb_flag = 0;
int val[100];
int max_v = 0;
double VmaxD = 0;
double VeffD = 0;
double Veff = 0;

void setup() {
  Serial.begin(9600);
  pinMode(A1, INPUT);
}

void loop() {

  for (int i=0; i<100; i++) {
    sensorValue1 = analogRead(A1);
    if (analogRead(A1) > 511) { val[i] = sensorValue1; }
    else { val[i] = 0; }
    delay(1);
  }

  max_v = 0;

  for ( int i = 0; i < 100; i++ ) {
    if ( val[i] > max_v ) { max_v = val[i]; }
    val[i] = 0;
  }
  if (max_v != 0) {
    VmaxD = max_v;
    VeffD = VmaxD / sqrt(2);
    Veff = (((VeffD - 420.76) / -90.24) * -110.2) + 110.2;
  }
  else { Veff = 0; }
  Serial.print("Voltage: ");
  Serial.println(Veff);
  VmaxD = 0;

  delay(300);
}*/
