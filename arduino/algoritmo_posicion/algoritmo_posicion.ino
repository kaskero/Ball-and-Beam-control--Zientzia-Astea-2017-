/*
  Ball & Beam control.
  Created by Oskar Casquero, October 26, 2017.
  Released into the public domain.
*/

const int ppm_pin = 46;

const int sensor_pin = 15;
const int sample_num = 10;
int sample_array[sample_num]; // circular buffer
int sample_array_sum = 0;

float setpoint = 20.0;
  
struct PID_data {
  float Ts = 60;
  double Kp = 15.00,
         Ki = 3.00,
         Kd = 25.00;
  double e_1 = 0.0;
  double iError = 0.0,
         dError = 0.0;
  int outputMin = -900,
      outputMax = 900;
};

PID_data myPID;

long last_time = 0;
double previous_input = 0.0;
 
void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);

  pinMode(ppm_pin, OUTPUT); // OC5A
  configure_Timer5(); // PPM

  pinMode(sensor_pin, INPUT);
  analogReference(3.3);

  // initialize circular buffer
  for(int i=0; i<sample_num; i++) {
    sample_array[i] = analogRead(sensor_pin);
    sample_array_sum += sample_array[i];
    delay(60); // sensor measurement interval
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  while(millis() < last_time + myPID.Ts) {}
  last_time = millis();

  sample_array_sum -= sample_array[0];
  // shift circular buffer as FIFO
  for(int i=1; i<sample_num; i++) {
    sample_array[i-1] = sample_array[i];
  }
  sample_array[sample_num-1] = analogRead(sensor_pin);
  sample_array_sum += sample_array[sample_num-1];
  
  float ADC_mean_value = sample_array_sum / sample_num;
  float estimated_voltage = ADC_mean_value * (3.3 / 1024);

  // input = f(voltage) polynomial regression based on the following empirical values
  // distance [cm]: 45, 40,35, 30, 25, 20, 15, 10, 5
  // voltage [v]: 0.486, 0.564, 0.643, 0.783, 0.939, 1.21, 1.575, 2.202, 3.0
  // R-squared = 0.9997
  double input = 1.2502 * pow(estimated_voltage, 6) 
               - 16.396 * pow(estimated_voltage, 5) 
               + 84.983 * pow(estimated_voltage, 4) 
               - 225.46 * pow(estimated_voltage, 3) 
               + 330.80 * pow(estimated_voltage, 2) 
               - 272.10 * estimated_voltage 
               + 120.72; 

  // LPF (Low Pass Filter) by means of EMA (Estimated Mobile Average)
  float alpha = 0.5;
  input = (alpha * input) + (1-alpha) * previous_input;
  previous_input = input;
  Serial.print(input, 1);
    
  double error = setpoint - input;
  Serial.print(" "); Serial.print(error, 1);
  
  int output = compute_PID(&myPID, error);
  Serial.print(" "); Serial.println(output); 
  OCR5A = output + 1470; // add servo offset at 90º

  myPID.e_1 = error;
}

double compute_PID(PID_data* pid, double e) {
  double pTerm, dTerm, iTerm;
  float Ts = pid->Ts / 1000;

  pTerm = pid->Kp * e;   
  Serial.print(" "); Serial.print(pTerm, 1);

  //pid->iError += e*Ts; // aproximacion rectangular o de Euler
  pid->iError += ((e + pid->e_1) / 2) * Ts; // aproximación trapezoidal o de Tustin
  iTerm = pid->Ki * pid->iError;
  Serial.print(" "); Serial.print(iTerm, 1);

  float dError = e - pid->e_1;
  // EMA LPF to reduce the effect of input noise on dError
  float alpha = 0.75;
  pid->dError = (alpha * dError) + ((1 - alpha) * pid->dError);
  dTerm = pid->Kd * (pid->dError / Ts);
  Serial.print(" "); Serial.print(dTerm, 1);

  double u = pTerm + iTerm + dTerm;
  if(u > pid->outputMax) return pid->outputMax;
  else if(u < pid->outputMin) return pid->outputMin;
  else return round(u);
}

void configure_Timer5() {
  // Comments from ATmega328 datasheet
  //
  // Table 16-4. Waveform Generation Mode Bit Description
  // Mode WGM13 WGM12 WGM11 WGM10 Timer/Counter_Mode_of_Operation  TOP
  // 8    1     0     0     0     PWM, Phase and Frequency Correct ICR1
    
  // 16.11.1 TCCR1A – Timer/Counter1 Control Register A
  // COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
  // COM1A1:0 and COM1B1:0 control the Output Compare pins (OC1A and OC1B) behavior.
  // Table 16-3. Compare Output Mode, Phase Correct and Phase and Frequency Correct PWM
  // COM1A1/COM1B1 COM1A0/COM1B0 Description
  //       1             0       Clear OC1A/OC1B on Compare Match when upcounting.
  //                             Set OC1A/OC1B on Compare Match when downcounting.
  TCCR5A |= _BV(COM5A1);
  TCCR5A &= ~(_BV(COM5A0));
  TCCR5A &= (~(_BV(WGM51)) & ~(_BV(WGM50)));
  
  // 16.11.2 TCCR1B – Timer/Counter1 Control Register B
  // X X - WGM13 WGM12 CS12 CS11 CS10
  TCCR5B |= _BV(WGM53);
  TCCR5B &= ~(_BV(WGM52));
  
  // Table 16-5. Clock Select Bit Description
  // CS12 CS11 CS10 Description
  // 0    1    0    clk/8
  TCCR5B |= _BV(CS51);
  TCCR5B &= (~(_BV(CS52)) & ~(_BV(CS50)));

  // t_BIT_TCNT1: 1 / (16*10^6 / 8) = 0.5us
  ICR5 = 20000; // t_PWM: 2 * (20000 * 0.5us) = 20ms --> 50Hz
                // x2 because it is phase correct
 
  OCR5A = 545; // 2 * (545 * 0.5us) = 0.545ms --> 0º
  delay(1000);
  OCR5A = 2400; // 2 * (2400 * 0.5us) = 2.4ms  --> 180º
}

