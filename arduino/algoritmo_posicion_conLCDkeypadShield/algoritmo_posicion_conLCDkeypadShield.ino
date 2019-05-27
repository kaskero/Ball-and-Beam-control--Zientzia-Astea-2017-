/*
  Ball & Beam control with LCD keypad shield.
  Created by Oskar Casquero, October 31, 2017.
  Released into the public domain.
*/

#include <LiquidCrystal.h>

const int ppm_pin = 46;

const int sensor_pin = 15;
const int sample_num = 10;
int sample_array[sample_num]; // circular buffer
int sample_array_sum = 0;

bool right_button_pushed = false,
     left_button_pushed = false,
     select_button_pushed = false;
int option = 0;

volatile float setpoint = 30.0;

volatile int timer2_overflow = 0;
volatile float t = 0.0;
volatile bool flag = 0;
  
struct PID_data {
  float Ts = 60;
  double Kp = 15.00,
         Ki = 5.00,
         Kd = 20.00;
  double e_1 = 0.0;
  double iError = 0.0,
         dError = 0.0;
  int outputMin = -900,
      outputMax = 900;
};

PID_data myPID;

long last_time = 0;
double previous_input = 0.0;

LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // D/C, EN, D4, D5, D6, D7

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);

  lcd.begin(16, 2);
  lcd.print("BALL & BEAM CTRL");
  lcd.setCursor(2, 2);
  lcd.print("DISA UPV/EHU");
  delay(3000);

  pinMode(ppm_pin, OUTPUT); //OC5A
  configure_Timer5(); // PPM

  pinMode(sensor_pin, INPUT);
  analogReference(EXTERNAL);

  // initialize circular buffer
  for(int i=0; i<sample_num; i++) {
    sample_array[i] = analogRead(sensor_pin);
    sample_array_sum += sample_array[i];
    delay(60); // sensor measurement interval
  }

  configure_Timer2(); // 30s
}

void loop() {
  // put your main code here, to run repeatedly:

  while(millis() < last_time + myPID.Ts) {
    int button = analogRead(0);
    if(button < 50) {
      Serial.println(button);
      right_button_pushed = true;
      while(right_button_pushed && analogRead(0)<50) { delay(50); }
      right_button_pushed = false;
      setpoint--;
    } else if(button > 650 && button < 750) {
      Serial.println(button);
      left_button_pushed = true;
      while(left_button_pushed && analogRead(0)>650 && analogRead(0)<750) { delay(100); }
      left_button_pushed = false;
      setpoint++;
    } else if(button > 900 && button < 1000) {
      Serial.println(button);
      select_button_pushed = true;
      while(select_button_pushed && analogRead(0)>900 && analogRead(0)<1000) { delay(1500); }
      select_button_pushed = false;
      option++;
      if(option == 1) { myPID.Kp = 15.00, myPID.Ki = 0.00, myPID.Kd = 0.00; }
      else if(option == 2) { myPID.Kp = 15.00, myPID.Ki = 0.00, myPID.Kd = 25.00; }
      else if(option == 3) { 
        myPID.Kp = 15.00, myPID.Ki = 3.00, myPID.Kd = 25.00;
        option = 0;
      }
      flag = 1;
    }
  }
  last_time = millis();

  if(flag) {
    delay(500);
    OCR5A = 2400; // 2 * (2400 * 0.5us) = 2.4ms  --> 180º
    
    myPID.e_1 = 0.0;
    myPID.iError = 0.0;
    flag = 0;
  }
   
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

  double error = input - setpoint;
  Serial.print(" "); Serial.print(error, 1);

  int output = compute_PID(&myPID, error);
  Serial.print(" "); Serial.println(output); 
  OCR5A = output + 1470; // add servo offset at 90º

  myPID.e_1 = error;

  visualize_data(input, error);
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
  else return round(-u);
}

void visualize_data(double input, double error) {
  lcd.clear(); 
  lcd.setCursor(0, 0);
  if(option == 0) lcd.print("PID");
  else if(option == 1) lcd.print("P");
  else if(option == 2) lcd.print("PD");
  lcd.setCursor(4, 0);
  lcd.print("S:"); lcd.print(setpoint, 0);
  lcd.setCursor(9, 0);
  lcd.print("D:"); lcd.print(input, 1);
  lcd.setCursor(1, 1);
  lcd.print("O:"); lcd.print(OCR5A);
  lcd.setCursor(9, 1);
  lcd.print("E:"); lcd.print(error, 1);
}

ISR(TIMER2_COMPA_vect) {
  if(timer2_overflow == 1222) {
    //Serial.println((micros()-t)/1000000.0, 1);
    //t = micros();
    
    OCR5A = 545; // 2 * (545 * 0.5us) = 0.545ms --> 0º

    flag = true;
    
    timer2_overflow = 0;
  } else {
    timer2_overflow++;
  }
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

void configure_Timer2() {
  cli();
  // Comments from ATmega2540 datasheet
  //
  // Timer2 clk/1024 --> t_BIT_TCNT2: 1 / (16*10^6 / 1024) = 64us
  // OCR2A? --> OCR2A * 64us = 1s --> OCR2A = 15625
  // OCR2A 8bit-eko erregistroa da... --> 15625 / 255 = 61 --> 1s
  //                                                  1220 --> 20s
  
  // Table 20-8. Waveform Generation Mode Bit Description
  // Mode WGM22 WGM21 WGM20 Timer/Counter_Mode_of_Operation  TOP
  // 0    0     0     0     Normal                           0xFF
  
  // 20.10.1 TCCR2A – Timer/Counter2 Control Register A
  // COM2A1 COM2A0 COM2B1 COM2B0 X X WGM21 WGM20
  // Table 20-2. Compare Output Mode, non-PWM
  // COM2A1 COM2A0 Description
  // 0      0      Normal port operation, OC2A disconnected
  TCCR2A &= (~(_BV(COM2A1)) & ~(_BV(COM2A0)));
  TCCR2A &= (~(_BV(WGM21)) & ~(_BV(WGM20)));
  
  // 20.10.2 TCCR2B – Timer/Counter1 Control Register B
  // X X - - WGM22 CS22 CS21 CS20
  TCCR2B &= ~(_BV(WGM22));
  
  // Table 20-9. Clock Select Bit Description
  // CS22 CS21 CS20 Description
  // 1    1    1    clk/1024
  TCCR2B |= (_BV(CS22) | _BV(CS21) | _BV(CS20));

  TCNT2 = 0;
  TIMSK2 |= 1<<OCIE2A;
  
  sei();
}
