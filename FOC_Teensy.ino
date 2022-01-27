
/////////////////////////////////////////////////////
/* space vector modulation */
// pins define
#define AH_pin 7
#define AL_pin 8
#define BH_pin 9
#define BL_pin 10
#define CH_pin 11
#define CL_pin 12
// svm rontine 
#include <TimerOne.h>
#define pwm_freq 2000.0f  // unit -> Hz
#define pwm_res 200
const float pwm_period = 1000000.0f / pwm_freq / (float)pwm_res; // unit -> us
uint16_t pwm_count = 0; // maximum -> 16 bit
// parameter
#define Vm 12.0f  // power supply voltage
#define deg_to_rad          0.01745329251f
#define _2_divided_by_3     0.66666666666f  
#define root_3_divided_by_2 0.86602540378f 
#define root_3_divided_by_3 0.57735026919f 
#define root_3              1.73205080757f  
#define c -1.15f  // constant value of the linear equation
const float m  = 2.3f / pwm_res;  // slope value of the linear equation
float Vv,         // vector result between Vx and Vy
      Vx, Vy,     // config by serial command
      duty_cycle, // duty cycle of pwm
      Va, Vb, Vc, // voltage each phase
      unit_vector, 
      Va_unit, Vb_unit, Vc_unit,  // voltage each phase in unit vector
      pwmA, pwmB, pwmC;           // pwm output for config to compare match registor
uint16_t pwmA_on, pwmB_on, pwmC_on, pwmA_off, pwmB_off, pwmC_off;
volatile boolean motor_stop = false;

void svm_generate()
{
  pwm_count++;
   
  if(pwm_count > pwm_res - 1) pwm_count = 0;  
  if(motor_stop)
  {
    digitalWrite(AH_pin, LOW);
    digitalWrite(AL_pin, LOW);
    digitalWrite(BH_pin, LOW);
    digitalWrite(BL_pin, LOW); 
    digitalWrite(CH_pin, LOW);
    digitalWrite(CL_pin, LOW);
  }
  else
  {
    pwmA_on = (pwm_res - pwmA) / 2;
    pwmB_on = (pwm_res - pwmB) / 2;
    pwmC_on = (pwm_res - pwmC) / 2;
    pwmA_off = pwm_res - pwmA_on;
    pwmB_off = pwm_res - pwmB_on;
    pwmC_off = pwm_res - pwmC_on;

    if(pwm_count == pwmA_on)
    {
      digitalWrite(AH_pin, HIGH);
      digitalWrite(AL_pin, LOW);
    }
    else if(pwm_count == pwmA_off)
    {
      digitalWrite(AH_pin, LOW);
      digitalWrite(AL_pin, HIGH);
    }
    if(pwm_count == pwmB_on)
    {
      digitalWrite(BH_pin, HIGH);
      digitalWrite(BL_pin, LOW);
    }
    else if(pwm_count == pwmB_off)
    {
      digitalWrite(BH_pin, LOW);
      digitalWrite(BL_pin, HIGH);
    } 
    if(pwm_count == pwmC_on)
    {
      digitalWrite(CH_pin, HIGH);
      digitalWrite(CL_pin, LOW);
    }
    else if(pwm_count == pwmC_off)
    {
      digitalWrite(CH_pin, LOW);
      digitalWrite(CL_pin, HIGH);
    }
  }    
}
void inverse_clarke(float Vx_, float Vy_)
{
  /* svm compute */   
  Va = _2_divided_by_3 * Vx_;                                    
  Vb = (Vy_ - root_3_divided_by_3 * Vx_) / root_3;
  Vc = 2.0f * (Va - Vx_) - Vb;
  Vv = sqrt(Vx_ * Vx_ + Vy_ * Vy_);  
  duty_cycle = Vv / Vm;
  unit_vector = sqrt(Va * Va + Vb * Vb + Vc * Vc); 
  Va_unit = Va / unit_vector;
  Vb_unit = Vb / unit_vector;
  Vc_unit = Vc / unit_vector; 
  pwmA = (Va_unit - c) / m * duty_cycle;
  pwmB = (Vb_unit - c) / m * duty_cycle;
  pwmC = (Vc_unit - c) / m * duty_cycle;
}
/////////////////////////////////////////////////////
/* encoder */
// pins define
#define A_CH_pin 5
#define B_CH_pin 4
#define Z_CH_pin 6
// encoder rountine
#include <TimerThree.h>
#define max_motor_spd 480.0f  // unit -> rpm
#define ppr 2048.0f           // pulse per revolution
#define detect_freq_ratio 10  // actual freq = ratio * freq
const float encoder_freq =  max_motor_spd / 60.0f * ppr;
const float encoder_period = 1000000.0f / encoder_freq / detect_freq_ratio; // unit -> us
// parameter
#define pp 20.0f  // pole-pair of motor
const float deg_per_pul = 360.0f / ppr * pp;
const float deg_per_pul_rev = 360.0f / ppr;
float theta = 0, theta_rev = 0; // theta = rotor position, theta_rev = theta in revolution
volatile boolean A_rising = false;
volatile boolean Z_rising = false;
// alignment
#define align_freg 1000.0f
#define align_theta_res 0.001f // (+)value = ccw, (-)value  = cw
#define align_vector 10.0f
const float align_period = 1000.0f / align_freg;  // unit -> ms  
unsigned long align_prevTime = 0;
float index_theta, align_theta;

void encoder_event()
{
  if(digitalRead(Z_CH_pin) == HIGH && !Z_rising)
    Z_rising = true;
  if(digitalRead(Z_CH_pin) == LOW && Z_rising)
  {
    Z_rising = false;
    theta = index_theta;
    theta_rev = 0;
  }
  
  if(digitalRead(A_CH_pin) == HIGH && !A_rising)
    A_rising = true;        
  if(digitalRead(A_CH_pin) == LOW && A_rising)
  {
    A_rising = false;        
    if(digitalRead(B_CH_pin))
    {
      theta += deg_per_pul;
      theta_rev += deg_per_pul_rev;
    }
    else
    {
      theta -= deg_per_pul; 
      theta_rev -= deg_per_pul_rev;             
    }
    
         if(theta > 360)  theta = theta - 360.0f;
    else if(theta <   0)  theta = 360.0f + theta;
         if(theta_rev > 360)  theta_rev = theta_rev - 360.0f;
    else if(theta_rev <   0)  theta_rev = 360.0f + theta_rev;
  }  
}
void rotor_position_alignment()
{
  align_prevTime = millis();
  while(!digitalRead(Z_CH_pin))
  {
    if(millis() - align_prevTime <= align_period)
    {
      align_prevTime = millis();    
      align_theta += align_theta_res;
           if(align_theta_res > 0 && align_theta > 360) align_theta = align_theta - 360.0f;  
      else if(align_theta_res < 0 && align_theta < 0)   align_theta = 360.0 + align_theta;  
    }
    inverse_clarke(align_vector * cos(align_theta * deg_to_rad),
                   align_vector * sin(align_theta * deg_to_rad));
  }
  index_theta = align_theta;
  theta = index_theta; 
//  Serial.println(index_theta); 
  delay(500);   motor_stop = true; 
  delay(3000);  motor_stop = false;   
}
/////////////////////////////////////////////////////
/* curent sensor */
// config
#define sense_wait_start 2000 
#define BP_pin 14  
#define CP_pin 17
#define mV_A_ratio 100.0f   // typical ratio of +-20A range
#define R1_divider  4700.0f // unit -> ohm
#define R2_divider  6800.0f 
#define signal_input 5.0f  // feedback from asc712 
#define analog_res 12      // unit -> bit
const float analog_res_value = pow(2, analog_res);  // -> 0-4096
const float signal_output = signal_input * (R2_divider / (R1_divider + R2_divider)); 
const float divider_ratio  = signal_input / signal_output;
//float b_adc_top_range, b_adc_bottom_range;  // > top and bottom adc value range 
//float c_adc_top_range, c_adc_bottom_range;  // for calculate adc threshold value <
float b_adc_thres, c_adc_thres; // adc threshold value

// optimize
#include <Filters.h>
#define coef_phase_cutoff   10.0f
#define coef_clarke_cutoff  5.0f
#define coef_park_cutoff    2.0f
FilterOnePole Ib_LPF(LOWPASS, coef_phase_cutoff);
FilterOnePole Ic_LPF(LOWPASS, coef_phase_cutoff);
FilterOnePole Ix_LPF(LOWPASS, coef_clarke_cutoff);
FilterOnePole Iy_LPF(LOWPASS, coef_clarke_cutoff);
FilterOnePole Id_LPF(LOWPASS, coef_park_cutoff);
FilterOnePole Iq_LPF(LOWPASS, coef_park_cutoff);
// parameter
#define root_3_divided_by_2 0.866025404f 
#define _2_divided_by_3     0.666666667f  
#define root_3_divided_by_3 0.577350269f  
#define root_3              1.732050808f
#define deg_to_rad          0.017453292f 
#define align_time 200 // unit -> ms 
float theta_deg, theta_rad, cos_theta, sin_theta;
float Ib, Ic, Ia_f, Ib_f, Ic_f, Ib_offset, Ic_offset; 
float Ix, Iy, Ix_f, Iy_f;         // clarke transfrom frame
float Id, Iq, Id_f, Iq_f, Vd, Vq; // park transfrom frame

void phase_current_alignment()
{
  delay(sense_wait_start);
  
//  // useless phase as off  
//  digitalWrite(AH_pin,  LOW);
//  digitalWrite(AL_pin,  LOW);
//  // b phase as negative drive
//  // c phase as positive drive
//  digitalWrite(BH_pin,  LOW);
//  digitalWrite(BL_pin, HIGH);
//  digitalWrite(CH_pin, HIGH);
//  digitalWrite(CL_pin,  LOW);
//  delay(align_time);
//  b_adc_bottom_range = analogRead(BP_pin);
//  c_adc_top_range    = analogRead(CP_pin);
//  // c phase as negative drive
//  // b phase as positive drive
//  digitalWrite(BH_pin, HIGH);
//  digitalWrite(BL_pin, LOW);
//  digitalWrite(CH_pin, LOW);
//  digitalWrite(CL_pin, HIGH);
//  delay(align_time);
//  c_adc_bottom_range = analogRead(CP_pin);
//  b_adc_top_range    = analogRead(BP_pin);
//  // all phase as off 
//  digitalWrite(BH_pin, LOW);
//  digitalWrite(CL_pin, LOW);
//  
//  // offset calculation
//  b_adc_thres = (b_adc_top_range + b_adc_bottom_range) / 2.0f; 
//  c_adc_thres = (c_adc_top_range + c_adc_bottom_range) / 2.0f; 

    b_adc_thres = analogRead(BP_pin);
    c_adc_thres = analogRead(CP_pin);

//  Serial.print(b_adc_thres); Serial.print(" : "); 
//  Serial.println(c_adc_thres); 
//  delay(50000);
}
void get_phase_current()
{
  Ib = ((b_adc_thres - analogRead(BP_pin)) / analog_res_value * 3300.0f) * divider_ratio / mV_A_ratio;
  Ic = ((c_adc_thres - analogRead(CP_pin)) / analog_res_value * 3300.0f) * divider_ratio / mV_A_ratio;
}
void optimize_phase_current()
{
  Ib_LPF.input(Ib);
  Ic_LPF.input(Ic);
  Ib_f = Ib_LPF.output();
  Ic_f = Ic_LPF.output();
  Ia_f = -(Ib_f + Ic_f);
}
void optimize_clarke_current()
{
  Ix_LPF.input(Ix);
  Iy_LPF.input(Iy);
  Ix_f = Ix_LPF.output();
  Iy_f = Iy_LPF.output();
}
void optimize_park_current()
{
  Id_LPF.input(Id);
  Iq_LPF.input(Iq);
  Id_f = Id_LPF.output();
  Iq_f = Iq_LPF.output();
}
void clarke(float Ia_, float Ib_, float Ic_)
{
  Ix = Ia_ - Ib_ / 2.0f - Ic_ / 2.0f;             
  Iy = root_3_divided_by_2 * Ib_ - root_3_divided_by_2 * Ic_;
}
void park(float Ix_, float Iy_, float cos_, float sin_)
{
  Id = Ix_ * cos_ + Iy_ * sin_;
  Iq = Iy_ * cos_ - Ix_ * sin_;
}

/////////////////////////////////////////////////////
/* system */
// serial monitor
#define monitor_baudrate 4000000
// loop rountine
IntervalTimer loop_task;
#define loop_freq 500.0f  // unit -> Hz
const float loop_period = 1000000.0f / loop_freq; // unit -> us

/* debug */
#define dummy_vector 8.0f // maximum -> power supply voltage   
#define test_theta_res 1.0f
#define rev_per_time 1
float test_theta = 0; 
uint8_t pp_count = 0; // pole-pair counter

/////////////////////////////////////////////////////
/* control */
// config
#define Kp_d 3.825f  // <-- 0.53
#define Ki_d 0.000000f 
#define Kd_d 0.00f 
#define Kp_q 3.825f  //<--
#define Ki_q 0.000000f 
#define Kd_q 0.00f 
#define setpoint_d 0.0f
#define setpoint_q 5.5f
#define error_clearance 1.5
float error_d, 
      integral_d,
      derivative_d,
      prev_error_d;
float error_q, 
      integral_q,
      derivative_q,
      prev_error_q;

void PI_control(float Id_, float Iq_)
{
  /* PI Id control */  
  error_d = setpoint_d - Id_;
  integral_d += error_d;
//  if(error_d > -error_clearance && error_d < error_clearance)
//    integral_d = 0;
  derivative_d = error_d - prev_error_d;
  Vd = Kp_d * error_d + Ki_d * integral_d + Kd_d * derivative_d; 
  prev_error_d = error_d;
  /* PI Iq control */
  error_q = setpoint_q - Iq_;
  integral_q += error_q;
//  if(error_q > -error_clearance && error_q < error_clearance)
//    integral_q = 0;
  derivative_q = error_q - prev_error_q;
  Vq = Kp_q * error_q + Ki_q * integral_q + Kd_q * derivative_q; 
  prev_error_q = error_q;
}
void inverse_park(float Vd_, float Vq_, float cos_, float sin_, float theta)
{
  Vx = Vd_ * cos_ - Vq_ * sin_;  
  if(int(theta)!=0 && int(theta)!=180)
  {
    Vy = (Vd_ - Vx * cos_) / sin_; 
    // possible nan -> sin(0 or 180) = 0     
  }
}
/////////////////////////////////////////////////////

void loop_rountine()
{
  /* get theta in encoder event */
//  theta_deg = theta;  
//  theta_deg = theta + 90.0f;
//       if(theta_deg > 360) theta_deg = theta_deg - 360.0f;
//  else if(theta_deg < 0  ) theta_deg = 360.0f + theta_deg;  
//  
//  theta_rad = theta_deg * deg_to_rad;
//  cos_theta = cos(theta_rad);
//  sin_theta = sin(theta_rad);




//  /* open loop drive */   
//  test_theta += test_theta_res; 
//  if(test_theta > 360)
//  {
//    test_theta = test_theta - 360.0f;  
////    test_theta = 360 + test_theta; 
//    pp_count++;
//  }
//  if(pp_count > (pp * rev_per_time) - 1) 
//  {
//    motor_stop = true;
//    pp_count = 0;
//    loop_task.end();
//  }
//  Vx = dummy_vector * cos(test_theta * deg_to_rad);  
//  Vy = dummy_vector * sin(test_theta * deg_to_rad);
//  inverse_clarke(Vx, Vy);




//  theta_deg += 90.0f;
//  theta_rad = theta_deg * deg_to_rad;
//  cos_theta = cos(theta_rad);
//  sin_theta = sin(theta_rad);

//  PI_control(Id_f, Iq_f);
//  inverse_park(Vd, Vq, cos_theta, sin_theta, theta_deg);  

  
  
  /* position control test! */
  #define Kp 1.8f
  float th, p, error;
  float sp = map(analogRead(4), 0, analog_res_value, 10, 350);

  p = sp - theta_rev;
  p *= Kp;
  if(p > 0)
    th = theta + 90;
  else
    th = theta - 90;
  
  p = abs(p);
       if(p > 12) p = 12.0f;
  else if(p < 0)  p = 0.0f;
  
       if(th > 360) th = th - 360.0f;
  else if(th < 0  ) th = 360.0f + th;  
  Vx = p * cos(th * deg_to_rad);  
  Vy = p * sin(th * deg_to_rad);
 
  
  
//  /* sinusoidal control */
//  const float out_vect = 12.0f; 
//  float th = theta + 90.0f;
//       if(th > 360) th = th - 360.0f;
//  else if(th < 0  ) th = 360.0f + th;  
//  Vx = out_vect * cos(th * deg_to_rad);  
//  Vy = out_vect * sin(th * deg_to_rad);
  
  inverse_clarke(Vx, Vy);


//  float It = sqrts(Ix * Ix + Iy * Iy);
//  float sine_wave = sin(theta * deg_to_rad);
  
//  Serial.print(Id);Serial.print("\t");
//  Serial.println(theta_rev);

}
/////////////////////////////////////////////////////

void setup() 
{
  
  /* pins config */ 
  // mosfet 
  pinMode(AH_pin, OUTPUT);
  pinMode(AL_pin, OUTPUT);
  pinMode(BH_pin, OUTPUT);
  pinMode(BL_pin, OUTPUT);
  pinMode(CH_pin, OUTPUT);
  pinMode(CL_pin, OUTPUT);
  // encoder 
  pinMode(A_CH_pin, INPUT);
  pinMode(B_CH_pin, INPUT);
  pinMode(Z_CH_pin, INPUT);

  /* current sensor */
  analogReadResolution(analog_res);
  phase_current_alignment();
  
//  Serial.println(analog_threshold); delay(3000);

//  // alignment 

//  delay(sense_wait_start);
//  Ia_offset = Ia;
//  Ib_offset = Ib;
  
   
  /* svm */
  Timer1.initialize(pwm_period);
  Timer1.attachInterrupt(svm_generate);

  /* encoder */
  rotor_position_alignment();
  test_theta = index_theta;
  // interrupt
  Timer3.initialize(encoder_period);
  Timer3.attachInterrupt(encoder_event);
  
 /* system */
  // monitor
  Serial.begin(monitor_baudrate);
  // loop
  loop_task.begin(loop_rountine, loop_freq);
  
} 

void loop() 
{
  get_phase_current();                optimize_phase_current();
  clarke(Ia_f, Ib_f, Ic_f);
  park(Ix, Iy, cos_theta, sin_theta); optimize_park_current();
  
  if(motor_stop)
  {    
    delay(2000);
    motor_stop = false;
    loop_task.begin(loop_rountine, loop_freq);
  }
  delay(5);  
}
