
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
#define B_CH_pin 6
#define Z_CH_pin 4
// encoder rountine
#include <TimerThree.h>
#define max_motor_spd 180.0f  // unit -> rpm
#define ppr 3600.0f           // pulse per revolution
#define detect_freq_ratio 10  // actual freq = ratio * freq
const float encoder_freq =  max_motor_spd / 60.0f * ppr;
const float encoder_period = 1000000.0f / encoder_freq / detect_freq_ratio; // unit -> us
// parameter
#define pp 20.0f  // pole-pair of motor
const float deg_per_pul = 360.0f / ppr * pp;
float theta = 0;
volatile boolean rising_edge = false;

void encoder_event()
{
  if(digitalRead(A_CH_pin) == HIGH && !rising_edge)
  {
    rising_edge = true;        
  }
  if(digitalRead(A_CH_pin) == LOW && rising_edge)
  {
    rising_edge = false;        
    if(digitalRead(B_CH_pin)) theta -= deg_per_pul;
    else                      theta += deg_per_pul;  

         if(theta > 360)  theta = deg_per_pul;
    else if(theta <   0)  theta = 360.0f - deg_per_pul;
  }  
}

/////////////////////////////////////////////////////
/* curent sensor */
// config
#define sense_baudrate 9600
#define sense_timeout 50
#define sense_wait_start 1000 
// optimize
#include <Filters.h>
#define coef_phase_cutoff   5.0f
#define coef_clarke_cutoff  5.0f
#define coef_park_cutoff    0.5f
FilterOnePole Ia_LPF(LOWPASS, coef_phase_cutoff);
FilterOnePole Ib_LPF(LOWPASS, coef_phase_cutoff);
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
float theta_deg, theta_rad, cos_theta, sin_theta;
float Ia, Ib, Ia_f, Ib_f, Ic_f, Ia_offset, Ib_offset; 
float Ix, Iy, Ix_f, Iy_f;         // clarke transfrom frame
float Id, Iq, Id_f, Iq_f, Vd, Vq; // park transfrom frame

volatile boolean sense_update = false;

//void serialEvent1()
//{
//  if(Serial1.find('#'))
//  {
//    float Ia_buf = Serial1.parseFloat(); 
//    float Ib_buf = Serial1.parseFloat();     
//    int sum_buf  = (int)Serial1.parseFloat();
//    int sum = Ia_buf + Ib_buf; 
//    if(sum_buf == sum)
//    {
//      Ia = Ia_buf;
//      Ib = Ib_buf;
//      sense_update = true;
//    }
//    Serial1.flush();
//  }
//}
void optimize_phase_current()
{
  if(sense_update)
  {
    Ia_LPF.input(Ia - Ia_offset);
    Ib_LPF.input(Ib - Ib_offset);
    Ia_f = Ia_LPF.output();
    Ib_f = Ib_LPF.output();
    Ic_f = -(Ia_f + Ib_f);
  }
}
void optimize_clarke_current()
{
  if(sense_update)
  {
    Ix_LPF.input(Ix);
    Iy_LPF.input(Iy);
    Ix_f = Ix_LPF.output();
    Iy_f = Iy_LPF.output();
  }
}
void optimize_park_current()
{
  if(sense_update)
  {
    Id_LPF.input(Id);
    Iq_LPF.input(Iq);
    Id_f = Id_LPF.output();
    Iq_f = Iq_LPF.output();
  }
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
#define dummy_vector 3.0f // maximum -> power supply voltage   
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
#define setpoint_q 3.5f
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
  theta_deg = theta;
  theta_rad = theta_deg * deg_to_rad;
  cos_theta = cos(theta_rad);
  sin_theta = sin(theta_rad);
  
//  /* get phase current in serialEvent1 */ optimize_phase_current(); 
//  clarke(Ia_f, Ib_f, Ic_f);               optimize_clarke_current();
//  park(Ix_f, Iy_f, cos_theta, sin_theta); optimize_park_current();
  
  
  
  test_theta += test_theta_res; 
  if(test_theta > 360)
  {
    test_theta = test_theta_res;  
    pp_count++;
  }
  if(pp_count > (pp * rev_per_time) - 1)
  {
    motor_stop = true; 
//    Timer3.stop();
    pp_count = 0;
    test_theta = 0;
    loop_task.end();
  }

  float theta_rad = test_theta * deg_to_rad;
  Vx = dummy_vector * cos(theta_rad);  
  Vy = dummy_vector * sin(theta_rad);
  inverse_clarke(Vx, Vy);

  
//  PI_control(Id_f, Iq_f);
//  inverse_park(Vd, Vq, cos_theta, sin_theta, theta_deg);  
//  inverse_clarke(Vx, Vy);
//  
//  sense_update = false;
   

//  float sine_wave = sin(theta * deg_to_rad);
//  Serial.print(Id_f);Serial.print("\t");

//  Serial.print(Id_f);Serial.print("\t");
//  Serial.println(analogRead(8));
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
  
  /* svm */
  Timer1.initialize(pwm_period);
  Timer1.attachInterrupt(svm_generate);

//  /* encoder */
//  // interrupt
//  Timer3.initialize(encoder_period);
//  Timer3.attachInterrupt(encoder_event);
//  // alignment
//  unsigned long prevTime = millis();
//  while(millis() - prevTime <= 2000)
//  {
//    inverse_clarke(6.0f, 0.0f);
//    theta = 0;
//  }
//  
//  /* current sensor */
//  // communicate
//  Serial1.begin(sense_baudrate);
//  Serial.setTimeout(sense_timeout);
//  // alignment 
//  delay(sense_wait_start);
//  Ia_offset = Ia;
//  Ib_offset = Ib;
  
  /* system */
  // monitor
  Serial.begin(monitor_baudrate);
  // loop
  loop_task.begin(loop_rountine, loop_freq);
  analogReadResolution(12);
} 

void loop() 
{
  if(motor_stop)
  {    
    delay(2000);
    motor_stop = false;
//    theta = 0;
//    Timer3.start();
    loop_task.begin(loop_rountine, loop_freq);
  }
  delay(50);  
}
