
/*  Gate pin  */ 
//  A Phase - HIGH = D9   LOW = D7
//  B Phase - HIGH = D10, LOW = D8
//  C Phase - HIGH = D11, LOW = D12

/* Selector multipexer  */
//  BEMF A - A0
//  BEMF B - A1
//  BEMF C - A2
//  Positive comparator - D6

void gate_init()
{ /* Setup pin for gate control */
  /* Phase A - Pin setup */
  DDRD |= 0x80;       //  Config D7 as output.
  DDRB |= 0x02;       //  Config D9(PWM) as output.
  PORTD |= (1<<PD7);  //  Set default digital output.
  TCCR1A = 0x01;      //  Config PWM mode as phase correct. 
  TCCR1B = 0x01;      //  0x01 = Config timer1 prescaler as 1 (31.25 kHz).
  OCR1A = 0;          //  Set default PWM.  
  /* Phase B - Pin setup */
  DDRB |= 0x05;       //  Config D8, D10(PWM) as output.
  PORTB |= (1<<PB0);  //  Set default digital output.
  TCCR1A = 0x01;      //  Config PWM mode as phase correct. 
  TCCR1B = 0x01;      //  0x01 = Config timer1 prescaler as 1 (31.25 kHz).
  OCR1B = 0;          //  Set default PWM.
  /* Phase C - Pin setup */
  DDRB |= 0x18;       //  Config D12, D11(PWM) as output.
  PORTB |= (1<<PB4);  //  Set default digital output.
  TCCR2A = 0x01;      //  Config PWM mode as phase correct. 
  TCCR2B = 0x01;      //  0x02 = Config timer0 prescaler as 32 (3906.25 Hz).
  OCR2A = 0;          //  Set default PWM.
}
void A_POS()
{ /* Phase A - Positive */
  TCCR1A = 0x81;      // Enable PWM on D9.
  PORTD |= (1<<PD7);  // Set high on D7.
}
void A_OFF()
{ /* Phase A - Floating */
  TCCR1A = 0;          // Disable PWM on D9.
  PORTD |= (1<<PD7);   // Set high on D7.
}
void A_NEG()
{ /* Phase A - Negative */
  TCCR1A = 0;         // Disable PWM on D9.
  PORTD &= ~(1<<PD7); // Set low on D7.  
}
void B_POS()
{ /* Phase B - Positive */
  TCCR1A = 0x21;      //  Enable PWM on D10.
  PORTB |= (1<<PB0);  //  Set high on D8.
}
void B_OFF()
{ /* Phase B - Floating */
  TCCR1A = 0;         // Disable PWM on D10.
  PORTB |= (1<<PB0);  // Set high on D8.
} 
void B_NEG()
{ /* Phase B - Negative */
  TCCR1A = 0;         // Disable PWM on D10.
  PORTB &= ~(1<<PB0); // Set low on D8.  
}
void C_POS()
{ /* Phase C - Positive */
  TCCR2A = 0x81;      //  Enable PWM on D11.
  PORTB |= (1<<PB4);  //  Set high on D8.
}
void C_OFF()
{ /* Phase C - Floating */
  TCCR2A = 0;         // Disable PWM on D11.
  PORTB |= (1<<PB4);  // Set high on D8. 
}
void C_NEG()
{ /* Phase C - Negative */
  TCCR2A = 0;         // Disable PWM on D11.
  PORTB &= ~(1<<PB4); // Set low on D8. 
}
void A_RISE()
{ /* Step2 - A floating */ 
  ACSR |=  (1<<ACIS0);
  ACSR |=  (1<<ACIS1);  //  Config interrupt on rising edge.
  ADMUX = 0x00;         //  Select A0 to comparator.
}
void A_FALL()
{ /* Step5 - A floating */ 
  ACSR &= ~(1<<ACIS0);  
  ACSR |=  (1<<ACIS1);  //  Config interrupt on falling edge.
  ADMUX = 0x00;         //  Select A0 to comparator.
}
void B_RISE()
{ /* Step4 - B floating */ 
  ACSR |=  (1<<ACIS0);
  ACSR |=  (1<<ACIS1);  //  Config interrupt on rising edge.
  ADMUX = 0x01;         //  Select A1 to comparator.
}
void B_FALL()
{ /* Step1 - B floating */ 
  ACSR &= ~(1<<ACIS0);  
  ACSR |=  (1<<ACIS1);  //  Config interrupt on falling edge.
  ADMUX = 0x01;         //  Select A1 to comparator.
}
void C_RISE()
{ /* Step0 - C floating */  
  ACSR |=  (1<<ACIS0);
  ACSR |=  (1<<ACIS1);  //  Config interrupt on rising edge.
  ADMUX = 0x02;         //  Select A2 to comparator.
}
void C_FALL()
{ /* Step3 - C floating */  
  ACSR &= ~(1<<ACIS0);  
  ACSR |=  (1<<ACIS1);  //  Config interrupt on falling edge.
  ADMUX = 0x02;         //  Select A2 to comparator.
}
void bemf_init()
{ /* Analog Comparator Interrupt setup. */
  ADCSRA &= ~(1<<ADEN); //  Disable the ADC module.
  ADCSRB |=  (1<<ACME); //  Enable MUX selector.
  ACSR &= ~(1<<ACIE);   //  Disable analog comparator interrupt.
  ACSR |= (1<<ACI);     //  Clear interrupt flag.
}
void set_pwm(uint8_t _pwm)
{ /* Set all value of compare match on wave form */
  OCR1A = _pwm;  //  Set A phase PWM (D9)
  OCR1B = _pwm;  //  Set B phase PWM (D10)
  OCR2A = _pwm;  //  Set C phase PWM (D11)
}
void set_step(uint8_t _step)
{ /* Step manager */
  switch(_step)
  {
    case 0:
      B_NEG();  C_OFF();  A_POS();  C_RISE();  break;
    case 1:
      C_NEG();  B_OFF();  A_POS();  B_FALL();  break;
    case 2:
      C_NEG();  A_OFF();  B_POS();  A_RISE();  break;  
    case 3:
      A_NEG();  C_OFF();  B_POS();  C_FALL();  break;
    case 4:
      A_NEG();  B_OFF();  C_POS();  B_RISE();  break;
    case 5:
      B_NEG();  A_OFF();  C_POS();  A_FALL();  break;
  }
}
