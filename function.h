
/*  Gate pin  */ 
//  A Phase - HIGH = D5,  LOW = D4
//  B Phase - HIGH = D3,  LOW = D7
//  C Phase - HIGH = D11, LOW = D8

/* Selector multipexer  */
//  BEMF A - A0
//  BEMF B - A1
//  BEMF C - A2
//  Positive comparator - D6

/* Input PWM pin - D2 */

void A_SET()
{ /* Phase A - Pin setup */
  DDRD |= 0x30;       //  Config D4, D5(PWM) as output.
  PORTD |= (1<<PD4);  //  Set default digital output.
  TCCR0A = 0x01;      //  Config PWM mode as phase correct. 
  TCCR0B = 0x01;      //  0x02 = Config timer0 prescaler as 32 (3906.25 Hz).
  OCR0B = 0;          //  Set default PWM.
}
void A_POS()
{ /* Phase A - Positive */
  TCCR0A = 0x21;       // Enable PWM on D5.
  PORTD |= (1<<PD4);   // Set high on D4.
}
void A_OFF()
{ /* Phase A - Floating */
  TCCR0A = 0;          // Disable PWM on D5.
  PORTD |= (1<<PD4);   // Set high on D4.
}
void A_NEG()
{ /* Phase A - Negative */
  TCCR0A = 0;         // Disable PWM on D5.
  PORTD &= ~(1<<PD4); // Set low on D4.  
}
void B_SET()
{ /* Phase B - Pin setup */
  DDRD |= 0x88;       //  Config D7, D3(PWM) as output.
  PORTD |= (1<<PD7);  //  Set default digital output.
  TCCR2A = 0x01;      //  Config PWM mode as phase correct. 
  TCCR2B = 0x01;      //  0x02 = Config timer0 prescaler as 32 (3906.25 Hz).
  OCR2B = 0;          //  Set default PWM.
}
void B_POS()
{ /* Phase B - Positive */
  TCCR2A = 0x21;      //  Enable PWM on D3.
  PORTD |= (1<<PD7);  //  Set high on D7.
}
void B_OFF()
{ /* Phase B - Floating */
  TCCR2A = 0;         // Disable PWM on D3.
  PORTD |= (1<<PD7);  // Set high on D7.
} 
void B_NEG()
{ /* Phase B - Negative */
  TCCR2A = 0;         // Disable PWM on D3.
  PORTD &= ~(1<<PD7); // Set low on D7.  
}
void C_SET()
{ /* Phase C - Pin setup */
  DDRB |= 0x09;       //  Config D8, D11(PWM) as output.
  PORTB |= (1<<PB0);  //  Set default digital output.
  TCCR2A = 0x01;      //  Config PWM mode as phase correct. 
  TCCR2B = 0x01;      //  0x02 = Config timer0 prescaler as 32 (3906.25 Hz).
  OCR2A = 0;          //  Set default PWM.
}
void C_POS()
{ /* Phase C - Positive */
  TCCR2A = 0x81;      //  Enable PWM on D11.
  PORTB |= (1<<PB0);  //  Set high on D8.
}
void C_OFF()
{ /* Phase C - Floating */
  TCCR2A = 0;         // Disable PWM on D11.
  PORTB |= (1<<PB0);  // Set high on D8. 
}
void C_NEG()
{ /* Phase C - Negative */
  TCCR2A = 0;         // Disable PWM on D11.
  PORTB &= ~(1<<PB0); // Set low on D8. 
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
  ACSR |= (1<<ACI);     //  Clear interrupt flag.
  ACSR |= (1<<ACIE);    //  Enable analog comparator interrupt.
}
void set_pwm(uint8_t _pwm)
{ /* Set all value of compare match */
  OCR0B = _pwm;  //  Set A phase PWM (D5)
  OCR2B = _pwm;  //  Set B phase PWM (D3)
  OCR2A = _pwm;  //  Set C phase PWM (D11)
}
void set_step(uint8_t _step)
{ /* Step manager */
  switch(_step)
  {
    case 0:
      A_POS();  B_NEG();  C_OFF();  C_RISE(); break;
    case 1:
      A_POS();  B_OFF();  C_NEG();  B_FALL(); break;
    case 2:
      A_OFF();  B_POS();  C_NEG();  A_RISE(); break;  
    case 3:
      A_NEG();  B_POS();  C_OFF();  C_FALL(); break;
    case 4:
      A_NEG();  B_OFF();  C_POS();  B_RISE(); break;
    case 5:
      A_OFF();  B_NEG();  C_POS();  A_FALL(); break;
  }
}
void input_init()
{ /* Setup external interrupt for switch reading */
  EIMSK |= (1<<INT0); //  Config interrupt pin.
  EICRA = 0x03;       //  Set default interrupt on rising edge.  
                      //  0x02 as falling edge and 0x03 as rising edge. 
}
