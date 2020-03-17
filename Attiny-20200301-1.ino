// set timer to 0.016s
// WDTCR |= (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
// set timer to 0.032s
// WDTCR |= (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
// set timer to 0.064s
// WDTCR |= (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (0<<WDP0);
// set timer to 0.125s
// WDTCR |= (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0);
// set timer to 0.250s
// WDTCR |= (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0);
// set timer to 0.5s
// WDTCR |= (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);
//set timer to 1 sec
//WDTCR |= (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0);
//set timer to 2 sec
//WDTCR |= (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
//set timer to 4 sec
//WDTCR |= (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
//set timer to 8 sec
//WDTCR |= (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);

    // WDP3 WDP2 WDP1 WDP0 Typical Time-out at VCC = 5.0V
    // 0 0 0 0 16ms
    // 0 0 0 1 32ms
    // 0 0 1 0 64ms
    // 0 0 1 1 0.125 s
    // 0 1 0 0 0.25 s
    // 0 1 0 1 0.5 s
    // 0 1 1 0 1.0 s
    // 0 1 1 1 2.0 s
    // 1 0 0 0 4.0 s
    // 1 0 0 1 8.0 s




#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>


const short  nPulseLength = 350;

const short  nHighPulses_0 = (nPulseLength * 1);
const short nLowPulses_0 = (nPulseLength * 3);

const short nHighPulses_1 = (nPulseLength * 3);
const short nLowPulses_1 = (nPulseLength * 1);

const short nLowPulses_sync =  (nPulseLength * 31);

#define PIN_TX    (1<<PB3) // PB3 pin, goes to transmitter data pin, UNO pin 11
#define PIN_RFPW   (1<<PB4) // PB4 pin, for RF transmitter power, UNO pin 12. We will power transmitter only during the transmission.


const int detectorPin = 0; //PB0 is INT0 pin
const int statusLED = 1;   //LED for some visual indication.In PCB use a jumper to isolate LED to save battery.

bool isIinterrupt = false;
volatile byte time_counter = 0;

void send(char* sCodeWord) {

  while (*sCodeWord != '\0') {

    PORTB |= PIN_TX; // same as digitalWrite high

    if (*sCodeWord == '0')
    {
      _delay_us(nHighPulses_0);
      PORTB &= ~PIN_TX; // same as digitalWrite low
      _delay_us(nLowPulses_0);
    } else
    {
      _delay_us(nHighPulses_1);
      PORTB &= ~PIN_TX;
      _delay_us(nLowPulses_1);
    }

    ++sCodeWord;
  }

  PORTB |= PIN_TX;
  _delay_us(nHighPulses_0);

  PORTB &= ~PIN_TX;
  _delay_us(nLowPulses_sync);
}



void setup() {
  DDRB |= PIN_TX; // Set output direction on PIN_TX
  DDRB |= PIN_RFPW; // Set output direction on PIN_RFPW
  pinMode(detectorPin, INPUT);
  pinMode(statusLED, OUTPUT);

  alive();  //this RF code tells receiver that ckt just got alive or rebooted.
  checkDoor();

  // Flash quick sequence so we know setup has started
    for (int k = 0; k < 10; k = k + 1) {
        if (k % 2 == 0) {
            digitalWrite(statusLED, HIGH);
            }
        else {
            digitalWrite(statusLED, LOW);
            }
        delay(250);
        } // for
}

void sleep()
{
  GIMSK |= _BV(PCIE); // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT0); // Use PB0 as interrupt pin
  ADCSRA &= ~_BV(ADEN); // ADC off

 
 // WDTCR |= (1 << WDP3); // (1<<WDP2) | (1<<WDP0);   //4s

  WDTCR |= (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0); //8sec

  // Enable watchdog timer interrupts
  WDTCR |= (1 << WDTIE);


  // turn off brown-out enable in software
  // BODS must be set to one and BODSE must be set to zero within four clock cycles
  MCUCR = bit (BODS) | bit (BODSE);
  // The BODS bit is automatically cleared after three clock cycles
  MCUCR = bit (BODS);



  MCUCR |= _BV(SM1); MCUCR &= ~_BV(SM0); // Select "Power-down" sleep mode
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Another way to Select "Power-down" sleep mode

  sleep_enable(); // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)

  sei(); // Enable interrupts
  sleep_cpu(); // SLEEP

  cli(); // Disable interrupts
  PCMSK &= ~_BV(PCINT0); // Turn off PB0 as interrupt pin
  sleep_disable(); // Clear SE bit
  ADCSRA |= _BV(ADEN); // ADC on

  sei(); // Enable interrupts
}



ISR(PCINT0_vect) {
  isIinterrupt = true; //Increment isIinterrupt and use for LED flashing.On interrupt detection main loop will be processed.
}

ISR(WDT_vect) {
  time_counter++;
}

void loop() {

  if (isIinterrupt) {
    checkDoor();
    isIinterrupt = false;
  }

  if (time_counter > 60) { //60 is approx 8-9Minute, not accurate
    checkDoor();
    for (int i = 0; i < time_counter; i++) {
      digitalWrite(statusLED, HIGH);
      delay(50);
      digitalWrite(statusLED, LOW);
      delay(50);
    }
    time_counter = 0;
  } // ca. 8 minutes (8s x 60=480sec)

  sleep();
}


void alive() {
    PORTB |= PIN_RFPW; // power on transmitter
  for (byte i = 0; i < 15; ++i)
  {
    send("00000111"); //1011101000110101
  }
  PORTB &= ~PIN_RFPW; // power off transmitter
  time_counter = 0;
}

void checkDoor() {
  PORTB |= PIN_RFPW; // power on transmitter
  _delay_ms(100); // let the debounce settle
  if (digitalRead(detectorPin)) {
    for (byte i = 0; i < 15; ++i)
    {
      send("00000100"); //1011101000110101
    }
      for (int i = 0; i < 1; i++) {
      digitalWrite(statusLED, HIGH);
      delay(50);
      digitalWrite(statusLED, LOW);
      delay(50);
    }
  }
  else {
    for (byte i = 0; i < 15; ++i)
    {
      send("00000101"); //1011101000110101
    }
        for (int i = 0; i < 2; i++) {
      digitalWrite(statusLED, HIGH);
      delay(50);
      digitalWrite(statusLED, LOW);
      delay(50);
    }
  }
  
  PORTB &= ~PIN_RFPW; // power off transmitter
}
