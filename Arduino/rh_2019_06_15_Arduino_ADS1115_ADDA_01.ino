/*
Test program for ADS115 attached to arduino.
Output via COM port +/- PCF8591 module by I2C.

Main features:
output via COM-port:
  - convert data with ADS1115
  - simulate data conversion by synthetic data
    (data format is choosable to be conformant with SpectrumLab).
output via PCF8591:
  - additionally you may attach a PCF8591 module by I2C

Conversion rate:
  - Conversion rate is controlled by flags set by an interrupt handler.


ADC with ADS1115
---------------
A ADS1115 is usable by the Adafruit_ADS1115 library.

ADC simulated by synthetic sinus curve
--------------------------------------
A synthetic sinus curve (int16_t sinetable [50]) is fed into
COM port to simulate a AD-conversion.

Adjustable conversion rate
--------------------------
The conversion rate is realized by an interrupt routine called
at a rate of 10.000 Hz. Interrupts are counted.
Within the main loop the flags are continously checked. If a
predefined threshold of the counter is reached the function >send_data()<
is called and the data is sent.

Analog output by PCF8591
------------------------
There is a hardware ADDA-Module (PCF8591) that converts the date
to an analog output. This analog value can be checked by an oscilloscope.
Note that the PCF8591 needs 8-Bit data (the ADS1115 is 16 bit ).

LED Output
----------
Additionally the LED at pin 13 can be toggled to check other
events. An example is the moment, when the analog value is
sent to the COM port. Pin 13 too may be contolled by an
oscilloscope.

Note: You can watch the data by the serial monitor of the Arduino IDE.

ASCII        ( output for serial monitor of Arduino IDE)
S16          ( == signed int (16 Bit) for Spectrum Lab.
+/- PCF8591  ( ADDA Module: analog output)
+/- LED      ( PIN 13 )

SpectrumLab
-----------
This program was originally developped to feed data into SpectrumLab
via COM port. Especially with low frequencies SpectrumLab behaves in
some way unpredictable (my experience) if data is fed by serial line.
Nevertheless, you may give it a try, setting >serial_mode == serial_SpecLab<.



timer preset according to
  https://www.heise.de/developer/artikel/Timer-Counter-und-Interrupts-3273309.html

  initcount = maxcount - (cpufreq / deltaT)  / prescale

  examples:
  initcount = 65536 - (16000000/20)/256  = 62411   // prescale == 256! ; f == 1/40 sec
  initcount = 65536 - (16000000/10000)/8 = 65336   // prescale ==   8! ; f == 1/10000 sec

note:
There are some unused variables and some unused lines of code...

*/

// ==================================================================================================

int16_t sinetable [50]  = {
  0, 1369, 2716, 4021, 5262, 6420, 7477, 8416, 9222, 9883, 10388,
  10729, 10901, 10901, 10729, 10388, 9883, 9222, 8416, 7477, 6420,
  5262, 4021, 2716, 1369, 0,
  -1368, -2716, -4020, -5261, -6420, -7476, -8415, -9222, -9882,
  -10387, -10729, -10900, -10900, -10729, -10387, -9882, -9222,
  -8415, -7476, -6420, -5261, -4020, -2716, -1368
};
int  idx_sinetable;  // idx of sinetable

#define ledPin 13
int16_t led_status;

// outp_mode == data format transmitted via serial IO: ASCII or signed int
#define ascii   1  // ASCII output via (e.g. Arduino IDE serial monitor)
#define S16     6  // for SpectrumLab: signed int * little endian * +/-32767 * 0x8000
int16_t outp_mode   ;

// data mode == use real AD-data or sinetable
#define synt_data 0  // use sinetable as data
#define real_data 1  // use real AD-data
int16_t data_mode     ;  // input data mode: sinetable oder AD-data

// serial mode == configure COM port
#define serial_SpecLab  0     // For communication with SpectrumLab
#define serial_jAmaseis 1     // For use with jAmaseis
int16_t serial_mode ;         // serial mode SpectrumLab or jAmaseis

// when S16 (for SpectrumLab) : sync pattern
unsigned int sync_patt ;  // sync    pattern
unsigned int repl_patt ;  // replace pattern

// when AD-data: convert channel 1 v 2.
const int16_t ch_1 = 0;  // ch 1 == Geofono 1
const int16_t ch_2 = 1;  // ch 2 == Geofono 2

// buffer for string output
char      buf[120];  // char buffer

// some counters
int  cnt_1_Hz ;      // count number for 1 Hz Signal
int  cnt_val  ;      // counts number of values since last interrupt

// vars for calculating average values
int  act_val;        // actual ADC value == signed int
int  sum_val;        // sum           of values since last interrupt
int  avg_val;        // average value == sum_val / cnt_val

// for communicating via I2C with PCF8591 ADDA-module
#include "Wire.h"
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;       // Use this for the 16-bit version

#define PCF8591 (0x90 >> 1) // I2C bus address

// some vars to scale int16_t to byte
int16_t tmp_val     ;
int16_t scaled_val  ;       // skalierter Wert

byte byte_low  ;            //
byte byte_high ;            //
byte byte_array[2]={0,0};   // global; returns result of function >void scale_to_8_bit(int16_t tmp_val)<

volatile int16_t ADS_result; // result from ADS1x15

volatile int16_t cnt_0_1_ms  ;

volatile int16_t cnt_1_ms    ;
volatile int16_t cnt_10_ms   ;
volatile int16_t cnt_100_ms  ;
volatile int16_t cnt_1000_ms ;

volatile int16_t cnt_20_ms  = 0 ;
volatile int16_t cnt_40_ms  = 0 ;
volatile int16_t cnt_50_ms  = 0 ;
volatile int16_t cnt_500_ms = 0 ;

volatile bool flag_1_ms    ;
volatile bool flag_2_ms    ;
volatile bool flag_4_ms    ;
volatile bool flag_8_ms    ;
volatile bool flag_10_ms   ;
volatile bool flag_100_ms  ;
volatile bool flag_1000_ms ;

// lets start!

void setup()
{
  Wire.begin();

  // outp_mode   = S16   ;                 // output mode
  // serial_mode = serial_SpecLab ;        // serial mode for SpectrumLab
  // data_mode   = synt_data ;             // synthetic data

  outp_mode   = ascii ;                 // output mode
  serial_mode = serial_jAmaseis ;       // serial mode for jAmaSeis
  data_mode   = real_data ;             // data from seismometer
  // data_mode   = synt_data ;             // synthetic data

  if (serial_mode == serial_SpecLab){
    Serial.begin(115200);               // SpectrumLab: Options > Audio I/O:
                                        //   Input Device: params 115200,8-N-1,S16,SYNC,1
  } else if (serial_mode == serial_jAmaseis) {
    //Serial.begin(19200);                 // jAmaseis
    Serial.begin(9600);                 // jAmaseis
  } else {
    Serial.begin(115200);   //
  }

  cnt_val       = 0 ;
  sum_val       = 0 ;
  avg_val       = 0 ;

  cnt_0_1_ms    = 0 ;
  cnt_1_ms      = 0 ;
  cnt_10_ms     = 0 ;
  cnt_100_ms    = 0 ;
  cnt_1000_ms   = 0 ;
  cnt_1_Hz      = 0 ;

  cnt_50_ms     = 0 ;
  cnt_500_ms    = 0 ;

  flag_1_ms     = false ;
  flag_2_ms     = false ;
  flag_4_ms     = false ;
  flag_8_ms     = false ;
  flag_10_ms    = false ;
  flag_100_ms   = false ;
  flag_1000_ms  = false ;

  pinMode(ledPin, OUTPUT);              // LED Pin

  // ADS1115
  //Adafruit_ADS1015 ads;               // Use thi for the 12-bit version
  ads.setGain(GAIN_SIXTEEN);            // ADS1115: 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.begin();                          // ADS1115

  // Set Timer #1
  noInterrupts();                       // diasable interrupts temporarily
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 65336;  // preset timer: 1/10.000 repeat statement interrupt routine !

  // PRESCALER: https://www.heise.de/developer/artikel/Timer-Counter-und-Interrupts-3273309.html
  // Prescale = 0:    TCCR1B = 0; TCCR1B |= (1 << CS10);
  // Prescale = 8:    TCCR1B = 0; TCCR1B |= (1 << CS11);
  // Prescale = 64:   TCCR1B = 0; TCCR1B |= (1 << CS10); TCCR1B |= (1 << CS11);
  // Prescale = 256:  TCCR1B = 0; TCCR1B |= (1 << CS12);
  // Prescale = 1024: TCCR1B = 0; TCCR1B |= (1 << CS10); TCCR1B |= (1 << CS12);

  TCCR1B = 0; TCCR1B |= (1 << CS11);               // Prescaler:   8

  TIMSK1 |= (1 << TOIE1);               // enable Timer Overflow Interrupt
  interrupts();                         // enable interrupts

  if (outp_mode == ascii) {
    ;
  } else if (outp_mode == S16) {
    sync_patt = 0x8000 ;
    repl_patt = 0x8001 ;
  }
}

void writeWire(int16_t device, int16_t channel, int16_t value){
  // value via I2C to device (e.g. PCF8591)
  Wire.beginTransmission(device);
  Wire.write(channel);
  Wire.write(value);
  Wire.endTransmission();
}

void scale_to_8_bit(int16_t tmp_val){
  // scale int-value (+- 32768) to byte (0 .. 255). (PCF8591)
  tmp_val = (tmp_val) / 128 ;           // scale to 8-Bit
  tmp_val = tmp_val + 128 ;             // scale to 0 .. 255

  byte_high = highByte(tmp_val) ;
  byte_low  = lowByte (tmp_val) ;

  // byte byte_array[2]={0,0}; global !
  byte_array[0] = byte_high ;           // global array >byte_array[]<
  byte_array[1] = byte_low ;            // global array >byte_array[]<
}

void analog_out_via_PCF8591(int16_t tmp_val){
  // output via PCF8591
  scale_to_8_bit(tmp_val) ;             // PCF8591 is 8-Bit
  byte_low  = byte_array[1] ;           // PCF8591 is 8-Bit
  writeWire(PCF8591, 0x40, byte_low) ;  // 0x40 = DA-Converter ; >byte_low< is sent to PCF8591 .
}

void serial_print_values(int16_t tmp_val, int16_t scaled_val, byte byte_high, byte byte_low){
  // testing
  Serial.print  (tmp_val);
  Serial.print  (" . " );

  Serial.print  (scaled_val);
  Serial.print  (" = " );
  Serial.print  (byte_high);
  Serial.print  (" + " );
  Serial.print  (byte_low);
  Serial.println("");
}


// interrupt subroutine
ISR(TIMER1_OVF_vect) {
  TCNT1 =   65336;              // preset timer 1/10000 second

  ++cnt_0_1_ms ;
  if (cnt_0_1_ms >= 10) {
    cnt_0_1_ms = 0 ;
    flag_1_ms  = true ;
    ++cnt_1_ms   ;
    if (cnt_1_ms >= 10) {
      cnt_1_ms = 0 ;
      flag_10_ms  = true ;
      ++cnt_10_ms  ;
      if (cnt_10_ms >= 10) {
        cnt_10_ms = 0 ;
        flag_100_ms  = true ;
        ++cnt_100_ms  ;
        if (cnt_100_ms >= 10) {
          cnt_100_ms = 0 ;
          flag_1000_ms  = true ;
          ++cnt_1000_ms  ;
        }
      }
    }
  }
}


void toggle_led() {
  led_status = digitalRead(ledPin) ^ 1 ;            // LED on -> off and vice versa
  digitalWrite(ledPin, led_status ) ;               // LED ON - LED OFF
}


void send_data() {
  // send (real or synthtic) data to output ()
  if (data_mode == real_data){  // use real data
    // calculate averaged value, reset sum and counter
    avg_val = (int) sum_val / cnt_val ;
  } else {
    // synthetic data from sinetable[]
    if (idx_sinetable < 49) {
      idx_sinetable++ ;
    } else {
      idx_sinetable = 0 ;
    }
    avg_val = (int) sinetable[idx_sinetable] ;
  }

  if (outp_mode == ascii){
    // jAmaSeis: write ASCII strings to serial port:
    buf[0] = (char)0;           // clear buffer
    sprintf(buf,"%d", avg_val); // jAmaSeis
    Serial.println(buf);
  }
  else if (outp_mode == S16) {  // For use with >Spectrum Lab<
    // avoid sync pattern:
    if (avg_val == sync_patt) {
       avg_val = repl_patt ;
       }

    // write binary data to serial port (http://www.qsl.net/dl4yhf/spectra1.html).
    noInterrupts();             // disable interrupts temporarily

    Serial.write(lowByte(avg_val));
    Serial.write(highByte(avg_val));

    Serial.write(0x00);         // sync pattern for 16-bit signed integers
    Serial.write(0x80);         // sync pattern for 16-bit signed integers

    Serial.flush();
    interrupts();               // enable interrupts

    analog_out_via_PCF8591(avg_val) ;
  }
// reset sum
sum_val = 0 ;
// reset counter of converted values from arduino ADC:
cnt_val = 0 ;                 // unused: use real data
}


void loop()
{
  // this loop checks if the interrupt routine has set some flags.
  // eventually the function >send_data()< is called to realize data output.
  // f = conversion rate.


  // delay (2); // With delay == 2 in the average about 10 -12 values are sampled every 1/40 sec


  if (flag_1_ms) {              // f  == 1000 Hz; adjust Options: Audio I/O: Nominal Sample Rate: 1000
    flag_1_ms     = false ;
    flag_2_ms     = flag_2_ms ^ 1 ;
    if (flag_2_ms) {              // f  == 500 Hz; adjust Options: Audio I/O: Nominal Sample Rate: 500
    }
    // uncomment this to send data with f = 1000 Hz
    //send_data() ;             // send data to SpectrumLab and to PCF8591
    //toggle_led() ;            // toggle LED

    if (flag_10_ms) {           // f  ==  100 Hz; adjust Options: Audio I/O: Nominal Sample Rate:  100
      flag_10_ms     = false ;

      /*
      act_val = analogRead(ch_1) ;    // 10 bit; act_val == signed int;  int16_t stores a 16-bit (2-byte) value. This yields a range of -32,768 to 32,767
      */
      act_val = ads.readADC_Differential_0_1();
      sum_val = sum_val + act_val ;   // sum of vals
      cnt_val = cnt_val + 1 ;         // count number of vals

      // uncomment this to send data with f =  100 Hz
      // send_data() ;             // send data to SpectrumLab and to PCF8591
      // toggle_led() ;                  // toggle LED

      cnt_20_ms++ ;             // f  ==   50 Hz; adjust Options: Audio I/O: Nominal Sample Rate:   50
      if (cnt_20_ms == 2) {
        cnt_20_ms  = 0 ;

        // uncomment this to send data with f =   50 Hz
        send_data() ;             // send data to SpectrumLab and to PCF8591
        toggle_led() ;            // toggle LED
      }

      cnt_40_ms++ ;             // f  ==   25 Hz; adjust Options: Audio I/O: Nominal Sample Rate:   25
      if (cnt_40_ms == 4) {
        cnt_40_ms  = 0 ;

        // uncomment this to send data with f =   25 Hz
        //send_data() ;             // send data to SpectrumLab and to PCF8591
        //toggle_led() ;            // toggle LED
      }

      cnt_50_ms++ ;
      if (cnt_50_ms == 5) {     // f  ==   20 Hz; adjust Options: Audio I/O: Nominal Sample Rate:   20
        cnt_50_ms  = 0 ;
      }
      if (flag_100_ms) {        // f  ==   10 Hz; adjust Options: Audio I/O: Nominal Sample Rate:   10
        flag_100_ms     = false ;
        cnt_500_ms++ ;

        if (cnt_500_ms == 5) {  // f  ==    2 Hz; adjust Options: Audio I/O: Nominal Sample Rate:    2
          cnt_500_ms  = 0 ;
        }

        if (flag_1000_ms) {     // f  ==    1 Hz
          flag_1000_ms     = false ;
        }
      }
    }
  }
}
