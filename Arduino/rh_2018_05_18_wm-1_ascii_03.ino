/*!
Arduino sketch for AD-conversion. Uses interrupts.
   Input:
      analog values at channel 1
   Output:
      serial: converted values
      // LED   : blinking with f = 1 Hz
      LED   : blinking with f = 1/2 sampling frequency == 20 Hz measured with DDS


In the main loop the signal of ch_1 is continously converted and summed up.
Every 1/40 second the interrupt subroutine >ISR(TIMER1_OVF_vect)< is called:
   It toggles the LED with f = 1 Hz
   It averages the measured values;
   It sends the averaged value via serial:
      Format of sent value is configurable:
          ASCII  or
          binary: signed or signed, big endian or little endian
            (may be used to communicate with Spectrum Lab)


URL == https://www.heise.de/developer/artikel/Timer-Counter-und-Interrupts-3273309.html

rh:
2018_05_04:
    a) toggle LED on/off;  f = 1 Hz
    b) send square signal; f = 1 Hz
2018_05_05:
    a) Sampling Rate == 40 Hz
    b) toggle LED on/off;  f =  1 Hz
    c) send square signal; f =  1 Hz
    d) send square signal; f = 20 Hz

timer preset according to URL:
initcount = 65536 - (16000000/20)/256 = 62411  // == 1/40 sec
*/

// ==================================================================================================

bool debug     = false ;
// bool debug     = true ;
bool debug_hex = false ;

#define ledPin 13

// outp_mode == data format
#define un_sgnd  1
#define sgnd     1

#define ascii  1
#define binary 2
#define IQ12   3

#define U16    4  // unsigned int * little endian * 0..65535 * 0xFFFF
#define U16_BE 5  // unsigned int * big endian    * 0..65535 * 0xFFFF

#define S16    6  // signed int   * little endian * +/-32767 * 0x8000
#define S16_BE 7  // signed int   * big endian    * +/-32767 * 0x8000

// ############# output mode ############
// int outp_mode = S16 ;
int outp_mode = ascii ;    // output mode

#define little_endian 1
#define big_endian 2

int endian    ;
int led_status;

int sync_patt ;  // sync    pattern
int repl_patt ;  // replace pattern

const int ch_1 = 0;  // ch 1
const int ch_2 = 1;  // ch 2
char      buf[120];   // char buffer

const int line_01_hz = 405;    // base value of  1 Hz line = square line
const int line_20_hz = 425;    // base value of 20 Hz line = square line
int       base_01_delta;       // height of  1 Hz square signal
int       base_20_delta;       // height of 20 Hz square signal

int  cnt_dt ;        // count number of delta t (1..20)

int  val;            // actual ADC value == signed int
int  cnt_val;        // counts number of values since last interrupt
int  sum_val;        // sum           of values since last interrupt
int  avg_val;        // average value == sum_val / cnt_val
unsigned int u_avg_val;  // dito

byte msg[2];         // char buffer (if binary output)
byte iq12_0;         // char
byte iq12_1;         // char
byte iq12_2;         // char
byte iq12_3;         // char

byte byte_high ;     // high_byte of binary of val
byte byte_low  ;     // low_byte of binary of val
byte u_byte_high ;   // high_byte of binary of val (unsigned int)
byte u_byte_low  ;   // low_byte  of binary of val (unsigned int)

byte sync_high ;     // high_byte of sync pattern
byte sync_low  ;     // low_byte  of sync pattern

int  cnt_dbg = 0 ;   // when in debug
int  cnt_cvs = 0 ;   // cnt conversions
int  cnt_cvs_max = 100 ;   // cnt conversions max


void setup()
{
  //Serial.begin(115200);
  Serial.begin(9600);

  base_01_delta = 5 ;
  base_20_delta = 5 ;

  cnt_dt        = 0 ;
  cnt_val       = 0 ;
  sum_val       = 0 ;
  avg_val       = 0 ;
  cnt_cvs       = 0 ;   // cnt conversions

  pinMode(ledPin, OUTPUT);  // LED Pin

  // Timer 1
  noInterrupts();           // diasable interrupts temporarily
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 65535;            // preset timer
  TCCR1B |= (1 << CS12);    // 256 as prescale value
  TIMSK1 |= (1 << TOIE1);   // enable Timer Overflow Interrupt
  interrupts();             // enable interrupts

  if (outp_mode == ascii) {
    ;
  } else if (outp_mode == binary) {
    sync_patt = -32768 ;
    sync_high = highByte(sync_patt);
    sync_low  = lowByte (sync_patt);
    ;
  } else if (outp_mode == IQ12) {
    ;
  } else if (outp_mode == U16) {
    endian = little_endian;
    sync_patt = 0xFFFF ;
    repl_patt = 0xFFFE ;
  } else if (outp_mode == U16_BE) {
    endian = big_endian;
    sync_patt = 0xFFFF ;
    repl_patt = 0xFFFE ;
  } else if (outp_mode == S16) {
    endian = little_endian;
    sync_patt = 0x8000 ;
    repl_patt = 0x8001 ;
  } else if (outp_mode == S16_BE) {
    endian = big_endian;
    sync_patt = 0x8000 ;
    repl_patt = 0x8001 ;
  }

  // binary outp_mode
}

void toggle_led() {
  led_status = digitalRead(ledPin) ^ 1 ;            // LED on -> off and vice versa
  digitalWrite(ledPin, led_status ) ;               // LED ON - LED OFF
}

void serial_write_Big_Enddian(byte byte_high, byte byte_low){
  Serial.write((byte)byte_high); // write high order byte
  Serial.write((byte)byte_low);  // write low order byte
}

void serial_write_Little_Enddian(byte byte_high, byte byte_low){
  Serial.write((byte)byte_low);  // write low order byte
  Serial.write((byte)byte_high); // write high order byte
}

void serial_write_Enddian(byte byte_high, byte byte_low){
  if (endian == little_endian){
    serial_write_Little_Enddian(byte_high, byte_low) ;
  } else {
    serial_write_Big_Enddian(byte_high, byte_low) ;
    }
}

// interrupt subroutine
ISR(TIMER1_OVF_vect) {
  // TCNT1 = 34286;    // preset timer
  // TCNT1 = 62411;    // preset timer
  TCNT1 = 63973;       // preset timer 1/40 sec

  cnt_dt = cnt_dt + 1 ;
  // 1 Hz square signal: 0.5 sec down, 0.5 sec up
  if(cnt_dt == 20){
    // toggle_led() ;
    base_01_delta = base_01_delta * -1 ;              // square wave
    cnt_dt = 0 ;
    }

  cnt_cvs = cnt_cvs + 1 ;
  // 1 Hz Signal-Linie
  if(cnt_cvs == cnt_cvs_max){
    cnt_cvs = 0 ;
    }

  // calculate averaged value, reset sum and counter
  if (!debug) {
    avg_val = sum_val / cnt_val ;
  } else {
    if (cnt_dbg < 200) {
      avg_val = 506 + cnt_dbg ;
      cnt_dbg = cnt_dbg + 4 ;
    } else {
      cnt_dbg = 0 ;
      avg_val = 506 + cnt_dbg ;
      }
    }

  // reset sum
  sum_val = 0 ;

  // 20 Hz signal-line
  base_20_delta = base_20_delta * -1 ;  //

  if (outp_mode == ascii){
    // write asccii strings to serial port:
    buf[0] = (char)0;  // clear buffer
    sprintf(buf,"%d", avg_val);  // jAmaSeis
    Serial.println(buf);
  } else if (outp_mode == U16 || outp_mode == U16_BE || outp_mode == S16 || outp_mode == S16_BE) {
    // For use with >Spectrum Lab< write binary data to serial port (http://www.qsl.net/dl4yhf/spectra1.html).
    // Parameters are: >9600,8-N-1,U16<, i.e. Baud rate == 9600, 8N1; Data format == 16-bit signed integer, little endian.
    //
    if (avg_val == sync_patt) {
       avg_val = repl_patt ;
       }
    byte_high = highByte(avg_val);
    byte_low  = lowByte(avg_val);

    serial_write_Enddian(byte_high, byte_low) ;

    if(cnt_cvs == 1){
      byte_high = highByte(sync_patt);
      byte_low  = lowByte (sync_patt);
      serial_write_Enddian(byte_high, byte_low) ;
      }
  } else if (outp_mode == IQ12){
    byte_high = highByte(avg_val);       // avg_val is a signed int
    byte_low  = lowByte(avg_val);

    // for numbers <= 32767:  unsigned int == signed int, even low byte and high byte!
    u_avg_val = avg_val;                 // avg_val is a signed int; u_avg_val is an unsigned int
    u_byte_high = highByte(u_avg_val);   // >u_byte_high< should be identical to >byte_high<
    u_byte_low  = lowByte(u_avg_val);    // >u_byte_low<  should be identical to >byte_low<

    iq12_0 = 0xFF ;                          // 'stop-byte' == header byte: file:///D:/Spectrum/html/settings.htm#audio_via_COM_frame_sync
    iq12_1 = u_byte_low ;                    // low byte of channel_1
    iq12_2 = u_byte_low ;                    // test: channel_2 := channel_1
    iq12_2 = lowByte(led_status);         // test: channel_2 = led_status (to measure conversion rate: results in 40 Hz
    iq12_3 = byte_high + (byte_high << 4) ;  // significant parts of ch_1 and ch_2


    if (iq12_2 == 0xFF){                 // prevent confusion with 'stop byte'
      iq12_2 == 0xFE ;
      }
    if (iq12_1 == 0xFF){                 // prevent confusion with 'stop byte'
      iq12_1 == 0xFE ;
      }


    if (!debug_hex) {                    // see binary data with >Device Monitoring Studio<: https://www.hhdsoftware.com/device-monitoring-studio
      Serial.write(iq12_0) ;             // write binary data to COMx: 'stop byte'
      Serial.write(iq12_1) ;             // values
      Serial.write(iq12_2) ;             //         ...
      Serial.write(iq12_3) ;
      }
    else {                             // write values as Hex readable in ASCII
      buf[0] = (char)0;  // clear buffer
      sprintf(buf,"- %d %u %02X:%02X %02X:%02X", cnt_val,u_avg_val, iq12_0, iq12_1, iq12_2, iq12_3);
      Serial.println(buf);
    }
  }

  toggle_led() ;
  cnt_val = 0 ;
}


void loop()
{
  // https://www.arduino.cc/reference/en/language/functions/analog-io/analogread/
  val = analogRead(ch_1) ;        // 10 bit; val == signed int;  int stores a 16-bit (2-byte) value. This yields a range of -32,768 to 32,767
//  val = 512 ;                     // 10 bit; val ==  signed int
  sum_val = sum_val + val ;       // sum of vals
  cnt_val = cnt_val + 1 ;         // count number of vals

  // With delay == 2 in the average about 10 -12 values are sampled every 1/40 sec
  delay (2);
}
