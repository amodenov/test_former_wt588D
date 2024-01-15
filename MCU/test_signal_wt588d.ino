#include <Arduino.h>
#ifdef WITH_PROGRAMMING_POSSIBILITIES
#   include <SPI.h>
#endif
#include <crc_lib.h>
#ifdef DISPLAY_TM1637
#   include <GyverTM1637.h>
#endif
#include <LiquidCrystal_I2C.h>
/*
 */
 #include <avr/eeprom.h>
/*
 *  The device contains LCD display and three control buttons:
 *     - "Parameter" (select sample or output voltage control)
 *     - "Up"
 *     - "Dn"
 */
/*
 * Samples, preloaded to the WT588D-16P chip
 *
 *  Sample #     Data
 * ------------------------------------------
 *     0     |    40 Hz
 *     1     |    80 Hz
 *     2     |   160 Hz
 *     3     |   315 Hz
 *     4     |     1 KHz
 *     5     |  3.15 KHz
 *     6     |   6.3 KHz
 *     7     |    10 KHz
 *     8     |   40-95 Hz
 *     9     |  100-155 Hz
 *    10     |  pseudo noise 400 Hz (not sure)
 *  --------------------------------------------
 */
/*
 *  The code supports WT588D programming capabilities and 
 *  test signal device code
 *    programming capabilities controlled by  #define WITH_PROGRAMMING_POSSIBILITIES
 *    undefine to disable the programming capabilities
 */
# define DEFAULT_QUEUE_LENGTH 8
struct FixedAverageBuffer {
 public:  
  void FillBuffer(uint32_t valb = 0) {
    for(int i = 0; i < DEFAULT_QUEUE_LENGTH; ++i)
      avgbuffer[i] = valb;
  }
  int Init(void) {
    int rtv = 0;
    avglen = DEFAULT_QUEUE_LENGTH;
    avgsum = 0;
    avgptr = 0;
    avgpoints = 0;
    FillBuffer();
    return rtv;
  }
  uint16_t getValue(void) {
    return (uint16_t)(avgsum/avgpoints);
  }
  void putValue(uint16_t val) {
    if (avglen > avgpoints) {
      avgsum += (uint32_t)val;
      avgbuffer[avgptr] = val;
      ++avgpoints;
      ++avgptr;
    } else {
      if (avglen <= avgptr)
        avgptr = 0;
      avgsum -= (uint32_t)avgbuffer[avgptr];
      avgbuffer[avgptr] = val;
      avgsum += (uint32_t) val;
      ++avgptr;
    }
  }
  
 public:
  byte avgptr;
  uint32_t avgsum;
  byte avgpoints;
  uint16_t avgbuffer[DEFAULT_QUEUE_LENGTH];
  byte avglen;
};
/*
 */
struct DisplayLine {
  uint8_t x;
  uint8_t y;
  const char*   text;
};
/*
 */
/*
  The ICSP connector on MEGA256 is being used
  ICSP connector  (Arduino) WT588D-16P
   ICSP (Arduino) |  WT588D-16P
  ----------------------------------------------------------------------------------
  1. MISO         |  6 (DO)   --- pullup to +3.3 V via 10k to improve signal quality
  2. SCK          |  7 (CLK)
  3. RESET        |  1 (RESET)
  4. GND          |  8 (GND)
  5. MOSI         |  5 (DI)
                  |  11 (K3/CS) to +3.3V for program downloading/reading/verifying
  6. +5 V         |  16 (Vdd)

  Stellaris(Tiva) |   WT588D-16P
     Launchpad
  ------------------------------------------------------------------------------------
  17 PF_0(MISO1)   |  6 (DO)
  40 PF_2(SCK1)    |  7 (CLK)
  30 PF_1(MOSI1)   |  5 (DI)
  29 PE_3(RESET)   |  1 (RESET)
  28 PE_2          |  9 (CS) Flash
  27 PE_1          |  10 (ONE LINE DATA INPUT)
  26 PD_3          |  15 BUSY

    ATmega88PA     |  WT588D-16P
  ------------------------------------------------------------------------------------
  18 PIN_PB4(MISO) |   6 (DO)
  19 PIN_PB5 (SCK) |   7 (CLK)
  17 PIN_PB3(MOSI) |   5 (DI)
  28 PIN_PD6(RESET)|   1 (RESET)
  27 PIN_PD7       |   9 (CS) Flash
  26 PIN_PC3       |   10 (ONE LINE DATA INPUT)
  25 PIN_PC2       |   15 (BUSY)

  WT5688D-16P pin 14 (Vcc) to 3.3 V

   6 PIN_PD4       |   MCP4011 (CS)
  28 PIN_PC5       |   MCP4011 (U/D)
 */
#define ATMEGA88PA 

#ifdef STELLARIS_TIVA
# define MODULE_DATA PE_1
# define MEM_CS PE_2
# define MODULE_RESET PE_3
# define MODULE_BUSY PD_3
# define OPERATION_STATUS PA_7

#endif
/* */
#ifdef ATMEGA88PA
# define MODULE_DATA PIN_PC3
# define MEM_CS PIN_PD7
# define MODULE_RESET PIN_PD6
# define MODULE_BUSY PIN_PC2
# define MODULE_BUSY_LED PIN_PD2
# define OPERATION_STATUS PIN_PB1

# define MCP4011_CS PIN_PB1
# define MCP4011_UD PIN_PB2
# define MONITOR PIN_PC0
# define cs PIN_PB1
# define ud PIN_PB2
// device control buttons
# define PARAM_SELECT PIN_PD5   // PCINT21 
# define VALUE_UP     PIN_PD4   // PCINT20
# define VALUE_DN     PIN_PD3   // PCINT19
# define TDIVIDER 62410 /* 31250 by 3125 to 100 ms */
#endif
/*
 */
#define TOTAL_CHIP_BYTES 4194304
#define TOTAL_CHIP_BLOCKS 16384
#define DATA_BLOCK_SIZE 256
/*
  FLASH IC (25L3206E) commands
 */
#define MEM_WRITE_STATUS_REGISTER 1
#define MEM_PROGRAM_PAGE 2
#define MEM_READ_DATA_BYTES 3
#define MEM_WRITE_DISABLE 4
#define MEM_READ_STATUS_REGISTER 5
#define MEM_WRITE_ENABLE 6
#define MEM_READ_FAST_DATA_BYTES 0x0B
#define MEM_SECTOR_ERASE 0x20
#define MEM_READ_SECURITY_REGISTER 0x2B
#define MEM_CHIP_ERASE 0x60
#define MEM_READ_IDENTIFICATION_ID 0x9F
#define MEM_READ_DEVICE_ID 0x90  // read manufacturer code and device ID
/* WT588D one line commands
 */
#define WT588_STOP_PLAY 0xFE
/* select and start play program - 00 .. 0xDB */
#define WT588_LOOP_PLAY 0xF2
/* set volume as WT588_ZERO_VOLUME+N, where N=0..7
 */
#define WT588_ZERO_VOLUME 0xE0
/*
 */
#define MILLIVOLT_TENTH 10000
#define INTERNAL_1_1V 14  /* Internal 1.1V */
#define INTERNAL_GND  15  /* Ground channel */
/*
 */
#define RECTIFIER_COMPENSATION 3 /* output signal degradation on rectifier */
/* */
#ifdef DISPLAY_TM1637
/*
 * For 7 segment display controlled by 1637 chip
 */
GyverTM1637 disp(PIN_PC4, PIN_PC5);
#endif
/*
 * bit weight and bit offset values will be determined during calibration
 */
static uint16_t bitweight = 45;
static uint16_t bitoffset = 0;
/*
 */
LiquidCrystal_I2C lcd(0x27,16,2);
/*
 */
#define TOTAL_SAMPLES 11
const static char* sample_names[TOTAL_SAMPLES] = {
"40 Hz",
"80 Hz",
"160 Hz",
"315 Hz",
"1 KHz",
"3.15 KHz",
"6.3 KHz",
"10 KHz",
"40-95 Hz",
"100-155Hz",
"pn 400 Hz"
};
/*
 */
DisplayLine msgs[] = {
  {0, 0, (char*)"Sample:"},
  {4, 1, (char*)" V : "},
  {9, 1, NULL}, /* for voltage */
  {8, 0, NULL}, /* for sample ID */
  {8, 0, "        "}, /* to delete sample ID */
  {9, 1, "      "}    /* to delete voltage */
};
/*
 */
void MCP4011_UP(uint8_t);
void MCP4011_DN(uint8_t);
/*
 */
void PrintDisplay(uint8_t nmb) {
  lcd.setCursor(msgs[nmb].x, msgs[nmb].y);
  lcd.print(msgs[nmb].text);
}
/*
 *
 */
enum {
IDLE = 0, SAMPLE_SET, VOLTAGE_SET
};
enum {
   DN_BUTTON = _BV(VALUE_DN), UP_BUTTON = _BV(VALUE_UP), PARAMETER_BUTTON = _BV(PARAM_SELECT)
};
volatile static uint8_t active_button = 0;
volatile static uint8_t active_button_time = 0;
volatile static uint8_t delay_time = 0;

static uint8_t device_state = IDLE;
#define EEPROM_DATA_ENTRY 0
#define MCP4011_HALF_RANGE 32
#define MCP4011_FULL_RANGE 63
static uint8_t current_sample = 0;  /* may be stored in EEPROM */
static uint8_t  voltage_steps = MCP4011_HALF_RANGE;  /* may be stored in EEPROM, initially set to half of range 
                                       because it is set to half of range after power on */
/*
 */
//#    define WITH_PROGRAMMING_POSSIBILITIES
/*
 */
#ifdef WITH_PROGRAMMING_POSSIBILITIES
static uint8_t test_data_block[DATA_BLOCK_SIZE + 2];

bool SendRecord(uint8_t*);
#endif
/*
 */
ISR(TIMER1_OVF_vect) {
  TCNT1 = TDIVIDER;
  if (active_button && (5 > active_button_time)) {
    ++active_button_time;
  }
  if (delay_time) {
    --delay_time;
  }
}
/*
 */
#define BUTTON_PRESSED_MASK (_BV(PARAM_SELECT) | _BV(VALUE_UP) | _BV(VALUE_DN))
ISR(PCINT2_vect) {
  if (PIND & BUTTON_PRESSED_MASK) {
    active_button = 0;
    /* PARAMETER is higher priority button */
    /* no simultaneous pressed buttons allowed */
    // determine source button
    if (0 != (PIND & _BV(PARAM_SELECT))) {
      // process parameter selection button
      active_button |= PARAMETER_BUTTON;
    } else if (0 != (PIND & _BV(VALUE_UP))) {
      // process value up button
      active_button |= UP_BUTTON;
    } else if (0 != (PIND & _BV(VALUE_DN))) {
      // process value dn
      active_button |= DN_BUTTON;
    }
    active_button_time = 0; /* reset button pressed time in 10 ms ticks */
  } else {
    /* button(s) released */
    active_button &= PIND; /* reset activity if released */
    if (0 == active_button) {
      active_button_time = 0;
    }
  }
}
/*
 *
 */
void setup() {
  pinMode(MODULE_RESET, OUTPUT);
  digitalWrite(MODULE_RESET, LOW);
#ifdef WITH_SERIAL_INTERFACE  
  Serial.begin(115200);
  while (!Serial)
    ;  // wait for serial monitor to open
#endif
#ifdef WITH_PROGRAMMING_POSSIBILITIES    
  pinMode(MEM_CS, OUTPUT);
  digitalWrite(MEM_CS, HIGH);  // disable memory chip access
#endif  
  pinMode(MODULE_DATA, OUTPUT);
  digitalWrite(MODULE_DATA, HIGH);
  pinMode(MODULE_BUSY, INPUT);
  pinMode(OPERATION_STATUS, OUTPUT);
  digitalWrite(OPERATION_STATUS, HIGH);
  pinMode(MODULE_BUSY_LED, OUTPUT);
  digitalWrite(MODULE_BUSY_LED, HIGH);
  /*
   */
  #ifdef WITH_PROGRAMMING_POSSIBILITIES
  #ifdef STELLARIS_TIVA 
  SPI.setModule(1);
  #endif

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  #endif
  /* */
  pinMode(MCP4011_CS, OUTPUT);
  digitalWrite(MCP4011_CS, HIGH);
  pinMode(MCP4011_UD, OUTPUT);
  digitalWrite(MCP4011_UD, LOW);
  pinMode(MONITOR, INPUT);
  /*
   */
#ifdef DISPLAY_TM1637
  disp.clear();
  disp.brightness(2);
  disp.clear();
  disp.displayByte(_S | 0b10000000, _A, _7, _8);
#endif
  lcd.init();        
  lcd.init();        
  lcd.backlight();// Включаем подсветку дисплея
  PrintDisplay(0);
  PrintDisplay(1);
  /* test and load (if exists) sample number and steps for MCP4010 
   */
  current_sample = eeprom_read_byte(EEPROM_DATA_ENTRY);
  if (10 >= current_sample) {
    /* looks like there are value in EEPROM */
    voltage_steps = eeprom_read_byte((uint8_t*)(EEPROM_DATA_ENTRY+1));
    lcd.setCursor(0, 1);
    lcd.print(voltage_steps);
    /* set the MCP4011 resistor to previously saved value */
    if (MCP4011_HALF_RANGE <= voltage_steps) {
      MCP4011_UP(voltage_steps-MCP4011_HALF_RANGE);
    } else {
      MCP4011_DN(MCP4011_HALF_RANGE - voltage_steps);
    }
  } else {
    current_sample = 0;
  }
  /*
   */
  msgs[3].text = sample_names[current_sample];
  PrintDisplay(3);
  cli();
  /* Prepare for button interrupts
   */
  PCICR |= 4;       // enable PCINT[16-23] interrupts
  PCMSK2 |= _BV(PCINT19) | _BV(PCINT20) | _BV(PCINT21);   // unmask PCINT19, PCINT20, PCINT21 --- button interrupts
  /*
  */
  TCCR1A = TCCR1C = 0;
  TCCR1B = 4;  /* divide by 256 to 31250 Hz */
  TCNT1 = TDIVIDER; /* divide to 10 ms */
  TIMSK1 |= 1;
  sei();
  /*
   * restore previously saved parameters (if exists)
   */
  
}
#ifdef WITH_PROGRAMMING_POSSIBILITIES

void MemorySetAddress(uint32_t devaddr) {
  union {
    uint32_t val;
    struct {
      uint8_t lsb;
      uint8_t mbb;
      uint8_t msb;
    };
  } addr;
  addr.val = devaddr;
  SPI.transfer(addr.msb);
  SPI.transfer(addr.mbb);
  SPI.transfer(addr.lsb);
}

uint8_t ReadStatusRegister(void) {
  digitalWrite(MEM_CS, LOW);
  SPI.transfer(MEM_READ_STATUS_REGISTER);
  uint8_t status = SPI.transfer(0x00);
  status = SPI.transfer(0x00);
  asm volatile("nop");
  digitalWrite(MEM_CS, HIGH);
  return status;
}

void MemoryReadDataBlock(uint16_t blockaddress, uint8_t* block_buffer) {
  uint32_t devaddr = (uint32_t)blockaddress << 8;
  digitalWrite(MEM_CS, LOW);
  SPI.transfer(MEM_READ_DATA_BYTES);
  MemorySetAddress(devaddr);
  for (int i = 0; i < DATA_BLOCK_SIZE; ++i) {
    block_buffer[i] = SPI.transfer(0x00);
  }
  asm volatile("nop");
  digitalWrite(MEM_CS, HIGH);
}

uint16_t MemoryCheckEmptyChip(void) {
  uint8_t flg = 0;
  uint16_t errblocks = 0;
  uint16_t current_block = 0;
  for (uint32_t blocknumber = 0; blocknumber < TOTAL_CHIP_BLOCKS; ++blocknumber) {
    MemoryReadDataBlock(blocknumber, test_data_block);
    flg = 0;
    for (int i = 0; i < DATA_BLOCK_SIZE; ++i) {
      if (0xFF != test_data_block[i]) {
        flg = 1;
      }
    }
    if (0 == (++current_block % 200)) {
      test_data_block[0] = 'B';
      test_data_block[1] = current_block & 0xFF;
      test_data_block[2] = (current_block >> 8) & 0xFF;
      test_data_block[3] = errblocks & 0xFF;
      test_data_block[4] = (errblocks >> 8) & 0xFF;
      SendRecord(test_data_block);
    }
    if (flg) {
      ++errblocks;
    }
  }
  test_data_block[0] = 'B';
  test_data_block[1] = current_block & 0xFF;
  test_data_block[2] = (current_block >> 8) & 0xFF;
  test_data_block[3] = errblocks & 0xFF;
  test_data_block[4] = (errblocks >> 8) & 0xFF;
  SendRecord(test_data_block);
  return errblocks;
}

void MemorySetWriteEnable(uint8_t action) {
  digitalWrite(MEM_CS, LOW);
  if (action) {
    SPI.transfer(MEM_WRITE_ENABLE);
  } else {
    SPI.transfer(MEM_WRITE_DISABLE);
  }
  asm volatile("nop");
  digitalWrite(MEM_CS, HIGH);
}

void MemoryChipErase(void) {
  MemorySetWriteEnable(1);
  digitalWrite(MEM_CS, LOW);
  SPI.transfer(MEM_CHIP_ERASE);
  asm volatile("nop");
  digitalWrite(MEM_CS, HIGH);
  while (3 & ReadStatusRegister()) {
    delay(10);
  }
  MemorySetWriteEnable(0);
}

void MemoryChipEraseAndReportTheProgress(void) {
  MemorySetWriteEnable(1);
  digitalWrite(MEM_CS, LOW);
  SPI.transfer(MEM_CHIP_ERASE);
  asm volatile("nop");
  digitalWrite(MEM_CS, HIGH);
  while (3 & ReadStatusRegister()) {
    delay(100);
    test_data_block[0] = 'B';
    test_data_block[1] = 0;
    SendRecord(test_data_block);
  }
  test_data_block[0] = 'B';
  test_data_block[1] = 0;
  SendRecord(test_data_block);
  MemorySetWriteEnable(0);
}

bool MemoryProgramPage(uint16_t blockaddress, const uint8_t* datablock) {
  bool is_problem = false;
  uint32_t devaddr = (uint32_t)blockaddress << 8;
  MemorySetWriteEnable(1);
  digitalWrite(MEM_CS, LOW);
  SPI.transfer(MEM_PROGRAM_PAGE);
  MemorySetAddress(devaddr);
  for (int i = 0; i < DATA_BLOCK_SIZE; ++i) {
    SPI.transfer(datablock[i]);
  }
  asm volatile("nop");
  digitalWrite(MEM_CS, HIGH);
  /* wait til end of operation
   */
  while (3 & ReadStatusRegister())
    delayMicroseconds(5);
  MemorySetWriteEnable(0);
  /* read current block and compare with pattern
   */
  MemoryReadDataBlock(blockaddress, test_data_block);
  for (int i = 0; i < DATA_BLOCK_SIZE; ++i) {
    if (datablock[i] != test_data_block[i])
      is_problem = true;
  }
  return is_problem;
}

#endif // ifdef   WITH_PROGRAMMING_POSSIBILITIES
/*
 *  Send single line command
 */
#define SHORT_STEP 200
#define LONG_STEP  400
void SendSingleLineBit(uint8_t b) {
  digitalWrite(MODULE_DATA, HIGH);
  if (b) { /* represent 1 */
    delayMicroseconds(LONG_STEP);
    digitalWrite(MODULE_DATA, LOW);
    delayMicroseconds(SHORT_STEP);
  } else { /* represent 0 */
    delayMicroseconds(SHORT_STEP);
    digitalWrite(MODULE_DATA, LOW);
    delayMicroseconds(LONG_STEP);
  }
  digitalWrite(MODULE_DATA, HIGH);
}

void SendSingleLineCommand(uint8_t cmd, bool withreset=false) {
  if (withreset) {
    digitalWrite(MODULE_RESET, LOW);
    delay(5);
    digitalWrite(MODULE_RESET, HIGH);
    /* WAIT 17 ms*/
    delay(20);
  }
  /* DATA - 5ms prefix */
  digitalWrite(MODULE_DATA, LOW);
  delay(5);
  for(uint8_t i = 0; i < 8; ++i) {
    digitalWrite(MODULE_DATA, HIGH);
    if (cmd & 1) {
      delayMicroseconds(LONG_STEP);
      digitalWrite(MODULE_DATA, LOW);
      delayMicroseconds(SHORT_STEP);    
    } else {
      delayMicroseconds(SHORT_STEP);
      digitalWrite(MODULE_DATA, LOW);
      delayMicroseconds(LONG_STEP);
    }
    cmd >>= 1;
  }
  digitalWrite(MODULE_DATA, HIGH);
  delay(2);
}
/*
 */
#ifdef WITH_PROGRAMMING_POSSIBILITIES
#define MAX_TIMEOUT 2000
#define START_OF_RECORD 'S'
#define READY_TO_GET_RECORD 'R'
#define QUIT_RECORD 'Q'
#define GIVE_ME_A_RECORD 'G'
#define CHUNKSIZE 32
unsigned int nbytes = 0;
bool ReceiveRecord(uint8_t* buffer) {
  uint8_t prefix = 0;
  uint8_t attempts = 5;
  uint16_t crc_pattern = 0;
  Serial.flush();
  while (0 < attempts) {
    do {
      prefix = Serial.read();
    } while (START_OF_RECORD != prefix);
    // ready to receive
    Serial.write(READY_TO_GET_RECORD);
    // wait for data block
    nbytes = 0;
    unsigned long t0 = millis();
    while (DATA_BLOCK_SIZE > nbytes) {
      if (CHUNKSIZE <= Serial.available()) {
        Serial.readBytes((char*)&buffer[nbytes], CHUNKSIZE);
        nbytes += CHUNKSIZE;
      }
      if (MAX_TIMEOUT < (millis() - t0)) {
        --attempts;
        break;
      }
    }
    if (DATA_BLOCK_SIZE <= nbytes) {
      Serial.readBytes((char*)&crc_pattern, 2);
      uint16_t vcrc = get_crc16((const uint8_t*)buffer, DATA_BLOCK_SIZE);
      if (crc_pattern != vcrc) {
        Serial.print("NAK\n");
        --attempts;
        break;
      } else {
        Serial.print("ACK\n");
        return true;
      }
    }
  }
  return false;
}
/*
 */
#define ANSWER_LENGTH 3
bool SendRecord(uint8_t* buffer) {
  uint8_t prefix = 0;
  uint8_t attempts = 5;
  while (0 < attempts) {
    do {
      prefix = Serial.read();
    } while (GIVE_ME_A_RECORD != prefix);
    memcpy(test_data_block, buffer, DATA_BLOCK_SIZE);
    uint16_t crcvalue = get_crc16(test_data_block, DATA_BLOCK_SIZE);
    test_data_block[DATA_BLOCK_SIZE] = crcvalue & 0xFF;
    test_data_block[DATA_BLOCK_SIZE + 1] = (crcvalue >> 8) & 0xFF;
    Serial.write(test_data_block, DATA_BLOCK_SIZE + 2);
    unsigned long t0 = millis();
    // waiting for an answer
    while (ANSWER_LENGTH > Serial.available()) {
      if (MAX_TIMEOUT < (millis() - t0)) {
        --attempts;
        break;
      }
    }
    if (ANSWER_LENGTH <= Serial.available()) {
      char ans[5];
      Serial.readBytes(ans, ANSWER_LENGTH);
      ans[ANSWER_LENGTH] = '\0';
      if (0 != strcmp(ans, "ACK")) {
        --attempts;
      } else {
        return true;
      }
    }
  }
  return false;
}
#endif
/*
 */
void MCP4011_DN(uint8_t steps) {
  digitalWrite(MCP4011_UD, HIGH);
  delayMicroseconds(1);
  digitalWrite(MCP4011_CS, LOW);
  delayMicroseconds(1);
  for(uint8_t i = 0;i < steps; ++i) {
    digitalWrite(MCP4011_UD, LOW);
    delayMicroseconds(2);
    digitalWrite(MCP4011_UD, HIGH); // value should be incremented
    delayMicroseconds(1);
  }
  digitalWrite(MCP4011_CS, HIGH);
  delayMicroseconds(1);
  digitalWrite(MCP4011_UD, LOW);
}

void MCP4011_UP(uint8_t steps) {
  digitalWrite(MCP4011_UD, LOW);
  delayMicroseconds(1);
  digitalWrite(MCP4011_CS, LOW);
  delayMicroseconds(1);
  for(uint8_t i = 0; i < steps; ++i) {
    digitalWrite(MCP4011_UD, HIGH);
    delayMicroseconds(1);
    digitalWrite(MCP4011_UD, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(MCP4011_CS, HIGH);
}
/*
 */
uint16_t GetChannel(uint8_t chn) {
  uint16_t wADC;
  ADMUX = chn | _BV(REFS0);
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  return wADC;
}
/*
 */
# define TIMESTEP 300
# define POINTS_TO_BUFFER DEFAULT_QUEUE_LENGTH
# define INTERNAL_REFERENCE_1_1 11000
FixedAverageBuffer test_output;
static uint16_t adcvalue = 0;
/*
 * Determine bit weight value based on channel 15 - ground and channel 14 - internal reference 1.1 V
 */
void DetermineBitWeight(void) {
  test_output.Init();
  /* 0V value */
  for(uint8_t i = 0; i < DEFAULT_QUEUE_LENGTH; ++i) {
    test_output.putValue(GetChannel(INTERNAL_GND));
  }
  adcvalue = test_output.getValue();
  test_output.Init();
  /* 1.1 V value */
  for(uint8_t i = 0; i < DEFAULT_QUEUE_LENGTH; ++i) {
    test_output.putValue(GetChannel(INTERNAL_1_1V));
  }
  bitweight = INTERNAL_REFERENCE_1_1/(test_output.getValue() - adcvalue);
  bitoffset = adcvalue;
  Serial.print("\r\nADC koeffs : "); Serial.print(bitweight); Serial.print("   "); Serial.println(bitoffset);
  test_output.Init();
}
/*
 */
static bool isFirstEntry = true;

#ifdef WITH_PROGRAMMING_POSSIBILITIES
static uint8_t datablock[DATA_BLOCK_SIZE];
static uint16_t non_empty_blocks = 0;
static uint16_t total_blocks_to_read = 0;
static uint16_t blocks_to_write = 0;
static uint16_t write_from_block = 0;
#endif
static unsigned long time_interval_left = 0;
static uint8_t points_counter = POINTS_TO_BUFFER;
static uint8_t display_value = 1;
void loop() {  
  if (isFirstEntry) {
    DetermineBitWeight(); /* prepare ADC value koefficients */
    test_output.Init();
    isFirstEntry = false;
    delay(400);
    //
#ifdef WITH_PROGRAMMING_POSSIBILITIES    
    SPI.end();
#endif
    pinMode(MEM_CS, INPUT);
    /* */
    pinMode(PIN_PB5, INPUT);
    pinMode(PIN_PB3, INPUT);
    /* */
    delay(50);

    SendSingleLineCommand(8, true);
    SendSingleLineCommand(0xE7);
    SendSingleLineCommand(0xF2);
    //
    time_interval_left = millis();
  }
#ifdef WITH_PROGRAMMING_POSSIBILITIES  
  // get current command and check command flag (second byte should be 0)
  non_empty_blocks = 0;
  total_blocks_to_read = 0;
  blocks_to_write = 0;
  write_from_block = 0;
  if ((1 <= Serial.available()) && ('S' == Serial.peek())) {
    if (ReceiveRecord(datablock) & (0 == datablock[1])) {
      // check what we should do
      switch (datablock[0]) {
        case 'C':  // check if chip is empty
          digitalWrite(MODULE_RESET, LOW);
          // send operation status as '0' - non-empty chip, '1' - empty chip
          non_empty_blocks = MemoryCheckEmptyChip();
          datablock[0] = ((0 < non_empty_blocks) ? '0' : '1');
          datablock[1] = 'O';  // operation status sign
          memcpy(&datablock[2], &non_empty_blocks, 2);
          SendRecord(datablock);
          break;
        case 'R':  // read whole chip contents
          digitalWrite(MODULE_RESET, LOW);
          //
          total_blocks_to_read = (datablock[2] & 0xFF) + ((datablock[3] << 8) & 0xFF00);
          for (uint16_t i = 0; i < total_blocks_to_read; ++i) {
            MemoryReadDataBlock(i, datablock);
            SendRecord(datablock);
          }
          break;
        case 'E':  // erase whole chip
          digitalWrite(MODULE_RESET, LOW);
          // send operation status as '0' - erase complete OK, '1' - problem during erase
          MemoryChipEraseAndReportTheProgress();
          datablock[0] = 'F';  // operation complete (finished)
          datablock[1] = 'O';  // operation status sign
          memcpy(&datablock[2], &non_empty_blocks, 2);
          SendRecord(datablock);
          break;
        case 'W':  // write arbiterary number of block to chip
          digitalWrite(MODULE_RESET, LOW);
          // get number of blocks to write and starting block number
          write_from_block = (datablock[2] & 0xFF) + ((datablock[3] << 8) & 0xFF00);
          blocks_to_write = (datablock[4] & 0xFF) + ((datablock[5] << 8) & 0xFF00);
          for (uint16_t i = 0; i < blocks_to_write; ++i) {
            if (ReceiveRecord(datablock)) {
              uint8_t sts = (uint8_t)MemoryProgramPage(i + write_from_block, datablock);
              datablock[0] = sts + 0x30;
              SendRecord(datablock);
            }
          }
          break;
        case 'S':  // start selected program by number
          // datablock[2] - program number to play
          SPI.end();
          pinMode(MEM_CS, INPUT);
          /* */
          pinMode(PIN_PB5, INPUT);
          pinMode(PIN_PB3, INPUT);
          /* */
          delay(50);
          SendSingleLineCommand(datablock[2], true);
          break;
        case 'F':
          SendSingleLineCommand(WT588_STOP_PLAY);
          SPI.begin();
          pinMode(MEM_CS, OUTPUT);
          digitalWrite(MEM_CS, HIGH);
          break;
        case 'V' : // set volume level
          SendSingleLineCommand(WT588_ZERO_VOLUME + datablock[2]);
        break;
        case 'L' : // set loop play
          SendSingleLineCommand(WT588_LOOP_PLAY);
        break;
      }
    }
  } else {
    Serial.flush();
  }
#endif // ifdef  WITH_PROGRAMMING_POSSIBILITIES
#ifdef WITH_SERIAL_INTERFACE
  /*
   * The same actions as for device buttons
   */
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print(cmd);
    switch(toupper(cmd)) {
      case 'U' :
        Serial.println("MCP UP");
        MCP4011_UP(1);
        break;
      case 'D' :
        Serial.println("MCP down");
        MCP4011_DN(1);
        break;
      case 'P' :
        break;
    }
  }
#endif  
  /* read voltage and convert to volts
   */
  if (TIMESTEP <= (abs(millis() - time_interval_left))) {
    time_interval_left = millis();
    adcvalue = analogRead(MONITOR);
    test_output.putValue(adcvalue);
    if (0 == points_counter) {
      points_counter = POINTS_TO_BUFFER;
      adcvalue = test_output.getValue();
      adcvalue = adcvalue*bitweight + bitoffset;
      adcvalue *= RECTIFIER_COMPENSATION; // ? should be investigated
      if (VOLTAGE_SET != device_state) {
        lcd.setCursor(9, 1);
        lcd.print("       ");
        lcd.setCursor(9, 1);
        lcd.print(adcvalue/10); /* remove tenth of millivolts */
        lcd.print("mV");
      }
    }
    --points_counter;
  }  
  /*
   *
   */
  if ((0 != active_button) && (0 == active_button_time)) {
    if (PARAMETER_BUTTON & active_button) {
      /* to next device state */
      if (SAMPLE_SET == device_state) {
        /* state chnaged from sample set means sample already selected */
        SendSingleLineCommand(current_sample, true);
        SendSingleLineCommand(0xE7);
        SendSingleLineCommand(0xF2);
        /* save to eeprom */
        eeprom_write_byte(EEPROM_DATA_ENTRY, current_sample);
        eeprom_busy_wait();
        if (eeprom_read_byte(EEPROM_DATA_ENTRY) != current_sample) {
          lcd.setCursor(0, 1);
          lcd.print("   ");
          lcd.setCursor(0, 1);
          lcd.print("ERR0");
        }
      }
      /* switch form voltage set */
      if (VOLTAGE_SET == device_state) {
        /* save to eeprom */
        eeprom_write_byte((uint8_t*)(EEPROM_DATA_ENTRY+1), voltage_steps);
        eeprom_busy_wait();
        if (eeprom_read_byte((uint8_t*)(EEPROM_DATA_ENTRY+1)) != voltage_steps) {
          lcd.setCursor(0, 1);
          lcd.print("   ");
          lcd.setCursor(0, 1);
          lcd.print("ERR1");
        }
      }
      device_state = (device_state + 1) % 3;
      if (SAMPLE_SET != device_state) {
        msgs[3].text = sample_names[current_sample];
        PrintDisplay(3);
      }
    }
    if (UP_BUTTON & active_button) {
      if (SAMPLE_SET == device_state) {
        if ((current_sample+1) < TOTAL_SAMPLES) {
          ++current_sample;
          PrintDisplay(4);
          msgs[3].text = sample_names[current_sample];
          PrintDisplay(3);
        }
      } else if (VOLTAGE_SET == device_state) {
        MCP4011_UP(1);
        if (MCP4011_FULL_RANGE > voltage_steps)
          ++voltage_steps;
      }
    }
    if (DN_BUTTON & active_button) {
      if (SAMPLE_SET == device_state) {
        if ((current_sample-1) >= 0) {
          --current_sample;
          PrintDisplay(4);
          msgs[3].text = sample_names[current_sample];
          PrintDisplay(3);
        }
      } else if (VOLTAGE_SET == device_state) {
        MCP4011_DN(1);
        if (0 < voltage_steps)
          --voltage_steps;
      }
    }
    ++active_button_time;
  }
  if ((0 != active_button) && (5 == active_button_time)) {
    /* enter autorepeat mode */
    
  }
  /*
   * update display according to the mode
   */
  if (IDLE != device_state) {
    if (0 == delay_time) {
      if (SAMPLE_SET == device_state) {
        if (display_value) {
          msgs[3].text = sample_names[current_sample];
          PrintDisplay(3);
        } else {
          PrintDisplay(4);
        }
      } else if (VOLTAGE_SET == device_state) {
        if (display_value) {
          adcvalue = test_output.getValue();
          adcvalue = adcvalue*bitweight + bitoffset;
          adcvalue *= RECTIFIER_COMPENSATION; // ? should be investigated
          lcd.setCursor(9, 1);
          lcd.print(adcvalue/10); /* remove tenth of millivolts */
          lcd.print("mV");
        } else {
          lcd.setCursor(9, 1);
          lcd.print("       ");
        }
      }
      display_value ^= 1;
      delay_time = 5;
    }
  }
/*
*/
digitalWrite(MODULE_BUSY_LED, digitalRead(MODULE_BUSY));
/*
 */  
}
