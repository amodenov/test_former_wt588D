#include <Arduino.h>
#include <SPI.h>
#include <crc_lib.h>
/*
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

   6 PIN_PD4       |   MCP41011 (CS)
  28 PIN_PC5       |   MCP41011 (U/D)
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

# define MCP41011_CS PIN_PD4
# define MCP41011_UD PIN_PC5
# define MCP41011_OUT PIN_PC4
# define cs PIN_PD4
# define ud PIN_PC5
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
/*
 */
#define BITWEIGHT 31 /* ADC bit weight in millivolts */
#define MILLIVOLT_TENTH 10000
/*
 */
static uint8_t test_data_block[DATA_BLOCK_SIZE + 2];

bool SendRecord(uint8_t*);
/*
 */
void setup() {
  pinMode(MODULE_RESET, OUTPUT);
  digitalWrite(MODULE_RESET, LOW);
  Serial.begin(115200);
  
  while (!Serial)
    ;  // wait for serial monitor to open
  pinMode(MEM_CS, OUTPUT);
  digitalWrite(MEM_CS, HIGH);  // disable memory chip access
  pinMode(MODULE_DATA, OUTPUT);
  digitalWrite(MODULE_DATA, HIGH);
  pinMode(MODULE_BUSY, INPUT);
  pinMode(OPERATION_STATUS, OUTPUT);
  digitalWrite(OPERATION_STATUS, HIGH);
  pinMode(MODULE_BUSY_LED, OUTPUT);
  digitalWrite(MODULE_BUSY_LED, HIGH);
  /*
   */
  #ifdef STELLARIS_TIVA 
  SPI.setModule(1);
  #endif

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  /* */
  pinMode(MCP41011_CS, OUTPUT);
  digitalWrite(MCP41011_CS, HIGH);
  pinMode(MCP41011_UD, OUTPUT);
  digitalWrite(MCP41011_UD, LOW);
  pinMode(MCP41011_OUT, INPUT);
}
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

void WriteStatusRegister(uint8_t newstate) {
  digitalWrite(MEM_CS, LOW);
  SPI.transfer(MEM_WRITE_STATUS_REGISTER);
  SPI.transfer(newstate);
  asm volatile("nop");
  digitalWrite(MEM_CS, HIGH);
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

void MemorySectorErase(uint16_t sectornumber) {
  uint32_t devaddr = sectornumber << 12;
  MemorySetWriteEnable(1);
  digitalWrite(MEM_CS, LOW);
  SPI.transfer(MEM_SECTOR_ERASE);
  MemorySetAddress(devaddr);
  asm volatile("nop");
  digitalWrite(MEM_CS, HIGH);
  while (3 & ReadStatusRegister()) {
    delay(10);
  }
  MemorySetWriteEnable(0);
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
void SendSingleLineCommand_0(uint8_t cmd) {
  digitalWrite(MODULE_RESET, LOW);
  delay(5);
  digitalWrite(MODULE_RESET, HIGH);
  /* WAIT 17 ms*/
  delay(17);
  /* DATA - 5ms prefix */
  digitalWrite(MODULE_DATA, LOW);
  delay(5);
  digitalWrite(MODULE_DATA, HIGH);
  for (int i = 0; i < 8; ++i) {
    SendSingleLineBit(cmd & 1);
    cmd >>= 1;
  }
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
/*
 */
// void MCP41011_DN(uint8_t steps) {
//   digitalWrite(MCP41011_UD, HIGH);
//   delayMicroseconds(1);
//   digitalWrite(MCP41011_CS, LOW);
//   delayMicroseconds(1);
//   for(uint8_t i = 0;i < steps; ++i) {
//     digitalWrite(MCP41011_UD, LOW);
//     delayMicroseconds(2);
//     digitalWrite(MCP41011_UD, HIGH); // value should be incremented
//     delayMicroseconds(1);
//   }
//   digitalWrite(MCP41011_CS, HIGH);
//   delayMicroseconds(1);
//   digitalWrite(MCP41011_UD, LOW);
// }

// void MCP41011_UP(uint8_t steps) {
//   digitalWrite(MCP41011_UD, LOW);
//   delayMicroseconds(1);
//   digitalWrite(MCP41011_CS, LOW);
//   delayMicroseconds(1);
//   for(uint8_t i = 0; i < steps; ++i) {
//     digitalWrite(MCP41011_UD, HIGH);
//     delayMicroseconds(1);
//     digitalWrite(MCP41011_UD, LOW);
//     delayMicroseconds(1);
//   }
//   digitalWrite(MCP41011_CS, HIGH);
// }
/*
 */
# define TIMESTEP 200
# define POINTS_TO_BUFFER DEFAULT_QUEUE_LENGTH
FixedAverageBuffer test_output;
static bool isFirstEntry = true;
static uint8_t datablock[DATA_BLOCK_SIZE];

static uint16_t non_empty_blocks = 0;
static uint16_t total_blocks_to_read = 0;
static uint16_t blocks_to_write = 0;
static uint16_t write_from_block = 0;
//static uint8_t resistor_tap_value = 0x80;
static uint16_t adcvalue = 0;
static unsigned long time_interval_left = 0;
static uint8_t points_counter = POINTS_TO_BUFFER;
void loop() {  
  if (isFirstEntry) {
    test_output.Init();
    isFirstEntry = false;
    delay(400);
    //
    SPI.end();
    pinMode(MEM_CS, INPUT);
    /* */
    pinMode(PIN_PB5, INPUT);
    pinMode(PIN_PB3, INPUT);
    /* */
    delay(50);
    SendSingleLineCommand(3, true);
    SendSingleLineCommand(0xE7);
    SendSingleLineCommand(0xF2);
    //
# if 0     
    digitalWrite(MCP41011_CS, LOW);
    delayMicroseconds(1);
    SPI.transfer16(0x1100+resistor_tap_value);
    delayMicroseconds(1);
    digitalWrite(MCP41011_CS, HIGH);
# endif    
    time_interval_left = millis();
  }
  /*
   */
  #if 0
  while(1) {
    digitalWrite(MODULE_BUSY_LED, HIGH);
    delay(500);
    digitalWrite(MODULE_BUSY_LED, LOW);
    delay(500);
  }
  #endif
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
          SendSingleLineCommand(0xE0 + datablock[2]);
        break;
        case 'L' : // set loop play
          SendSingleLineCommand(0xF2);
        break;
      }
    }
  } else {
    Serial.flush();
  }
  //
  digitalWrite(MODULE_BUSY_LED, digitalRead(MODULE_BUSY));
  /*
   */
#if 0
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print("MCP command : "); Serial.print(cmd);
    switch(cmd) {
      case 'U' :
        Serial.println("MCP UP");
        //MCP41011_UP(1);
        if (255 > resistor_tap_value) {
          ++resistor_tap_value;
          Serial.println(resistor_tap_value);
          digitalWrite(MCP41011_CS, LOW);
          delayMicroseconds(1);
          SPI.transfer16(0x1100+resistor_tap_value);
          delayMicroseconds(1);
          digitalWrite(MCP41011_CS, HIGH);
        }
        else
         {Serial.print("Taps : "); Serial.println(resistor_tap_value);}
        break;
      case 'D' :
        Serial.println("MCP down");
        //MCP41011_DN(1);
        if (1 <= resistor_tap_value) {
          --resistor_tap_value;
          Serial.println(resistor_tap_value);
          digitalWrite(MCP41011_CS, LOW);
          delayMicroseconds(1);
          SPI.transfer16(0x1100+resistor_tap_value);
          delayMicroseconds(1);
          digitalWrite(MCP41011_CS, HIGH);
        }
        else
         {Serial.print("Taps : "); Serial.println(resistor_tap_value);}
        break;
    }
  }
#endif
  /* read voltage and convert to volts
   */
  if (TIMESTEP <= (abs(millis() - time_interval_left))) {
    time_interval_left = millis();
    adcvalue = analogRead(MCP41011_OUT);
    test_output.putValue(adcvalue);
    if (0 == points_counter) {
      points_counter = POINTS_TO_BUFFER;
      adcvalue = test_output.getValue();
      adcvalue *= BITWEIGHT;
      uint16_t volts = adcvalue / MILLIVOLT_TENTH;
      uint16_t decimal = adcvalue % MILLIVOLT_TENTH;
      Serial.print("MCP : "); Serial.print(volts);
      sprintf((char*)datablock, "%04d", decimal);
      Serial.print("."); Serial.println((char*)datablock);
    }
    --points_counter;
  }
/*
 */
/*
 */  
}
