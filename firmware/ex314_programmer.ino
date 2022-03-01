/*
 *    A Programmer for ICOM EX-314 RAM Board Based on Arduino Uno
 *      Copyright (C) 2021 Zhen Ren (BG1TPT)
 */


#include <Bounce2.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <CRC32.h>

#define SRAM_WRITE_TAS 0
#define SRAM_WRITE_TWP 60
#define SRAM_WRITE_TDH 10
#define SRAM_READ_TAA 100
#define SRAM_LVDR_TCDR 100

#define BTN_PIN 19
#define WR_PIN 18
#define LED_GRN_PIN 10
#define LED_RED_PIN 11
#define RAMEN_PIN 11
#define SOFTSERIAL_TX_PIN 13
#define SOFTSERIAL_RX_PIN 12

#define SLOWLY_DELAY 500

#define BTN_RESTORE_EEPROM_PRESS_DELAY 1000
#define BTN_RESTORE_ORIGINAL_PRESS_DELAY 5000
#define BTN_PRESS_DETECT_INTERVAL 25

#define MEMORY_SIZE 1024
#define MEMORY_BUF_SIZE 512

#define SERIAL_BAUD 115200
#define LINE_BUF_SIZE 70

#define HZ 16             // Arduino Uno 16MHz crystal
#define DELAY_CYCLE 1000/HZ


#define DUMP_HEADER             F("                    +1              +2              +3               \r\n"\
                                  "     0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF\r\n"\
                                  "     ||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\r\n")


const char ORIGINAL_R71_MEM[] PROGMEM = "00F8FF8FF001FFFF000DAC34340009100AC9000A000000000800000090009000"
                                        "80000000900FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
                                        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
                                        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
                                        "FFFFFFFFFFFFFFFFFF000F8000000000000008B0000000000010A02300000000"
                                        "000013A348B100000000000000A0B00000000000903910280000000000800948"
                                        "2800000000008009B12800FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000"
                                        "0090304A9000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF00000000943AC041"
                                        "00FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF000000009A3040410000000000"
                                        "10A80008000000000010A80A10000000000010A800A0000000000010A84AC000"
                                        "0000000010A88AC0000000000010A803C000000000009030408100FFFFFFFFFF"
                                        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
                                        "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF0000000010A8C83000000000008001"
                                        "00080000000000000800080000000000000800A00000000000800100A0000000"
                                        "000010A0230000FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
                                        "FFFFFFFFFFFFFF0000000000000000000000000000000000FFF0FFFFFFFFFFB8";

typedef struct dumpinfo {
  unsigned char dumpValid;
  uint16_t dumpLength;
  uint32_t crc32;
} dumpinfo;

Bounce b = Bounce();
unsigned long btnUpTime = 0;
unsigned long btnDownTime = 0;
unsigned long btnPressTime = 0;
unsigned char membuf[MEMORY_BUF_SIZE];
unsigned char checkbuf[MEMORY_BUF_SIZE];

SoftwareSerial mySerial(SOFTSERIAL_RX_PIN, SOFTSERIAL_TX_PIN); // RX, TX

void initIO() {
  pinMode(LED_GRN_PIN, OUTPUT);
  pinMode(RAMEN_PIN, OUTPUT);
  pinMode(WR_PIN, OUTPUT);
  digitalWrite(WR_PIN, HIGH);
  digitalWrite(LED_GRN_PIN, LOW);
  digitalWrite(RAMEN_PIN, HIGH);

  b.attach (BTN_PIN, INPUT_PULLUP);
  b.interval(BTN_PRESS_DETECT_INTERVAL);
}

void initPortsForWrite() {
  // set direction
  DDRC = DDRC | 0x1F;
  DDRD = DDRD | 0xFF;
  DDRB = DDRB | 0x3;
  delay(SLOWLY_DELAY);
}

void initPortsForRead() {
  // set direction
  DDRC = DDRC & 0xF0;
  DDRD = DDRD | 0xFF;
  DDRB = DDRB | 0x7;
  delay(SLOWLY_DELAY);
}

void enterLVDRMode() {    // enter low voltage data retention mode
  pinMode(RAMEN_PIN, OUTPUT);
  digitalWrite(RAMEN_PIN, HIGH);
  __builtin_avr_delay_cycles(SRAM_LVDR_TCDR / DELAY_CYCLE);
}

void enterWorkMode() {
  pinMode(RAMEN_PIN, OUTPUT);
  digitalWrite(RAMEN_PIN, LOW);
  __builtin_avr_delay_cycles(SRAM_LVDR_TCDR / DELAY_CYCLE);  
}

void setAddress(int address) {
  unsigned char al = address & 0xFF;
  unsigned char ah = (address & 0x700) >> 8;
  PORTD = al;
  PORTB &= 0xFC;
  PORTB |= ah;
}

void setData(unsigned char data) {
  PORTC &= 0xF0;
  PORTC |= data & 0xF;
}

unsigned char readData() {
  return PINC & 0xF;
}

void writeSRAMNibble(int address, unsigned char data) {
  setAddress(address);
  setData(data);
  digitalWrite(WR_PIN, LOW);
  __builtin_avr_delay_cycles(ceil((SRAM_WRITE_TAS + SRAM_WRITE_TWP) / DELAY_CYCLE));
  digitalWrite(WR_PIN, HIGH);
  __builtin_avr_delay_cycles(ceil(SRAM_WRITE_TDH / DELAY_CYCLE));
}

unsigned char readSRAMNibble(int address) {
  digitalWrite(WR_PIN, HIGH);
  setAddress(address);
  __builtin_avr_delay_cycles(ceil(SRAM_READ_TAA / DELAY_CYCLE));
  return readData();
}

void dumpSRAM(unsigned char *data, int count, int offset) {
  digitalWrite(LED_GRN_PIN, HIGH);   
  for (int i = 0; i < count; i ++) {
    *(data + i) = readSRAMNibble(offset + i);
  }
  digitalWrite(LED_GRN_PIN, LOW);   
}

void saveToEEPROM(const unsigned char *data, int count, int offset) {
  uint16_t addr = sizeof(dumpinfo) + offset / 2;
  unsigned char bytel, byteh;
  unsigned char tmp;
  for (int i = 0; i < count; i ++) {
    if (i % 2 == 0) {
      byteh = *(data + i);
    } else {
      bytel = *(data + i);
      tmp = (bytel & 0xF) | ((byteh & 0xF) << 4);
      EEPROM.update(addr + i / 2, tmp);
    }
  }
}

void restoreSRAM(const unsigned char *data, int count, int offset) {
  digitalWrite(LED_GRN_PIN, HIGH);   
  for (int i = 0; i < count; i ++) {
    writeSRAMNibble(offset + i, *(data + i));
  }
  digitalWrite(LED_GRN_PIN, LOW);   
}

int loadFromEEPROM(unsigned char *data, int count, int offset) {
  dumpinfo info;
  EEPROM.get(0, info);
  if (info.dumpValid != 1) return -1;
  uint16_t addr = sizeof(dumpinfo) + offset / 2;
  unsigned char bytel, byteh;
  unsigned char tmp;
  for (int i = 0; i < count / 2; i ++) {
    tmp = EEPROM.read(addr + i);
    bytel = tmp & 0xF;
    byteh = (tmp >> 4) & 0xF;
    *(data + 2 * i) = byteh;
    *(data + 2 * i + 1) = bytel;
  }
}

uint32_t calcEEPROMCRC(int count) {
  uint16_t addr = sizeof(dumpinfo);
  uint32_t crcVal;
  CRC32 crc;
  for (int i = 0; i < count / 2; i ++) {
    crc.update(EEPROM.read(addr + i));
  }
  crcVal = crc.finalize();
  return crcVal;
}

void validEEPROM(int count) {
  uint32_t crc = calcEEPROMCRC(count);
  dumpinfo info;
  info.dumpValid = 1;
  info.dumpLength = count;
  info.crc32 = crc;
  EEPROM.put(0, info);
}

void convertFMemA2B(const char *memStr, unsigned char *data, int count) {
  char cur = pgm_read_byte_near(memStr);

  int i = 0;
  while (cur) {
    *(data + i) = h2c(cur);
    i ++;
    if (i >= count) break;
    cur = pgm_read_byte_near(++memStr);
  }
}

char c2h(unsigned char c) {
  return "0123456789ABCDEF"[0x0F & c];
}

unsigned char h2c(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  } else if (c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  } else if (c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  } else {
    // data error
  }
}

void serialPrintDumpMem(const unsigned char *data, int count, int offset) {
  char linebuf[LINE_BUF_SIZE];
  int line = 0;
  int i = 0;

  while (line < count/64) {
    memset(linebuf, 0, LINE_BUF_SIZE);
    snprintf(linebuf, LINE_BUF_SIZE, "%03X: ", offset + line * 64);
    for (int i=0; i < 64; i ++) {
      *(linebuf + 5 + i) = c2h(*(data + line * 64 + i));
    }
    mySerial.println(linebuf);
    line ++;
  }
}

void serialPrintDumpHeader() {
  mySerial.print(DUMP_HEADER);
}

void restoreData(int fromEEPROM) {
  char msg[64];
  int cmp;
  
  for (int i = 0; i < MEMORY_SIZE/MEMORY_BUF_SIZE; i ++) {
    memset(membuf, 0, MEMORY_BUF_SIZE);
    memset(checkbuf, 0, MEMORY_BUF_SIZE);
    if (fromEEPROM) {
      loadFromEEPROM(membuf, MEMORY_BUF_SIZE, i * MEMORY_BUF_SIZE);
    } else {
      convertFMemA2B(ORIGINAL_R71_MEM+i*MEMORY_BUF_SIZE, membuf, MEMORY_BUF_SIZE);
    }
    snprintf(msg, sizeof(msg), "Writing 0x%02X-0x%02X...", i * MEMORY_BUF_SIZE, (i + 1) * MEMORY_BUF_SIZE - 1);
    mySerial.println(msg);
    initPortsForWrite();
    restoreSRAM(membuf, MEMORY_BUF_SIZE, i * MEMORY_BUF_SIZE);
    snprintf(msg, sizeof(msg), "Verifying 0x%02X-0x%02X...", i * MEMORY_BUF_SIZE, (i + 1) * MEMORY_BUF_SIZE - 1);
    mySerial.println(msg);
    initPortsForRead();
    dumpSRAM(checkbuf, MEMORY_BUF_SIZE, i * MEMORY_BUF_SIZE);
    serialPrintDumpHeader();
    serialPrintDumpMem(checkbuf, MEMORY_BUF_SIZE, i * MEMORY_BUF_SIZE);
    cmp = memcmp(membuf, checkbuf, MEMORY_BUF_SIZE);
    if (cmp) {
      mySerial.println("Memory verify failed");
    } else {
      mySerial.println("Memory verify OK.");
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  initIO();
  enterLVDRMode();
  mySerial.begin(SERIAL_BAUD);
}

void loop() {
  // put your main code here, to run repeatedly:
  b.update();
  if (b.fell()) {
    btnDownTime = millis();
  }
  
  if (b.rose()) {
    btnUpTime = millis();
    btnPressTime = btnUpTime - btnDownTime;
    enterWorkMode();
    if (btnPressTime > BTN_RESTORE_ORIGINAL_PRESS_DELAY) {
      mySerial.println("Reset SRAM to original.");
      // restore SRAM        
      restoreData(0);
    } else if (btnPressTime >= BTN_RESTORE_EEPROM_PRESS_DELAY && btnPressTime < BTN_RESTORE_ORIGINAL_PRESS_DELAY) {
      // restore from EEPROM
      mySerial.println("Restore SRAM from EEPROM.");
      restoreData(1);
    } else {
      // dump SRAM
      mySerial.println("Dump SRAM");
      initPortsForRead();
      serialPrintDumpHeader();

      for (int i = 0; i < MEMORY_SIZE/MEMORY_BUF_SIZE; i ++) {
        memset(membuf, 0, MEMORY_BUF_SIZE);
        dumpSRAM(membuf, MEMORY_BUF_SIZE, i * MEMORY_BUF_SIZE);
        saveToEEPROM(membuf, MEMORY_BUF_SIZE, i * MEMORY_BUF_SIZE);
        serialPrintDumpMem(membuf, MEMORY_BUF_SIZE, i * MEMORY_BUF_SIZE);
      }
      validEEPROM(MEMORY_SIZE);
    }
    btnPressTime = 0;
    initIO();
    enterLVDRMode();
  }
}
