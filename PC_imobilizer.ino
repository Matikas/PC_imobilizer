/*
  PC_imobilizer by Marius Paskevicius

  Runs on Arduino Micro or Leonardo (with Mega32U4 chip which has hardware USB for Keyboard
  library).

  PC unlocking password can be changed on serial sending command:
    set-password <password>
  password setting command can be changed on define statement for COMMAND_SET_PWD
  Password length is limited to 50 symbols (variable BUFSIZE).


  P.S. sorry for EEPROM managing functions mess at the end of the file. Should be on separate 
    library, but in this case it could not use Arduino libraries (it needs EEPPROM.h).
*/

#include <EEPROM.h>
#include <RCSwitch.h>

#define BUZ_PIN 9

#define LOCK_PROTOCOL 1
#define LOCK_BITLENGTH 24
#define LOCK_VALUE 16777215

#define UNLOCK_PROTOCOL 1
#define UNLOCK_BITLENGTH 24
#define UNLOCK_VALUE 12219748
#define COMMAND_SET_PWD "set-password"

RCSwitch mySwitch = RCSwitch();

struct rcData {
  uint8_t protocol;
  uint8_t bitLength;
  unsigned long data;
};

rcData rc;
String serialCommand;
String password;

const int BUFSIZE = 50;
char buf[BUFSIZE];

void setup() {
  Serial.begin(9600);
  mySwitch.enableReceive(1);  // Receiver on interrupt 0 => that is pin #2
  
  eeprom_read_string(0, buf, BUFSIZE);
  password = String(buf);
  clearBuffer();
  
  Keyboard.begin();
  beep();
}

void loop() {
  if (mySwitch.available()) { 
    int value = mySwitch.getReceivedValue(); 
      
    if (value == 0) { 
      Serial.print("Unknown encoding"); 
    }
    else{ 
      rc.protocol = mySwitch.getReceivedProtocol();
      rc.bitLength = mySwitch.getReceivedBitlength();
      rc.data = mySwitch.getReceivedValue();
      /*Serial.print("Received "); 
      Serial.print(rc.data); 
      Serial.print(" / "); 
      Serial.print(rc.bitLength); 
      Serial.print("bit "); 
      Serial.print("Protocol: "); 
      Serial.println(rc.protocol);*/ 
       
       if(rc.protocol == LOCK_PROTOCOL && rc.bitLength == LOCK_BITLENGTH && rc.data == LOCK_VALUE){
        Lock();
       }
       else if(rc.protocol == UNLOCK_PROTOCOL && rc.bitLength == UNLOCK_BITLENGTH && rc.data == UNLOCK_VALUE){
        Unlock();
       }
       
     } 
     
     mySwitch.resetAvailable(); 
   }

  serialCommand = Serial.readString();
  if(serialCommand.length() > 0){
    
    if(getSubstringValue(serialCommand, ' ', 0) == COMMAND_SET_PWD){
      String newPassword = getSubstringValue(serialCommand, ' ', 1);
      changePassword(newPassword);
    }
  }
}

void Lock()
{
  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.press(108);
  Keyboard.releaseAll();
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  beep();
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);
}

void Unlock()
{
  Keyboard.press(KEY_LEFT_CTRL);
  Keyboard.press(KEY_LEFT_ALT);
  Keyboard.press(KEY_DELETE);
  Keyboard.releaseAll();
  delay(500);
  Keyboard.print(password);
  Keyboard.press(KEY_RETURN);
  Keyboard.releaseAll();
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  beep();
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);
}

void changePassword(String newPass){
  password = newPass;      
  password.toCharArray(buf, BUFSIZE);
  eeprom_write_string(0, buf);
  clearBuffer();
  Serial.println("New password was set succesfully.");
}

String getSubstringValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void beep(){
  analogWrite(BUZ_PIN, 100);
  delay(200);
  analogWrite(BUZ_PIN, 50);
  delay(200);
  analogWrite(BUZ_PIN, 0);
}

void clearBuffer(){
    for (uint8_t i = 0; i < BUFSIZE; i++) {
      buf[i] = 0;
    }
}





//EEPROM stuff down here:

//
// Absolute min and max eeprom addresses.
// Actual values are hardware-dependent.
//
// These values can be changed e.g. to protect
// eeprom cells outside this range.
//
const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 1023;


void eeprom_erase_all() {
  char b = 0xff;
  int i;

  for (i = EEPROM_MIN_ADDR; i <= EEPROM_MAX_ADDR; i++) {
    EEPROM.write(i, b);
  }
}

//
// Returns true if the address is between the
// minimum and maximum allowed values,
// false otherwise.
//
// This function is used by the other, higher-level functions
// to prevent bugs and runtime errors due to invalid addresses.
//
boolean eeprom_is_addr_ok(int addr) {
  return ((addr >= EEPROM_MIN_ADDR) && (addr <= EEPROM_MAX_ADDR));
}

//
// Writes a sequence of bytes to eeprom starting at the specified address.
// Returns true if the whole array is successfully written.
// Returns false if the start or end addresses aren't between
// the minimum and maximum allowed values.
// When returning false, nothing gets written to eeprom.
//
boolean eeprom_write_bytes(int startAddr, const byte* array, int numBytes) {
  // counter
  int i;

  // both first byte and last byte addresses must fall within
  // the allowed range  
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }

  for (i = 0; i < numBytes; i++) {
    EEPROM.write(startAddr + i, array[i]);
  }

  return true;
}

//
// Reads the specified number of bytes from the specified address into the provided buffer.
// Returns true if all the bytes are successfully read.
// Returns false if the star or end addresses aren't between
// the minimum and maximum allowed values.
// When returning false, the provided array is untouched.
//
// Note: the caller must ensure that array[] has enough space
// to store at most numBytes bytes.
//
boolean eeprom_read_bytes(int startAddr, byte array[], int numBytes) {
  int i;

  // both first byte and last byte addresses must fall within
  // the allowed range  
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }

  for (i = 0; i < numBytes; i++) {
    array[i] = EEPROM.read(startAddr + i);
  }

  return true;
}

//
// Writes an int variable at the specified address.
// Returns true if the variable value is successfully written.
// Returns false if the specified address is outside the
// allowed range or too close to the maximum value
// to store all of the bytes (an int variable requires
// more than one byte).
//
boolean eeprom_write_int(int addr, int value) {
  byte *ptr;

  ptr = (byte*)&value;
  return eeprom_write_bytes(addr, ptr, sizeof(value));
}

//
// Reads an integer value at the specified address.
// Returns true if the variable is successfully read.
// Returns false if the specified address is outside the
// allowed range or too close to the maximum vlaue
// to hold all of the bytes (an int variable requires
// more than one byte).
//
boolean eeprom_read_int(int addr, int* value) {
  return eeprom_read_bytes(addr, (byte*)value, sizeof(int));
}

//
// Writes a string starting at the specified address.
// Returns true if the whole string is successfully written.
// Returns false if the address of one or more bytes
// fall outside the allowed range.
// If false is returned, nothing gets written to the eeprom.
//
boolean eeprom_write_string(int addr, char* string) {
  // actual number of bytes to be written
  int numBytes;

  // we'll need to write the string contents
  // plus the string terminator byte (0x00)
  numBytes = strlen(string) + 1;

  return eeprom_write_bytes(addr, (const byte*)string, numBytes);
}

//
// Reads a string starting from the specified address.
// Returns true if at least one byte (even only the
// string terminator one) is read.
// Returns false if the start address falls outside
// or declare buffer size os zero.
// the allowed range.
// The reading might stop for several reasons:
// - no more space in the provided buffer
// - last eeprom address reached
// - string terminator byte (0x00) encountered.
// The last condition is what should normally occur.
//
boolean eeprom_read_string(int addr, char* buffer, int bufSize) {
  // byte read from eeprom
  byte ch;

  // number of bytes read so far
  int bytesRead;

  // check start address
  if (!eeprom_is_addr_ok(addr)) {
    return false;
  }

  // how can we store bytes in an empty buffer ?
  if (bufSize == 0) {
    return false;
  }

  // is there is room for the string terminator only,
  // no reason to go further
  if (bufSize == 1) {
    buffer[0] = 0;
    return true;
  }

  // initialize byte counter
  bytesRead = 0;

  // read next byte from eeprom
  ch = EEPROM.read(addr + bytesRead);

  // store it into the user buffer
  buffer[bytesRead] = ch;

  // increment byte counter
  bytesRead++;

  // stop conditions:
  // - the character just read is the string terminator one (0x00)
  // - we have filled the user buffer
  // - we have reached the last eeprom address
  while ( (ch != 0x00) && (bytesRead < bufSize) && ((addr + bytesRead) <= EEPROM_MAX_ADDR) ) {
    // if no stop condition is met, read the next byte from eeprom
    ch = EEPROM.read(addr + bytesRead);

    // store it into the user buffer
    buffer[bytesRead] = ch;

    // increment byte counter
    bytesRead++;
  }

  // make sure the user buffer has a string terminator
  // (0x00) as its last byte
  if ((ch != 0x00) && (bytesRead >= 1)) {
    buffer[bytesRead - 1] = 0;
  }

  return true;
}
