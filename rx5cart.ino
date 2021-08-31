#include <CRC32.h>
#include <SoftwareSerial.h>

/* Data needs to be transfered in blocks, because the
   teensy only has 8k of RAM, but we need 128k */
#define BLOCKSIZE 0x1000

#define RX5DETECT 18 // PE6
#define WE 25 // PB5
#define OE 24 // PB4
#define CE0 23 // PB3
#define CE1 22 // PB2
#define A16 20 // PB0

char readRAM(uint32_t addr);
void writeRAM(uint32_t addr, char data);

SoftwareSerial debugSer = SoftwareSerial(8, 9);

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(30000);

  // Wait for unplug
  pinMode(RX5DETECT, INPUT);
  while (digitalRead(RX5DETECT)) {
    delay(500);
  }
  delay(200); // Allow cartrigde to be fully removed

  pinMode(8, INPUT);
  pinMode(9, OUTPUT);
  debugSer.begin(9600);

  // Set ports B, C, D to output
  DDRB = 0xFF;
  DDRC = 0xFF;
  DDRD = 0xFF;

  // Disable everything
  PORTB = 0b00111100;

  debugSer.println("Waiting for start symbol...");

  // Get start signal
  while (!Serial.available());
  char operation = Serial.read();

  CRC32 crc;
  char buff[BLOCKSIZE];

  if (operation == 'r') {

    debugSer.println("Reading data...");

    // Read block
    for (unsigned int i = 0; i < 0x20000 / BLOCKSIZE; i++) {
      for (unsigned int j = 0; j < BLOCKSIZE; j++) {
        buff[j] = readRAM((i * BLOCKSIZE) + j);
        crc.update(buff[j]); // Add to CRC buffer
      }

      Serial.write(buff, BLOCKSIZE);
    }

    debugSer.println("Read data");
    
    // Get checksum
    while (!Serial.available());
    debugSer.println("Serial available");
    uint32_t remotecrc = 0;
    debugSer.print("Read bytes: ");
    debugSer.println(Serial.readBytes((char *)&remotecrc, 4));

    debugSer.print("Read checksum: ");
    debugSer.println((unsigned long) remotecrc);

    uint32_t validcrc = crc.finalize(); // Calculate checksum
    debugSer.println("Calculated checksum");
    
    if (remotecrc == validcrc) {
      Serial.write(1);
      debugSer.println("All good!");
    } else {
      Serial.write(2);
      debugSer.println("Checksums don't match");
    }
    
  } else if (operation == 'w') {

    debugSer.println("Receiving data...");

    uint32_t pos = 0;

    //for (unsigned int i = 0; i < 0x20000; i++) {
    while (pos < 0x20000) {

      unsigned int bytesavailable = 0;
      while (!(bytesavailable = Serial.available()));
      Serial.readBytes(buff, bytesavailable); // Receive block
  
      // Write block
      for (unsigned int j = 0; j < bytesavailable; j++) {
        writeRAM(pos+j, buff[j]);
        crc.update(buff[j]); // Add to CRC buffer
      }

      pos += bytesavailable;

      Serial.write(1); // Acknowledgement
    }

    debugSer.println("Calculating checksum...");
    uint32_t mycrc = crc.finalize(); // Calculate checksum
    
    debugSer.print("Sending checksum: ");
    debugSer.println(mycrc);
    
    Serial.write((char *)&mycrc, 4);

    debugSer.println("Done!");
  }
}

void loop() {
}

char readRAM(uint32_t addr) {
  bool a16 = (addr >> 16) & 0x1;
  bool a17 = (addr >> 17) & 0x1;

  digitalWrite(WE, HIGH);
  digitalWrite(OE, LOW);
  
  digitalWrite(CE0, a17);
  digitalWrite(CE1, !a17);
  
  DDRF = 0x00; // Set data pins to input

  PORTD = addr & 0xFF; // Set lower byte of address
  PORTC = (addr >> 8) & 0xFF; // Set upper byte of address
  digitalWrite(A16, a16); // Set last bit

  return PINF; // Return data
}

void writeRAM(uint32_t addr, char data) {
  bool a16 = (addr >> 16) & 0x1;
  bool a17 = (addr >> 17) & 0x1;

  digitalWrite(OE, HIGH);

  digitalWrite(CE0, a17);
  digitalWrite(CE1, !a17);

  DDRF = 0xFF; // Set data pins to output

  // Set address
  PORTD = addr & 0xFF; // Set lower byte of address
  PORTC = (addr >> 8) & 0xFF; // Set upper byte of address
  digitalWrite(A16, a16); // Set last bit

  // Set data pins
  PORTF = data;

  // Write data
  digitalWrite(WE, LOW);
  digitalWrite(WE, HIGH);
}
