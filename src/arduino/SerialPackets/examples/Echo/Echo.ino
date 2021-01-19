#include <SerialPackets.h>

// On most Arduinos, the serial buffer is 64 bytes long, but only 63 are actually useable
const uint8_t serial_buffer_size = 64 - 1;

// Create the buffer for the read packet data
const uint8_t read_buffer_len = serial_buffer_size - SerialPackets::kPacketHeaderLen;
uint8_t read_buffer[read_buffer_len];

// Create the SerialPackets instance. It defaults to using the "standard" Serial object.
SerialPackets serial_packets;

void setup() {
  // The Arduino Uno can use a baud rate up to 2M!
  Serial.begin(2000000);
}

void loop() {
  // Try to read a packet
  // This function is non-blocking, so if a packet has not been fully received yet,
  // this function will not prevent your other code from executing.
  const int data_len = serial_packets.ReadPacketNonblocking(read_buffer, read_buffer_len);

  // A packet has been successfully received?
  if (data_len != -1) {
    // Echo the data right back
    serial_packets.WritePacket(read_buffer, data_len);
  }
}
