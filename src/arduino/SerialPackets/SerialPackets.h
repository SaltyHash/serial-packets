#ifndef SERIAL_PACKETS_H_
#define SERIAL_PACKETS_H_

#include <inttypes.h>
#include <Stream.h>


class SerialPackets {
  public:
    /* How many bytes are taken up by the packet header information.
     * Useful for calculating maximum data buffer sizes.
     */
    static const uint8_t kPacketHeaderLen = 4;

    /* If we are in the middle of receiving a packet, but it takes longer than this
     * to receive the next byte, then the packet is considered lost.
     */
    unsigned long read_packet_timeout_ms_ = 1000;

    /* Initialize using the Serial object as the stream. */
    SerialPackets();

    /* Initialize using whatever Serial/Stream object you want. */
    SerialPackets(Stream &stream);

    /* Blocks until a valid packet is received, and returns the length of the
     * packet data now stored in data_buffer.
     */
    int Read(uint8_t data_buffer[], const uint8_t buffer_len);

    /* Receives a packet, but does NOT block if the packet has not yet been fully received.
     * This is very useful in real-time control applications where it is important for code
     * not to block waiting for I/O.
     * 
     * Returns a -1 if a valid packet has not yet been fully received, but once a valid packet
     * HAS been fully received, the length of the packet data now stored in data_buffer is returned.
     */
    int ReadNonblocking(uint8_t data_buffer[], const uint8_t buffer_len);

    /* Sends a packet containing the specified data. */
    bool Write(uint8_t data[], const uint8_t data_len);

    /* Sends a packet containing the specified string. */
    bool Write(char data[]);

  private:
    enum class PacketTransitState {
      kStartByte,
      kChecksumSum2,
      kChecksumSum1,
      kDataLen,
      kData
    };

    static const uint8_t kStartByte = 0xAA;

    Stream * stream_;

    // Variables for keeping track of which part of the packet has been received
    PacketTransitState read_packet_state_ = PacketTransitState::kStartByte;
    uint16_t read_packet_checksum_;
    uint8_t read_packet_data_len_;
    uint8_t read_packet_data_received_len_;
    unsigned long last_byte_read_time_ms_ = 0;

    static uint16_t Fletcher16(uint8_t data[], const uint8_t data_len);
    int ReadNonblocking();
    bool ReadTimedOut();
};

#endif
