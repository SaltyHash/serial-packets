/* Library for sending and receiving packets over a serial interface.
 * 
 * Packet structure:
 * - start_byte: 0xAA (1 byte)
 * - fletcher_checksum: [sum_2 | sum_1] (2 bytes)
 * - data_len: 0x00 - 0xFF (1 byte)
 * - data: [byte | ...] (data_len bytes)
 */

#include "SerialPackets.h"
#include <Arduino.h>
#include <inttypes.h>


SerialPackets::SerialPackets() {
  stream_ = &Serial;
}


SerialPackets::SerialPackets(Stream &stream) {
  stream_ = &stream;
}


int SerialPackets::Read(uint8_t data_buffer[], const uint8_t buffer_len) {
  int data_len;

  do {
    data_len = ReadNonblocking(data_buffer, buffer_len);
  } while (data_len == -1);

  return data_len;
}


int SerialPackets::ReadNonblocking(uint8_t buffer[], const uint8_t buffer_len) {
  // Discard the currently in-progress packet if there is no data to read
  // and we have reached the timeout
  if (read_packet_state_ != PacketTransitState::kStartByte
      && !stream_->available()
      && ReadTimedOut()) {
    read_packet_state_ = PacketTransitState::kStartByte;
  }

  // Receive the start byte
  if (read_packet_state_ == PacketTransitState::kStartByte) {
    // Read bytes until we get a start byte
    int datum;
    do {
      datum = ReadNonblocking();
      if (datum == -1) {
        return -1;
      }
    } while (datum != kStartByte);

    read_packet_state_ = PacketTransitState::kChecksumSum2;
  }

  // Receive checksum sum 2
  if (read_packet_state_ == PacketTransitState::kChecksumSum2) {
    const int sum_2 = ReadNonblocking();
    if (sum_2 == -1) {
      return -1;
    }

    read_packet_checksum_ = sum_2 << 8;
    read_packet_state_ = PacketTransitState::kChecksumSum1;
  }

  // Receive checksum sum 1
  if (read_packet_state_ == PacketTransitState::kChecksumSum1) {
    const int sum_1 = ReadNonblocking();
    if (sum_1 == -1) {
      return -1;
    }

    read_packet_checksum_ |= sum_1;
    read_packet_state_ = PacketTransitState::kDataLen;
  }

  // Receive the data length
  if (read_packet_state_ == PacketTransitState::kDataLen) {
    const int data_len = ReadNonblocking();
    if (data_len == -1) {
      return -1;
    }

    read_packet_data_len_ = data_len;
    read_packet_data_received_len_ = 0;
    read_packet_state_ = PacketTransitState::kData;
  }

  // Receive the data
  if (read_packet_state_ == PacketTransitState::kData) {
    while (read_packet_data_received_len_ < read_packet_data_len_) {
      const int datum = ReadNonblocking();
      if (datum == -1) {
        return -1;
      }

      // Only add datum to data if the packet isn't too long, to avoid buffer overflow
      if (read_packet_data_len_ <= buffer_len) {
        buffer[read_packet_data_received_len_] = datum;
      }

      read_packet_data_received_len_++;
    }

    read_packet_state_ = PacketTransitState::kStartByte;

    // Had to receive a packet that was too big for the buffer? Throw the packet away
    if (read_packet_data_len_ > buffer_len) {
      return -1;
    }

    // Return the length of the packet data if the checksums match; otherwise, throw the packet away
    const uint16_t calculated_checksum = Fletcher16(buffer, read_packet_data_len_);
    return read_packet_checksum_ == calculated_checksum ? read_packet_data_len_ : -1;
  }

  return -1;
}


bool SerialPackets::Write(uint8_t data[], const uint8_t data_len) {
  const uint16_t checksum = Fletcher16(data, data_len);

  int bytes_sent = stream_->write(kStartByte);
  bytes_sent += stream_->write((checksum >> 8) & 0xFF);
  bytes_sent += stream_->write(checksum & 0xFF);
  bytes_sent += stream_->write(data_len);
  bytes_sent += stream_->write(data, data_len);

  return bytes_sent == kPacketHeaderLen + data_len;
}


bool SerialPackets::Write(char data[]) {
  const uint8_t data_len = min(strlen(data), 255);
  return Write((uint8_t *)data, data_len);
}


uint16_t SerialPackets::Fletcher16(uint8_t data[], const uint8_t data_len) {
  const uint8_t modulus = 255;

  uint16_t sum_1 = data_len % modulus;
  uint16_t sum_2 = sum_1;

  for (uint8_t i = 0; i < data_len; i++) {
    sum_1 = (sum_1 + data[i]) % modulus;
    sum_2 = (sum_2 + sum_1) % modulus;
  }

  return (sum_2 << 8) | sum_1;
}


int SerialPackets::ReadNonblocking() {
  if (!stream_->available()) {
    return -1;
  }

  const int datum = stream_->read();

  if (datum != -1) {
    last_byte_read_time_ms_ = millis();
  }

  return datum;
}

bool SerialPackets::ReadTimedOut() {
  return read_packet_timeout_ms_ && (millis() - last_byte_read_time_ms_) >= read_packet_timeout_ms_;
}
