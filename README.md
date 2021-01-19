# serial-packets
Simple serial packet library with error detection for Arduino and Python.

The design goals of this library include:
- *Re-syncing* if a packet is only partially sent, by using a start byte to mark the beginning of a packet.
- *Detecting packet errors* such as corrupted or lost bits/bytes, via a
  [Fletcher16 checksum](https://en.wikipedia.org/wiki/Fletcher%27s_checksum#Fletcher-16) over the data.
- *Asynchronous packet reading* in the Arduino library so the microcontroller can continue doing other real-time tasks
  while it receives the packet as data becomes available.

## Getting Started

```bash
# Clone the serial-packets repository
git clone https://github.com/SaltyHash/serial-packets.git
cd serial-packets/
```

### Arduino (or compatible microcontroller)

```bash
# Copy the SerialPackets library into the Arduino libraries 
./copy-arduino-library.sh
```

Now that the SerialPackets library has been copied into your Arduino libraries, (re)start your Arduino IDE so it can
pick up the new library.

To see example usage of the library in a [simple echo application](/src/arduino/SerialPackets/examples/Echo/Echo.ino),
select "File > Examples > SerialPackets > Echo". If you load this into your microcontroller, it will simply read a
packet, and then write a packet containing the same data that it just received.

### Python

```bash
cd src/python/
python3 serial_packets.py
```

## Notes

- Make sure you don't send a packet that is so long that it overflows the microcontroller's read buffer. The maximum
  amount of data that can be sent in a single packet can be determined using this formula: \
  `max_packet_data_len = usable_read_buffer_len - 4 [packet header length]`
- For example, a typical Arduino has a read buffer that is 64 bytes long, but only 64 - 1 = 63 bytes are actually
  usable. Thus, the maximum length of data that can be sent in a single packet is **59 bytes**: \
  `(64 - 1) [usable Arduino read buffer length] - 4 [packet header length] = 59 bytes`
- If read buffer size is not an issue (i.e. for some non-Arduino microcontrollers), then the maximum length of data that
  can be sent in a single packet is **255 bytes**.
