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
$ # Clone the serial-packets repository
$ git clone https://github.com/SaltyHash/serial-packets.git
$ cd serial-packets/
```

### Arduino (or compatible microcontroller)

```bash
$ # Copy the SerialPackets library into the Arduino libraries 
$ ./copy-arduino-library.sh
```

Now that the SerialPackets library has been copied into your Arduino libraries, (re)start your Arduino IDE so it can
pick up the new library.

To see example usage of the library in a simple [Echo project](/src/arduino/SerialPackets/examples/Echo/Echo.ino),
select "File > Examples > SerialPackets > Echo". If you load this into your microcontroller, it will simply read a
packet, and then write a packet containing the same data that it just received.

### Python

To use the `serialpackets.py` module in your own Python project, just make sure you have the `pyserial` library
installed, and the `serialpackets` library is in your `PYTHONPATH`, e.g.:
```bash
$ pip install pyserial    # You are in a virtualenv, right?
$ export PYTHONPATH=/path/to/serial-packets/src/python
$ python3
```
```python
>>> from serial import Serial
>>> from serialpackets import SerialPackets
>>>
>>> with Serial(...) as serial_conn:
...     packets = SerialPackets(serial_conn)
...     packets.write(b'Hello, world!')
...     print(packets.read())
...
b'Hello, how are you?'
```

If you have loaded the Echo project into your microcontroller, you can run the `serialpackets_echotest.py` module
directly to ensure it is working, and to display speed statistics:

```bash
$ cd test/python
$ export PYTHONPATH=../../src/python
$ python3 serialpackets_echotest.py <device>
e.g.
$ python3 serialpackets_echotest.py /dev/ttyACM0
Waiting 3 seconds, because connecting to the Arduino causes it to reset...

Starting echo test with:
- baud=2000000
- data_len=59

Press Ctrl+C to stop the test.

Bytes/s: 11900  Packets/s: 202  ms/packet: 4.958
Bytes/s: 13300  Packets/s: 225  ms/packet: 4.436
[...]
```

Run `python3 serialpackets_echotest.py -h` to view all the optional arguments.

## Notes
### Packet Data Length
- Make sure you don't send a packet that is so long that it overflows the microcontroller's read buffer. The maximum
  amount of data that can be sent in a single packet can be determined using this formula: \
  `max_packet_data_len = usable_read_buffer_len - 4 [packet header length]`
- For example, a typical Arduino has a read buffer that is 64 bytes long, but only 64 - 1 = 63 bytes are actually
  usable. Thus, the maximum length of data that can be sent in a single packet is **59 bytes**: \
  `(64 - 1) [usable Arduino read buffer length] - 4 [packet header length] = 59 bytes`
- If read buffer size is not an issue (i.e. for some non-Arduino microcontrollers), then the maximum length of data that
  can be sent in a single packet is **255 bytes**.

### Packet Structure
A single packet is composed of a 4-byte header, followed by the data, like so:

```
[start_byte := 0xAA | fletcher16_sum2 | fletcher16_sum1 | data_len | data_byte_1 | ... data_byte_N]
```

So as you can see, the maximum length of data that a packet can send is limited by the data_len byte, which can
represent data between 0 and 255 bytes long, inclusive.
