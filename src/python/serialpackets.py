"""Read/write serial packets from/to an Arduino-compatible microcontroller using the corresponding C++ library."""

from threading import RLock
from typing import Any, Optional, Union, Tuple

PACKET_HEADER_LEN = 4
MAX_ALLOWED_DATA_LEN = 255
DEFAULT_MAX_PACKET_LEN = 63
DEFAULT_MAX_DATA_LEN = DEFAULT_MAX_PACKET_LEN - PACKET_HEADER_LEN

_START_BYTE = b'\xAA'


class SerialPackets:
    """"""

    def __init__(self, serial_conn: Any, max_data_len: int = DEFAULT_MAX_DATA_LEN):
        """
        :param serial_conn:
        :param max_data_len: The maximum amount of data allowed to be sent in a single packet.
            Defaults to DEFAULT_MAX_PACKET_DATA_LEN. Must be in range [0, MAX_ALLOWED_PACKET_DATA_LEN], inclusive.
        """

        self.serial_conn = serial_conn

        if max_data_len < 0:
            raise ValueError(f'max_packet_data_len must be >= 0; got {max_data_len}.')
        if max_data_len > MAX_ALLOWED_DATA_LEN:
            raise ValueError(f'max_data_len must be <= {MAX_ALLOWED_DATA_LEN}; got {max_data_len}.')
        self._max_data_len = max_data_len

        self._read_lock = RLock()
        self._write_lock = RLock()

    def get_max_data_len(self) -> int:
        """Returns the maximum length of the data that can be sent in a packet."""
        return self._max_data_len

    def read(self) -> Optional[bytes]:
        """
        Blocks until a valid packet is received, or a serial timeout occurs. If a valid packet is received, then the
        data bytes are returned. If a serial timeout occurs, then None is returned.

        This method is thread-safe.

        :raises ChecksumError: If the calculated checksum does not match the packet checksum.
        """

        with self._read_lock:
            # Read until we hit the start byte
            while True:
                byte = self.serial_conn.read()
                if byte == _START_BYTE:
                    break
                if not byte:
                    return None

            received_checksum = self.serial_conn.read(2)
            if len(received_checksum) < 2:
                return None

            data_len = self.serial_conn.read()
            if not data_len:
                return None
            data_len = data_len[0]

            data = self.serial_conn.read(data_len)
            if len(data) < data_len:
                return None

        calculated_checksum = bytes(_fletcher16(data))
        if calculated_checksum != received_checksum:
            raise ChecksumError(received_checksum, calculated_checksum)

        return data

    def write(self, data: Union[bytearray, bytes]) -> None:
        """
        Returns True if the packet was sent, or False if it was not (in which case it may have been partially sent).

        This method is thread-safe.

        :raises WritePacketError: If something goes wrong writing the packet.
        """

        if len(data) > self._max_data_len:
            raise ValueError(f'data must be <= {self._max_data_len} bytes long; '
                             f'given data is {len(data)} bytes long.')

        packet = bytearray()
        packet.extend(_START_BYTE)
        packet.extend(_fletcher16(data))
        packet.append(len(data))
        packet.extend(data)

        with self._write_lock:
            if self.serial_conn.write(packet) < len(packet):
                raise WritePacketError('Failed to write all packet bytes.')

            self.serial_conn.flush()

    def write_then_read_packet(self, data: Union[bytearray, bytes], reset_input_buffer: bool = True) -> Optional[bytes]:
        """
        Writes a packet, and then reads a packet expected to be sent in response.

        This method is thread-safe.

        :param data: Bytes to send in the packet.
        :param reset_input_buffer: If True, the serial connection's input buffer will be cleared, discarding any
            previously received data. This helps to ensure that the next received packet is one which comes after the
            packet we send, and is not a packet that might have been received previously.
        :return: The bytes from the received packet, or None if timed out waiting to receive a packet.
        """

        with self._write_lock, self._read_lock:
            if reset_input_buffer:
                try:
                    self.serial_conn.reset_input_buffer()
                except AttributeError:
                    pass

            self.write(data)
            return self.read()


class ChecksumError(Exception):
    """Raised when a packet's checksum does not match the calculated checksum."""

    def __init__(self, received_checksum: bytes, calculated_checksum: bytes):
        super().__init__(f'Received checksum: {list(received_checksum)}. '
                         f'Calculated checksum: {list(calculated_checksum)}.')


class WritePacketError(Exception):
    """Raised when we fail to write a packet."""
    pass


def _fletcher16(data: Union[bytearray, bytes]) -> Tuple[int, int]:
    """
    Implementation of the Fletcher 16 checksum algorithm.

    https://en.wikipedia.org/wiki/Fletcher%27s_checksum

    Note: The two sums are initialized using the length of the data.

    :param data:
    :return: A tuple of (sum_2, sum_1), where sum_1 is the simple modulus.
    """

    modulus = 255
    sum_1 = sum_2 = len(data) % modulus

    for byte in data:
        sum_1 = (sum_1 + byte) % modulus
        sum_2 = (sum_2 + sum_1) % modulus

    return sum_2, sum_1


def _speed_test(packets: SerialPackets):
    import random
    import time

    random.seed(0)
    data_len = packets.get_max_data_len()

    print(f'Starting speed test with:')
    print(f'- baud={packets.serial_conn.baudrate}')
    print(f'- data_len={data_len}')
    print()

    while True:
        total_bytes = 0
        total_packets = 0
        total_time = 0

        while total_time < 1:
            data = bytes(random.randint(0, 255) for _ in range(data_len))

            start_time = time.time()
            try:
                response = packets.write_then_read_packet(data)
            except (ChecksumError, WritePacketError) as e:
                print(e)
                continue
            total_time += time.time() - start_time
            total_bytes += data_len
            total_packets += 1

            if response != data:
                print('ERROR:')
                print(f'- Expected: {data}')
                print(f'- Received: {response}')

        bytes_per_second = round(total_bytes / total_time)
        packets_per_second = round(total_packets / total_time)
        ms_per_packet = round(1000 * total_time / total_packets, 3)
        print(f'Bytes/s: {bytes_per_second} \tPackets/s: {packets_per_second} \tms/packet: {ms_per_packet}')


def _main():
    import argparse
    import random
    import serial
    import time

    class TestSerial(serial.Serial):
        def __init__(self, *args, read_error_rate: float = 0.0, write_error_rate: float = 0.0, **kwargs):
            super().__init__(*args, **kwargs)
            self.read_error_rate = read_error_rate
            self.write_error_rate = write_error_rate

        def reset_input_buffer(self):
            return super().reset_input_buffer()

        def read(self, *args, **kwargs):
            result = super().read(*args, **kwargs)
            return b'' if random.random() < self.read_error_rate else result

        def write(self, data):
            new_data = bytearray()
            for datum in data:
                if random.random() >= self.write_error_rate:
                    new_data.append(datum)
            return super().write(new_data)

    parser = argparse.ArgumentParser(
        description=r'''
        If you have the Echo example project loaded into your microcontroller, running this module will perform a "speed
        test" that will write packets of random data to your microcontroller, read the response packets, and print
        statistics about byte and packet rates.
        ''',
        epilog=r'''Simple example: python3 serial_packets.py /dev/ttyACM0'''
    )

    parser.add_argument(
        'device',
        help='On Linux, this is something like "/dev/ttyACM0". '
             'On Windows, this is something like "COM0".'
    )
    parser.add_argument(
        '--data-len', default=DEFAULT_MAX_DATA_LEN, type=int,
        help=f'Number of bytes to send. Defaults to DEFAULT_MAX_DATA_LEN={DEFAULT_MAX_DATA_LEN} bytes.'
    )
    parser.add_argument(
        '--baud', default=2000000, type=int,
        help='Baud rate. Defaults to 2000000, which is the fastest the Arduino (ATmega328P @ 16MHz) can handle.'
    )
    parser.add_argument(
        '--timeout', default=1.0, type=float,
        help='Serial read/write timeout. Defaults to 1.0 seconds.'
    )

    args = parser.parse_args()

    with TestSerial(args.device, baudrate=args.baud, timeout=args.timeout,
                    read_error_rate=0.000, write_error_rate=0.000) \
            as serial_conn:
        packets = SerialPackets(serial_conn, max_data_len=args.data_len)

        print('Waiting 3 seconds, because connecting to the Arduino causes it to reset...')
        time.sleep(3)

        try:
            _speed_test(packets)
        except KeyboardInterrupt:
            print()


if __name__ == '__main__':
    _main()
