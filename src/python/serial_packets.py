"""Read/write serial packets from/to an Arduino-compatible microcontroller using the corresponding C++ library."""

from threading import RLock
from typing import Any, Optional, Union, Tuple

DEFAULT_MAX_PACKET_LEN = 63

_PACKET_HEADER_LEN = 4
_MAX_ALLOWED_PACKET_DATA_LEN = 255
_START_BYTE = b'\xAA'


class SerialPackets:
    """"""

    def __init__(self, serial_conn: Any, max_packet_len: int = DEFAULT_MAX_PACKET_LEN):
        """
        :param serial_conn:
        :param max_packet_len: Defaults to DEFAULT_MAX_PACKET_LEN, which is the usable size of an Arduino's read buffer.
        """

        self.serial_conn = serial_conn

        if max_packet_len < _PACKET_HEADER_LEN:
            raise ValueError(f'max_packet_len must be >= {_PACKET_HEADER_LEN}; got {max_packet_len}.')
        max_allowed_packet_len = _PACKET_HEADER_LEN + _MAX_ALLOWED_PACKET_DATA_LEN
        if max_packet_len > max_allowed_packet_len:
            raise ValueError(f'max_packet_len must be <= {max_allowed_packet_len}; got {max_packet_len}.')
        self._max_packet_data_len = max_packet_len - _PACKET_HEADER_LEN

        self._read_lock = RLock()
        self._write_lock = RLock()

    def __enter__(self):
        if hasattr(self.serial_conn, '__enter__'):
            self.serial_conn.__enter__()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if hasattr(self.serial_conn, '__exit__'):
            return self.serial_conn.__exit__(exc_type, exc_val, exc_tb)

    def get_max_packet_data_len(self) -> int:
        """Returns the maximum length of the data that can be sent/received in a packet."""
        return self._max_packet_data_len

    def read_packet(self) -> Optional[bytes]:
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

    def write_packet(self, data: Union[bytearray, bytes]) -> None:
        """
        Returns True if the packet was sent, or False if it was not (in which case it may have been partially sent).

        This method is thread-safe.

        :raises WritePacketError: If something goes wrong writing the packet.
        """

        if len(data) > self._max_packet_data_len:
            raise ValueError(f'data must be <= {self._max_packet_data_len} bytes long; '
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

            self.write_packet(data)
            return self.read_packet()


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


# def _get_checksum(data: Union[bytearray, bytes]) -> int:
#     checksum = len(data)
#
#     for byte in data:
#         checksum ^= byte
#
#     checksum = ~checksum & 0xFF
#     return checksum


def _echo_test(packets: SerialPackets):
    while True:
        packets.write_packet(input('Write: ').encode())
        print('Reading...')
        print(f'Read: {packets.read_packet()}')


def _speed_test(packets: SerialPackets):
    import random
    import time

    # max_data_len = packets.get_max_packet_data_len()
    max_data_len = 59

    while True:
        total_bytes = 0
        total_packets = 0
        total_time = 0
        while total_time < 1:
            data_len = random.randint(0, max_data_len)
            data_len = max_data_len

            data = bytes(random.randint(0, 255) for _ in range(data_len))

            t0 = time.time()
            try:
                response = packets.write_then_read_packet(data)
            except (ChecksumError, WritePacketError) as e:
                print(e)
                continue
            total_bytes += data_len + len(response)
            total_packets += 2
            total_time += time.time() - t0

            if response != data:
                print('ERROR:')
                print(f'- Expected: {data}')
                print(f'- Received: {response}')

        bytes_per_second = round(total_bytes / total_time)
        packets_per_second = round(total_packets / total_time)
        ms_per_packet = round(1000 * total_time / total_packets, 3)
        print(f'Bytes/s: {bytes_per_second} \tPackets/s: {packets_per_second} \tms/packet: {ms_per_packet}')


def _main():
    import random
    import serial

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

    with SerialPackets(TestSerial('/dev/ttyACM0', baudrate=2000000, timeout=0.3, read_error_rate=0.000,
                                  write_error_rate=0.000)) as packets:
        import time; time.sleep(2)
        _speed_test(packets)
        # _echo_test(packets)


if __name__ == '__main__':
    _main()
