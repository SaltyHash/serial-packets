"""Read/write serial packets from/to an Arduino-compatible microcontroller using the corresponding C++ library."""

from threading import RLock
from typing import Any, Optional, Union, Tuple

PACKET_HEADER_LEN = 4
MAX_ALLOWED_DATA_LEN = 255
DEFAULT_MAX_PACKET_LEN = 63
DEFAULT_MAX_DATA_LEN = DEFAULT_MAX_PACKET_LEN - PACKET_HEADER_LEN

_START_BYTE = b'\xAA'


class SerialPackets:
    """Read/write serial packets from/to an Arduino-compatible microcontroller using the corresponding C++ library."""

    def __init__(self, serial_conn: Any, max_data_len: int = DEFAULT_MAX_DATA_LEN):
        """
        :param serial_conn:
        :param max_data_len: The maximum amount of data allowed to be sent in a single packet.
            Defaults to DEFAULT_MAX_DATA_LEN. Must be in range [0, MAX_ALLOWED_DATA_LEN], inclusive.
        """

        self.serial_conn = serial_conn
        self._max_data_len = max_data_len

        self._read_lock = RLock()
        self._write_lock = RLock()

    @property
    def max_data_len(self) -> int:
        """The maximum length of the data that can be sent in a packet."""
        return self._max_data_len

    @max_data_len.setter
    def max_data_len(self, max_data_len) -> None:
        """
        The maximum length of the data that can be sent in a packet.
        :param max_data_len: Must be in range [0, MAX_ALLOWED_DTA_LEN], inclusive.
        """

        if max_data_len < 0:
            raise ValueError(f'max_data_len must be >= 0; got {max_data_len}.')
        if max_data_len > MAX_ALLOWED_DATA_LEN:
            raise ValueError(f'max_data_len must be <= {MAX_ALLOWED_DATA_LEN}; got {max_data_len}.')

        self._max_data_len = max_data_len

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

    def write(self, data: Union[bytearray, bytes], flush: bool = True) -> None:
        """
        Sends a packet containing the specified data.

        This method is thread-safe.

        :param data: The data to send in the packet.
        :param flush: If True (the default), then the serial connection's "flush()" method will be called, which will
            block until all bytes have been sent.
        :raises WritePacketError: If something goes wrong writing the packet.
        """

        if len(data) > self.max_data_len:
            raise ValueError(f'data must be <= {self.max_data_len} bytes long; '
                             f'given data is {len(data)} bytes long.')

        packet = bytearray()
        packet.extend(_START_BYTE)
        packet.extend(_fletcher16(data))
        packet.append(len(data))
        packet.extend(data)

        with self._write_lock:
            if self.serial_conn.write(packet) < len(packet):
                raise WritePacketError('Failed to write all packet bytes.')

            if flush:
                try:
                    self.serial_conn.flush()
                except AttributeError:
                    pass

    def write_then_read(self, data: Union[bytearray, bytes], reset_input_buffer: bool = True) -> Optional[bytes]:
        """
        Writes a packet, and then reads a packet expected to be sent in response, and returns the data.

        This method is thread-safe.

        :param data: Bytes to send in the packet.
        :param reset_input_buffer: If True (the default), the serial connection's input buffer will be cleared,
            discarding any previously received data. This helps to ensure that the next received packet is one which
            comes after the packet we send, and is not a packet that might have been received previously.
        :return: The bytes from the received packet, or None if timed out waiting to receive a packet.
        :raises ChecksumError: If the calculated checksum of the received packet does not match the packet checksum.
        :raises WritePacketError: If something goes wrong writing the packet.
        """

        with self._write_lock, self._read_lock:
            if reset_input_buffer:
                try:
                    self.serial_conn.reset_input_buffer()
                except AttributeError:
                    pass

            self.write(data)
            return self.read()


class SerialPacketsError(Exception):
    """Base class for all errors in the serialpackets library."""
    pass


class ChecksumError(SerialPacketsError):
    """Raised when a packet's checksum does not match the calculated checksum."""

    def __init__(self, received_checksum: bytes, calculated_checksum: bytes):
        super().__init__(f'Received checksum: {list(received_checksum)}. '
                         f'Calculated checksum: {list(calculated_checksum)}.')


class WritePacketError(SerialPacketsError):
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
