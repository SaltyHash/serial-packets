import argparse
import random
import serial
import sys
import time

try:
    import serialpackets
except ImportError as e:
    print('Error:', e)
    print('Did you forget to add serialpackets to the PYTHONPATH?')
    print('export PYTHONPATH=../../src/python')
    sys.exit(1)


def speed_test(packets: serialpackets.SerialPackets):
    random.seed(0)
    data_len = packets.get_max_data_len()

    print(f'Starting echo test with:')
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
            except (serialpackets.ChecksumError, serialpackets.WritePacketError) as e:
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


def main():
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
        '--data-len', default=serialpackets.DEFAULT_MAX_DATA_LEN, type=int,
        help=f'Number of bytes to send. Defaults to DEFAULT_MAX_DATA_LEN={serialpackets.DEFAULT_MAX_DATA_LEN} bytes.'
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

    with serial.Serial(args.device, baudrate=args.baud, timeout=args.timeout) as serial_conn:
        packets = serialpackets.SerialPackets(serial_conn, max_data_len=args.data_len)

        print('Waiting 3 seconds, because connecting to the Arduino causes it to reset...')
        time.sleep(3)

        try:
            speed_test(packets)
        except KeyboardInterrupt:
            print()


if __name__ == '__main__':
    main()
