import serial
import time

if __name__ == "__main__":
    ser = serial.Serial()
    ser.port = "/dev/ttyACM0"
    ser.timeout = 1
    size = 2**15
    read_page_cmd = bytearray(b'\x06\x00\x00\x00')
    write_page_cmd = bytearray(b'\x03\x00\x00\x00')
    connect_cmd = bytearray(b'\x00\x00\x00\x00')
    to_write = bytearray(b'\x05' * size)

    with ser as blue:
        blue.write(connect_cmd)
        blue.read(1)
        start_write = time.time()
        # Write
        for addr in range(0, size, 64):
            write_page_cmd[1:3] = addr.to_bytes(2, byteorder="little")
            blue.write(write_page_cmd)
            response = blue.read(1)
            if response != b'\x09':
                print("ERROR cmd, response = {}".format(response))
                break
            blue.write(to_write[addr: addr + 64])
            response = blue.read(1)
            if response != b'\x02':
                print("ERROR writing, addr = {}, response = {}".format(
                    addr, response))

        end_write = time.time()
        print("Write took {}s".format(end_write - start_write))

        # Read
        with open("dump", "wb") as out_file:
            for addr in range(0, size, 64):
                read_page_cmd[1:3] = addr.to_bytes(2, byteorder="little")
                blue.write(read_page_cmd)
                page = blue.read(64)
                out_file.write(page)

        end_read = time.time()
        print("Reading took {}s".format(end_read - end_write))
