import struct
from smbus2 import SMBus

STM32_I2C_ADDRESS = 69

def test_read(bus: SMBus):
    print("Retrieving encoder data from STM32")
    
    encoder_data = bus.read_i2c_block_data(STM32_I2C_ADDRESS, 0, 8, True)
    print(encoder_data)
    encoder_left, encoder_right = struct.unpack("ff", bytes(encoder_data))
    
    print(f"Encoder data: left={encoder_left}, right={encoder_right}")

def test_write(bus: SMBus):
    print("Writing speed commmands to STM32")
    
    cmd_left = 1.56
    cmd_right = 8.667
    
    data = struct.pack("ff", cmd_left, cmd_right)
    bus.write_i2c_block_data(STM32_I2C_ADDRESS, 1, data) 

    print(f"Encoded data: {[hex(d) for d in list(data)]}")
    print(f"Speed command data: left={cmd_left}, right={cmd_right}")

def main():
    bus = SMBus(1)
    test_read(bus)
    print()  # spacing
    test_write(bus)

if __name__ == "__main__":
    main()
