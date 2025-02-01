import argparse
import time
import pandas
from smbus2 import SMBus
import struct

STM32_I2C_ADDRESS = 0x45
STM32_I2C_BUS = 1


def main():
    parser = argparse.ArgumentParser(
        "collect_motor_speeds",
        "Collects motor speeds from the encocders when running at a set pwm",
    )
    parser.add_argument("log_file_prefix", type=str)
    parser.add_argument("set_pwm_duty", type=float)
    parser.add_argument("sampling_freq", type=float)
    parser.add_argument("sampling_duration", type=float)
    args = parser.parse_args()

    bus = SMBus(STM32_I2C_BUS)

    df = pandas.DataFrame(columns=["time", "left_speed_radps", "right_speed_radps"])
    start_time = time.time()

    sampling_period = 1 / args.sampling_freq

    print(
        f"Starting sampling: freq={args.sampling_freq}, duration={args.sampling_duration}"
    )

    while True:
        current_time = time.time()
        if current_time - start_time > args.sampling_duration:
            break

        # reads the wheel speeds from the encoders and logs it
        encoder_left, encoder_right = read_encoder_speeds(bus)
        df.loc[len(df)] = [current_time, encoder_left, encoder_right]

        print(
            f"Sampled: time={current_time}, left={encoder_left}, right={encoder_right}"
        )

        # delay to the next reading
        time.sleep(sampling_period)

    df.to_csv(
        f"{args.log_file_prefix}_pwm_duty_{args.set_pwm_duty}_sample_freq_{args.sampling_freq}_sample_dur_{args.sampling_duration}.csv"
    )
    print("Done!")


def read_encoder_speeds(bus: SMBus):
    encoder_data = bus.read_i2c_block_data(STM32_I2C_ADDRESS, 0, 8, True)
    encoder_left, encoder_right = struct.unpack("ff", bytes(encoder_data))

    return (encoder_left, encoder_right)


if __name__ == "__main__":
    main()
