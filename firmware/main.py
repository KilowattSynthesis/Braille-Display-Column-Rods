from machine import ADC, Pin, I2C
import time
import random

# Sleep a bit longer in init to ensure it gets set right.
SHIFT_REGISTER_INIT_SLEEP_MS = 100

# Sleep time between each bit shift.
SHIFT_REGISTER_SLEEP_US = 100

# Pin definitions for shift register control
PIN_SHIFT_SER_IN = Pin(2, Pin.OUT)  # GP2: Serial data input
PIN_SHIFT_SRCK = Pin(3, Pin.OUT)  # GP3: Shift register clock
PIN_SHIFT_N_SRCLR = Pin(4, Pin.OUT)  # GP4: Shift register clear
PIN_SHIFT_RCLK = Pin(5, Pin.OUT)  # GP5: Register clock (latch)
PIN_SHIFT_N_OE = Pin(6, Pin.OUT)  # GP6: Output enable

# Pin definitions for general purpose LEDs and switches.
PIN_SW1 = Pin(28, Pin.IN, Pin.PULL_UP)
PIN_SW2 = Pin(27, Pin.IN, Pin.PULL_UP)
PIN_GP_LED_0 = Pin(7, Pin.OUT)
PIN_GP_LED_1 = Pin(8, Pin.OUT)

# Pin definitions for hall sensor control (mux and ADC).
PIN_HALL_SELECT_0 = Pin(9, Pin.OUT)
PIN_HALL_SELECT_1 = Pin(10, Pin.OUT)
PIN_HALL_SELECT_2 = Pin(11, Pin.OUT)
PIN_HALL_SELECT_3 = Pin(12, Pin.OUT)
PIN_HALL_ADC = Pin(26, Pin.IN)
HALL_ADC = ADC(PIN_HALL_ADC)

# Pin definitions for the steppers.
I2C_BUS = I2C(1, scl=Pin(15), sda=Pin(14), freq=100000)


# References:
# - https://www.ti.com/lit/ds/symlink/drv8847.pdf
# - https://www.tij.co.jp/lit/ug/tidued1a/tidued1a.pdf (start around Page 14/35)

# Notes:
# - Set nSLEEP to 1 to use the device (or pull it up).
# - Pull nFAULT LOW to release/sleep them. Pull nFAULT HIGH to talk to them.
# - Reprogram the addresses one-by-one using nFAULT.

# 0x60 = 96
DRV8847S_DEFAULT_I2C_ADDR = 0x60  # 7-bit address


REGISTER_ADDRESS_IC1_CONTROL = 0x01  # Control register address
REGISTER_ADDRESS_IC2_CONTROL = 0x02  # Control register address
REGISTER_ADDRESS_FAULT_STATUS_2 = 0x04  # Fault status register address

HALF_STEP_SEQUENCE = [
    # 0b IN4 IN3 IN2 IN1
    0b1000,  # Sequence step 1
    0b1001,  # Sequence step 2
    0b0001,  # Sequence step 3
    0b0101,  # Sequence step 4
    0b0100,  # Sequence step 5
    0b0110,  # Sequence step 6
    0b0010,  # Sequence step 7
    0b1010,  # Sequence step 8
]

IC1_I2CBC_SELECT = 0b1  # Control via register instead of input pins.
IC1_MODE_4_INPUTS = 0b0  # 0b0=4-input interface.
IC1_TRQ_SETTING = 0b1  # 0b1=50% torque mode.

# Registers
# Table 7-15. I2C Registers
# | Addr | Acronym     | Register Name                | 7     | 6         | 5     | 4      | 3    | 2     | 1     | 0     | Access |
# |------|-------------|------------------------------|-------|-----------|-------|--------|------|-------|-------|-------|--------|
# | 0x00 | SLAVE_ADDR  | Slave Address                | RSVD  | SLAVE_ADDR|       |        |      |       |       |       | RW     |
# | 0x01 | IC1_CON     | IC1 Control                  | TRQ   | IN4       | IN3   | IN2    | IN1  | I2CBC | MODE  |       | RW     |
# | 0x02 | IC2_CON     | IC2 Control                  | CLRFLT| DISFLT    | RSVD  | DECAY  | OCPR | OLDOD | OLDFD | OLDBO | RW     |
# | 0x03 | SLR_STATUS1 | Slew Rate and Fault Status-1 | RSVD  | SLR       | RSVD  | nFAULT | OCP  | OLD   | TSDF  | UVLOF | RW     |
# | 0x04 | STATUS2     | Fault Status-2               | OLD4  | OLD3      | OLD2  | OLD1   | OCP4 | OCP3  | OCP2  | OCP1  | R      |
#
# IC1_CON register (main one)
# Default: Mode = 0b00 means 4-input interface.
# Must set the input pin control.
# Must set I2CBC=0b1 (control via register instead of input pins).
# Should set TRQ=0b1 (50% torque) to minimize overheat.
#
# IC2_CON register:
# Could explore DECAY: 0b0=25% fast (default) or 0b1=100% slow.
# Could explore SLR (slew rate): 0b0=150ns, 0b1=300ns


def write_register(address: int, register: int, value: int) -> None:
    """Write a single byte to a register on the DRV8847S."""
    print(f"write_register({address=}, {register=}, {value=}) ...")

    data = bytearray([register, value])
    I2C_BUS.writeto(address, data)


def read_from_register(address: int, register: int) -> int:
    """Read a single byte from a register on the DRV8847S."""
    I2C_BUS.writeto(address, bytearray([register]))
    data = I2C_BUS.readfrom(address, 1)
    val = data[0]
    print(
        f"read_from_register({address=}, {register=}) = {val} = 0x{val:x} = 0b{val:b} ..."
    )
    return val


def send_drive_command(
    i2c_addr: int, *, trq: int, step_value: int, i2cbc: int, mode: int
) -> None:
    """Send a drive command to the DRV8847S."""
    ic1_val = (trq << 7) | (step_value << 3) | (i2cbc << 2) | (mode << 1)
    write_register(i2c_addr, REGISTER_ADDRESS_IC1_CONTROL, ic1_val)


def drive_motor(
    step_period_sec: float,
    step_count: int,
    step_seq: list[int] = HALF_STEP_SEQUENCE,
    i2c_addr: int = DRV8847S_DEFAULT_I2C_ADDR,
) -> None:
    """Drive the motor."""
    # TODO: Support reverse direction.
    done_step_count = 0

    step_seq_run = step_seq[::-1] if step_count < 0 else step_seq
    abs_step_count = abs(step_count)

    while 1:
        for step_value in step_seq_run:
            # Enable TRQ = 50% torque mode (heat).
            send_drive_command(
                i2c_addr=i2c_addr,
                trq=IC1_TRQ_SETTING,
                step_value=step_value,
                i2cbc=IC1_I2CBC_SELECT,
                mode=IC1_MODE_4_INPUTS,
            )
            time.sleep(step_period_sec)
            done_step_count += 1

            if done_step_count >= abs_step_count:
                return


def disable_motor(i2c_addr: int = DRV8847S_DEFAULT_I2C_ADDR) -> None:
    """Disable the motor."""
    # Disable the motor.
    send_drive_command(
        i2c_addr=i2c_addr,
        trq=IC1_TRQ_SETTING,
        step_value=0,  # <- Set IN1=IN2 and IN3=IN4 to stop the motor.
        i2cbc=IC1_I2CBC_SELECT,
        mode=IC1_MODE_4_INPUTS,
    )


def i2c_scan():
    print("Scanning I2C buses")
    i2c_scan_result = I2C_BUS.scan()
    print(f"I2C scan result ({len(i2c_scan_result)} addresses): {i2c_scan_result}")


def init_shift_register() -> None:
    """Initialize shift register pins to default states."""
    # Clear shift register.
    PIN_SHIFT_N_SRCLR.value(0)
    time.sleep_ms(SHIFT_REGISTER_INIT_SLEEP_MS)
    PIN_SHIFT_N_SRCLR.value(1)  # Active low, so set to normal (not clearing).
    time.sleep_ms(SHIFT_REGISTER_INIT_SLEEP_MS)

    PIN_SHIFT_N_OE.value(0)  # Active low, set low to enable outputs
    PIN_SHIFT_SRCK.value(0)  # Clock starts low
    PIN_SHIFT_RCLK.value(0)  # Latch starts low

    # Wait for initialization to complete
    time.sleep_ms(SHIFT_REGISTER_INIT_SLEEP_MS)

    # Clear all outputs.
    set_shift_registers([False] * 16)
    time.sleep_ms(SHIFT_REGISTER_INIT_SLEEP_MS)


def set_shift_registers(data: list[bool]) -> None:
    """Set the state of all shift registers based on input data.

    Args:
        data: list of 16 boolean values representing desired output states
             (6 registers x 8 bits per register)

    """
    if len(data) != 16:
        msg = "Data must contain exactly 16 boolean values"
        raise ValueError(msg)

    # Shift out all 16 bits
    for bit in reversed(data):  # Shift MSB first
        # Set data bit.
        PIN_SHIFT_SER_IN.value(1 if bit else 0)
        time.sleep_us(SHIFT_REGISTER_SLEEP_US)

        # Clock in the bit.
        PIN_SHIFT_SRCK.value(1)
        time.sleep_us(SHIFT_REGISTER_SLEEP_US)
        PIN_SHIFT_SRCK.value(0)
        time.sleep_us(SHIFT_REGISTER_SLEEP_US)

    # Latch the data to outputs
    PIN_SHIFT_RCLK.value(1)
    time.sleep_us(SHIFT_REGISTER_SLEEP_US)
    PIN_SHIFT_RCLK.value(0)
    time.sleep_us(SHIFT_REGISTER_SLEEP_US)


def set_shift_register_activate_stepper(stepper_num: int) -> None:
    """Activate the stepper motor by setting the shift register bits.

    Stepper numbers start at 0.
    """
    if stepper_num < 0 or stepper_num > 7:
        msg = "Stepper number must be 0 <= stepper_num <= 7."
        raise ValueError(msg)

    reg = [False] * 16
    reg[stepper_num] = True
    set_shift_registers(reg)


def set_corner_motor_state(corner_num: int, state: int) -> None:
    """Activate the corner motor by setting the shift register bits.

    Corner numbers start at 0.
    """
    if corner_num < 0 or corner_num > 3:
        msg = "Corner number must be 0 <= corner_num <= 3."
        raise ValueError(msg)

    if state not in (-1, 0, 1):
        raise ValueError("Invalid state value. Must be -1, 0, or 1.")

    reg = [False] * 16

    if state == 0:
        pass
    elif state == 1:
        reg[8 + corner_num * 2 + 0] = True
        reg[8 + corner_num * 2 + 1] = False
    elif state == -1:
        reg[8 + corner_num * 2 + 0] = False
        reg[8 + corner_num * 2 + 1] = True
    else:
        raise ValueError("Invalid state value. Must be -1, 0, or 1.")

    set_shift_registers(reg)


def pulse_corner_motor(
    corner_num: int,
    direction: int,
    pulse_ms: int = 5,
    settle_ms: int = 10,
) -> None:
    """Drive the motor for a short pulse in the given direction, then stop."""
    set_corner_motor_state(corner_num, direction)
    time.sleep_ms(pulse_ms)
    set_corner_motor_state(corner_num, 0)
    time.sleep_ms(settle_ms)


def drive_all_corner_motors(direction: int = 1, duration_ms: int = 120) -> None:
    """Spin each corner motor about a half-turn in a direction.

    Order: NW (0), NE (1), SW (2), SE (3).
    """

    reg = [False] * 16

    for corner_num in range(4):
        if direction == 0:
            pass
        elif direction == 1:
            reg[8 + corner_num * 2 + 0] = True
            reg[8 + corner_num * 2 + 1] = False
        elif direction == -1:
            reg[8 + corner_num * 2 + 0] = False
            reg[8 + corner_num * 2 + 1] = True
        else:
            raise ValueError("Invalid direction value. Must be -1, 0, or 1.")

    set_shift_registers(reg)
    
    if direction != 0:
        time.sleep_ms(duration_ms)

    set_shift_registers([False] * 16)

def init_hall_sensor() -> None:
    """Initialize the hall sensor pins to default states (select channel 15)."""
    PIN_HALL_SELECT_0.value(1)
    PIN_HALL_SELECT_1.value(1)
    PIN_HALL_SELECT_2.value(1)
    PIN_HALL_SELECT_3.value(1)


def read_hall_sensor_u16(sensor_num: int) -> int:
    """Read the hall sensor and return a 16-bit value."""
    # Select the correct hall sensor (using the mux).
    PIN_HALL_SELECT_0.value(sensor_num & 0b0001)
    PIN_HALL_SELECT_1.value(sensor_num & 0b0010)
    PIN_HALL_SELECT_2.value(sensor_num & 0b0100)
    PIN_HALL_SELECT_3.value(sensor_num & 0b1000)

    time.sleep_ms(10)  # Wait for the sensor to settle.

    # Read the hall sensor.
    val = HALL_ADC.read_u16()

    # Reset the mux to default state.
    init_hall_sensor()

    return val


def read_hall_sensor_abs_u16(sensor_num: int) -> int:
    """Read the hall sensor and return the absolute 16-bit value."""
    val = read_hall_sensor_u16(sensor_num)
    return abs((1 << 15) - val)


def read_corner_hall_sensor_abs_u16(corner_num: int) -> int:
    """Read the hall sensor for a specific corner and return the absolute 16-bit value."""
    if corner_num < 0 or corner_num > 3:
        msg = "Corner number must be 0 <= corner_num <= 3."
        raise ValueError(msg)

    return read_hall_sensor_abs_u16(corner_num + 8)


def hall_sensor_self_check() -> None:
    """Check that the hall sensors are working.

    Expects that Input 14 is connected to a 50% voltage divider,
    and that Input 15 is pulled down to GND.
    """
    val_14 = read_hall_sensor_u16(14)
    print(f"Hall sensor 14: {val_14:,}")
    if (val_14 < (1 << 16) * 0.48) or (val_14 > (1 << 16) * 0.52):
        print("❌ Hall sensor 14 self-check failed.")
    else:
        print("✅ Hall sensor 14 self-check passed.")

    val_15 = read_hall_sensor_u16(15)
    print(f"Hall sensor 15: {val_15:,}")
    if val_15 > (1 << 15) * 0.1:
        print("❌ Hall sensor 15 self-check failed.")
    else:
        print("✅ Hall sensor 15 self-check passed.")

    # Reset the mux to default state.
    init_hall_sensor()


def demo_read_each_hall_sensor() -> None:
    """Read each hall sensor and print the value."""
    for i in range(16):
        val = read_hall_sensor_u16(i)
        print(f"Hall sensor {i}: {val:,}")


def demo_gp_leds() -> None:
    """Demo the general purpose LEDs."""
    for i in range(2):
        PIN_GP_LED_0.value(0)
        PIN_GP_LED_1.value(1)
        time.sleep_ms(100)
        PIN_GP_LED_0.value(1)
        PIN_GP_LED_1.value(0)
        time.sleep_ms(100)

    PIN_GP_LED_0.value(0)
    PIN_GP_LED_1.value(0)


def demo_zeroing_corner_motor(corner_number: int) -> None:
    """Try zeroing the top-left (U10 [corner 0] + U25 [hall 8]) corner motor.

    Note that the large cam lobe is aligned with the magnet. Thus, when zeroed, the cam
    presses "into" the PCB.
    """
    assert 0 <= corner_number <= 3
    shift_register_motor_number = corner_number * 2 + 8
    hall_sensor_number = corner_number + 8

    reg_off = [False] * 16

    reg_cw = [False] * 16
    reg_cw[shift_register_motor_number + 0] = True
    reg_cw[shift_register_motor_number + 1] = False

    reg_ccw = [False] * 16
    reg_ccw[shift_register_motor_number + 0] = False
    reg_ccw[shift_register_motor_number + 1] = True

    for i in range(100):
        # Set the motor to spin in one direction.
        set_shift_registers(reg_cw)
        time.sleep_ms(10)
        set_shift_registers(reg_off)

        # Read the hall sensor.
        val = read_hall_sensor_u16(hall_sensor_number)
        print(f"Hall sensor: {val:,}")

        time.sleep(1)


def is_at_peak(measurements: list[int], min_sample_count: int) -> bool:
    """Check if the current measurement is a peak."""
    if len(measurements) < min_sample_count:
        return False

    middle_val = measurements[len(measurements) // 2]
    return all(middle_val >= m for m in measurements)


def zero_corner_motors(
    corner_numbers: list[int] | None = None,
    timeout_per_motor_ms: int = 2000,
    min_valid_value: int = 1000,
    min_sample_count: int = 9,  # Best if it's odd.
) -> None:
    """Zero the specified corner motors.

    Note that the large cam lobe is aligned with the magnet. Thus, when zeroed, the cam
    presses "into" the PCB and the max abs hall sensor value should be minimized.

    For each corner motor (0..3):
      - Find a local maximum of the hall sensor value using greedy ascent.
      - Stop the motor positioned at that local maximum.
      - Timeout after 3 seconds per motor.
    """
    if corner_numbers is None:
        corner_numbers = [0, 1, 2, 3]

    print(f"Zeroing corner motors: {corner_numbers}")

    failed_corners: list[int] = []

    for corner_num in corner_numbers:
        start = time.ticks_ms()

        # Stop the motor.
        set_corner_motor_state(corner_num, 0)

        last_n_measurements: list[int] = []
        while True:
            # Check for timeout.
            if time.ticks_ms() - start > timeout_per_motor_ms:
                set_corner_motor_state(corner_num, 0)
                failed_corners.append(corner_num)
                print(f"Failed to zero corner motor {corner_num}.")
                break

            # Step to new position.
            pulse_corner_motor(corner_num, 1)

            # Read the hall sensor value.
            val = read_corner_hall_sensor_abs_u16(corner_num=corner_num)
            print(f"Corner {corner_num} - abs hall sensor: {val:,}")

            # Skip "considering" the measurement if it's not valid.
            # For example, if a magnet fell off a shaft, this value will be useless.
            if val < min_valid_value:
                continue

            # Update the list of last N measurements.
            last_n_measurements.append(val)
            if len(last_n_measurements) > min_sample_count:
                last_n_measurements.pop(0)

            if is_at_peak(last_n_measurements, min_sample_count):
                print(f"Peak detected for corner {corner_num}! Halting.")
                for _ in range(min_sample_count // 2):
                    pulse_corner_motor(corner_num, -1)  # Step motor back.
                break

    if failed_corners:
        raise RuntimeError(f"Timeout while zeroing corner motors: {failed_corners}")

    print("Zeroing complete.")


def demo_random_corner_positions(corner_numbers: list[int] | None = None) -> None:
    if corner_numbers is None:
        corner_numbers = [0, 1, 2, 3]

    for corner_num in corner_numbers:
        # Move the motor to a random position.
        random_position = random.randint(10, 1000)
        pulse_corner_motor(corner_num, 1, pulse_ms=random_position)


def demo_driving_stepper_motor() -> None:
    print("Starting demo_driving_stepper_motor()")

    drive_motor(step_period_sec=0.02, step_count=100)

    # Read the fault status register.
    fault_status_2 = read_from_register(
        DRV8847S_DEFAULT_I2C_ADDR, REGISTER_ADDRESS_FAULT_STATUS_2
    )
    if fault_status_2:
        print("Fault detected!")

    disable_motor()
    time.sleep(2)


def get_all_functions() -> list[str]:
    function_type = type(lambda: None)

    return list(
        sorted(
            n
            for n, o in globals().items()
            if isinstance(o, function_type) and not n.startswith("_")
        )
    )


def print_available_commands() -> None:
    print("""
Available commands:
    - help()

    - <just a single period>
        -> Repeat the last command.
    """)

    print("All functions:")
    for func in get_all_functions():
        print(f"    - {func}(...)")


def help() -> None:
    print_available_commands()


class GlobalStoreSingleton:
    def __init__(self):
        self.last_command = "help"


global_store = GlobalStoreSingleton()


def prompt_and_execute() -> None:
    print("Enter a command, or use 'help':")
    command = input(">> ").strip()

    if command == ".":
        print("Repeating last command.")
        command = global_store.last_command
    else:
        command = command.strip()
        global_store.last_command = command  # Store for repeat feature.

    # If the command does not have parentheses, add them.
    if "(" not in command and ")" not in command:
        command += "()"

    print(f"Executing command: {command}\n")

    try:
        exec(command)
    except Exception as e:
        print(f"Error: {e}")
    print()


def main() -> None:
    print("Starting init.")
    init_shift_register()
    PIN_GP_LED_0.value(0)
    PIN_GP_LED_1.value(0)

    init_hall_sensor()
    print("Init complete.")

    demo_gp_leds()

    i2c_scan()
    # TODO: Do the I2C address reprogramming here.
    set_shift_register_activate_stepper(0)
    i2c_scan()

    hall_sensor_self_check()

    while True:
        prompt_and_execute()


while True:
    main()
