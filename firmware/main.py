from machine import ADC, Pin
import time

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

PIN_SW1 = Pin(28, Pin.IN, Pin.PULL_UP)
PIN_SW2 = Pin(27, Pin.IN, Pin.PULL_UP)
PIN_GP_LED_0 = Pin(7, Pin.OUT)
PIN_GP_LED_1 = Pin(8, Pin.OUT)

PIN_HALL_SELECT_0 = Pin(9, Pin.OUT)
PIN_HALL_SELECT_1 = Pin(10, Pin.OUT)
PIN_HALL_SELECT_2 = Pin(11, Pin.OUT)
PIN_HALL_SELECT_3 = Pin(12, Pin.OUT)
PINT_HALL_ADC = Pin(26, Pin.IN)

HALL_ADC = ADC(PINT_HALL_ADC)


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
    """
    Set the state of all shift registers based on input data.

    Args:
        data: list of 16 boolean values representing desired output states
             (6 registers x 8 bits per register)
    """
    if len(data) != 16:
        raise ValueError("Data must contain exactly 16 boolean values")

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

def demo_each_corner_motor() -> None:
    """Spin each corner motor 1 sec in each direction."""

    DURATION_EACH_DIRECTION_MS = 200

    for motor_num in range(4):
        # Set the motor to spin in one direction.
        reg = [False] * 16
        reg[8 + motor_num * 2 + 0] = True
        reg[8 + motor_num * 2 + 1] = False
        set_shift_registers(reg)
        time.sleep_ms(DURATION_EACH_DIRECTION_MS)

        # Set the motor to spin in the other direction.
        reg = [False] * 16
        reg[8 + motor_num * 2 + 0] = False
        reg[8 + motor_num * 2 + 1] = True
        set_shift_registers(reg)
        time.sleep_ms(DURATION_EACH_DIRECTION_MS)

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

    time.sleep_ms(10) # Wait for the sensor to settle.

    # Read the hall sensor.
    val = HALL_ADC.read_u16()

    # Reset the mux to default state.
    init_hall_sensor()

    return val

def demo_read_each_hall_sensor() -> None:
    """Read each hall sensor and print the value."""
    for i in range(16):
        val = read_hall_sensor_u16(i)
        print(f"Hall sensor {i}: {val:,}")

def demo_zeroing_corner_motor() -> None:
    """Try zeroing the top-left (U10 [corner 0] + U25 [hall 8]) corner motor."""
    reg_off = [False] * 16

    reg_cw = [False] * 16
    reg_cw[8 + 0] = True
    reg_cw[8 + 1] = False

    reg_ccw = [False] * 16
    reg_ccw[8 + 0] = False
    reg_ccw[8 + 1] = True

    for i in range(100):
        # Set the motor to spin in one direction.
        set_shift_registers(reg_cw)
        time.sleep_ms(10)
        set_shift_registers(reg_off)

        # Read the hall sensor.
        val = read_hall_sensor_u16(8)
        print(f"Hall sensor: {val:,}")

        time.sleep(1)


def main() -> None:
    print("Starting init.")
    init_shift_register()
    PIN_GP_LED_0.value(0)
    PIN_GP_LED_1.value(0)

    init_hall_sensor()
    print("Init complete.")

    print("Starting demo_zeroing_corner_motor()")
    demo_zeroing_corner_motor()
    print("Done demo_zeroing_corner_motor()")

    while 1:
        print("Starting demo_each_corner_motor()")
        demo_each_corner_motor()
        print("Done demo_each_corner_motor()")

        print("Starting demo_read_each_hall_sensor()")
        demo_read_each_hall_sensor()
        print("Done demo_read_each_hall_sensor()")

    # while 1:
    #     print("Starting rolling sphere demo.")
    #     braille_demo_try_to_roll_sphere()


while True:
    main()
