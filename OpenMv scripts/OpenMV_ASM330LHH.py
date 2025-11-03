# ASM330LHH - By: mbenn - Sun Nov 2 2025

import sensor
import time
from pyb import Pin, SPI

cs = Pin("P3", Pin.OUT_OD)
int1 = Pin("P4", Pin.IN)

spi = SPI(2, SPI.MASTER, baudrate=10000000, polarity=1, phase=1)

SAMPLERATE_OFF	  = 0x0
SAMPLERATE_12_5HZ = 0x1
SAMPLERATE_26HZ	  = 0x2
SAMPLERATE_52HZ	  = 0x3
SAMPLERATE_104HZ  = 0x4
SAMPLERATE_208HZ  = 0x5
SAMPLERATE_417HZ  = 0x6
SAMPLERATE_833HZ  = 0x7
SAMPLERATE_1667HZ = 0x8
SAMPLERATE_3333HZ = 0x9
SAMPLERATE_6667HZ = 0xA

# Set desired sampling frequency
GYRO_SAMPLERATE = SAMPLERATE_3333HZ #SAMPLERATE_833HZ
XL_SAMPLERATE	= SAMPLERATE_3333HZ #SAMPLERATE_833HZ

# ASM330LHH Registers
CMD_READ				= 0x80	# MSB is set for read actions
REG_PIN_CTRL 			= 0x02
REG_FIFO_CTRL1 			= 0x07
REG_FIFO_CTRL2 			= 0x08
REG_FIFO_CTRL3 			= 0x09
REG_FIFO_CTRL4 			= 0x0A
REG_COUNTER_BDR_REG1 	= 0x0B
REG_COUNTER_BDR_REG2 	= 0x0C
REG_INT1_CTRL 			= 0x0D
REG_INT2_CTRL 			= 0x0E
REG_WHO_AM_I 			= 0x0F
REG_CTRL1_XL 			= 0x10
REG_CTRL2_G 			= 0x11
REG_CTRL3_C 			= 0x12
REG_CTRL4_C 			= 0x13
REG_CTRL5_C 			= 0x14
REG_CTRL6_C 			= 0x15
REG_CTRL7_G 			= 0x16
REG_CTRL8_XL 			= 0x17
REG_CTRL9_XL 			= 0x18
REG_CTRL10_C 			= 0x19
REG_ALL_INT_SRC 		= 0x1A
REG_WAKE_UP_SRC 		= 0x1B
REG_D6D_SRC 			= 0x1D
REG_STATUS_REG 			= 0x1E
REG_OUT_TEMP_L 			= 0x20
REG_OUT_TEMP_H 			= 0x21
REG_OUTX_L_G 			= 0x22
REG_OUTX_H_G 			= 0x23
REG_OUTY_L_G 			= 0x24
REG_OUTY_H_G 			= 0x25
REG_OUTZ_L_G 			= 0x26
REG_OUTZ_H_G 			= 0x27
REG_OUTX_L_A 			= 0x28
REG_OUTX_H_A 			= 0x29
REG_OUTY_L_A 			= 0x2A
REG_OUTY_H_A 			= 0x2B
REG_OUTZ_L_A 			= 0x2C
REG_OUTZ_H_A 			= 0x2D
REG_FIFO_STATUS1 		= 0x3A
REG_FIFO_STATUS2 		= 0x3B
REG_TIMESTAMP0_REG 		= 0x40
REG_TIMESTAMP1_REG 		= 0x41
REG_TIMESTAMP2_REG 		= 0x42
REG_TIMESTAMP3_REG 		= 0x43
REG_INT_CFG0 			= 0x56
REG_INT_CFG1 			= 0x58
REG_THS_6D 				= 0x59
REG_WAKE_UP_THS 		= 0x5B
REG_WAKE_UP_DUR 		= 0x5C
REG_FREE_FALL 			= 0x5D
REG_MD1_CFG 			= 0x5E
REG_MD2_CFG 			= 0x5F
REG_INTERNAL_FREQ_FINE 	= 0x63
REG_X_OFS_USR 			= 0x73
REG_Y_OFS_USR 			= 0x74
REG_Z_OFS_USR 			= 0x75
REG_FIFO_DATA_OUT_TAG 	= 0x78
REG_FIFO_DATA_OUT_X_L 	= 0x79
REG_FIFO_DATA_OUT_X_H 	= 0x7A
REG_FIFO_DATA_OUT_Y_L 	= 0x7B
REG_FIFO_DATA_OUT_Y_H 	= 0x7C
REG_FIFO_DATA_OUT_Z_L 	= 0x7D
REG_FIFO_DATA_OUT_Z_H 	= 0x7E

TAG_GYRO = (0x01 << 3)
TAG_ACC  = (0x02 << 3)
TAG_TEMP = (0x03 << 3)
TAG_TIME = (0x04 << 3)
TAG_CFG  = (0x05 << 3)

def write_reg(reg, data):
    cs.low()
    spi.send(reg)
    spi.send(data)
    cs.high()

def read_reg(reg, nbytes=1):
    cs.low()
    spi.send(reg | CMD_READ)
    data = spi.read(nbytes)
    cs.high()
    return data

def initASM330LHH():
    # Start by resetting the device
    write_reg(REG_CTRL3_C, 0x01)
    time.sleep_ms(100);

    # Enable SPI, enable LPF1
    write_reg(REG_CTRL4_C, (0x04 | 0x02))

    # Set LPF1 to 49Hz
    write_reg(REG_CTRL6_C, 0x05)

    # Set storage rate to FIFO
    write_reg(REG_FIFO_CTRL3, ((GYRO_SAMPLERATE << 4) | XL_SAMPLERATE))

    # Set accelerometer sample rate
    write_reg(REG_CTRL1_XL, (XL_SAMPLERATE << 4))

    # Set gyro sample rate, 125dps scale
    write_reg(REG_CTRL2_G, ((GYRO_SAMPLERATE << 4) | (0x1 << 1)))

    # Enables timestamp counter
    write_reg(REG_CTRL10_C, (0x1 << 5))

    # Enables batching of timestamps, temperatures to a continuous FIFO
    write_reg(REG_FIFO_CTRL4, ((0x1 << 6) | (0x1 << 4) | 0x6))

    # Enables proper device configuration
    write_reg(REG_CTRL9_XL, 0x02)


sensor.reset()  # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE)  # must be this
sensor.set_framesize(sensor.WVGA2)  # must be this
sensor.skip_frames(time=2000)  # Let new settings take affect.
clock = time.clock()  # Tracks FPS.

initASM330LHH()
fifoOverflows = 0

while True:
    clock.tick()  # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot()  # Take a picture and return the image.

    print(clock.fps())  # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.

    buf = read_reg(REG_WHO_AM_I)
    for b in buf:
        print(b)

    # Get the number of bytes in the fifo
    fifoStat = read_reg(REG_FIFO_STATUS1, 2) # Read both status registers
    fifoCnt = fifoStat[0] | ((fifoStat[1] & 0x03) << 8)
    print(fifoCnt)
    if(fifoStat[1] & 0x40):
        fifoOverflows += 1

    # Read the FIFO
    fifoCnt *= 7    # Convert from samples to bytes. 1 tag + 6 bytes
    fifoData = read_reg(REG_FIFO_DATA_OUT_TAG, fifoCnt)

    # Handle the data from the FIFO
    # ...

