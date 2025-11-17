# ASM330LHH Gyroscope with Madgwick Filter
# ASM330LHH Gyroscope with Madgwick Filter

import sensor
import time
import os
from pyb import Pin, SPI, Timer


cs = Pin("P3", Pin.OUT_OD)
spi = SPI(2, SPI.MASTER, baudrate=10000000, polarity=1, phase=1)

### Defines from Mathias given example
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
GYRO_SAMPLERATE = SAMPLERATE_208HZ
XL_SAMPLERATE	= SAMPLERATE_208HZ

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



ACCEL_FLAG = False
GYRO_FLAG  = False
TIME_FLAG  = False
TAKE_IMAGE = False
DESIRED_NUM_OF_IMG = 300

data_file_name  = "/data.csv"
image_file_name = "/image.bin"


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

def init_sensor():
    write_reg(REG_CTRL3_C,      0x01)
    time.sleep_ms(100)
    write_reg(REG_CTRL4_C,      0x06)
    write_reg(REG_CTRL6_C,      0x05)
    write_reg(REG_FIFO_CTRL3,   ((GYRO_SAMPLERATE << 4) | XL_SAMPLERATE))
    write_reg(REG_CTRL1_XL,     (XL_SAMPLERATE  << 4))
    write_reg(REG_CTRL2_G,      ((GYRO_SAMPLERATE << 4) | (0x1 << 1)))
    write_reg(REG_CTRL10_C,     0x20)
    write_reg(REG_FIFO_CTRL4,   0x56)
    write_reg(REG_CTRL9_XL,     0x02)

def send_and_empty_buffer(buf, data_file):
    for i in buf:
        data_file.write(i)
    data_file.flush()   #Make sure all data get written
    buf.clear()         #Empty data buffer

def get_offset_time():
    time_start = time.ticks_us() / 1000000;
    timestamp0 = read_reg(REG_TIMESTAMP0_REG)
    timestamp1 = read_reg(REG_TIMESTAMP1_REG)
    timestamp2 = read_reg(REG_TIMESTAMP2_REG)
    timestamp3 = read_reg(REG_TIMESTAMP3_REG)
    time_end = time.ticks_us() / 1000000;

    imu_ticks = (timestamp3[0] << 24) | (timestamp2[0] << 16) | (timestamp1[0] << 8) | timestamp0[0]
    imu_timestamp = imu_ticks * 25 / 1000000
    time_offset = imu_timestamp - (time_start - (time_end - time_start) / 2)

    return time_offset

# --- Timer/Counter Logic ---
def tick(timer):
    global TAKE_IMAGE
    TAKE_IMAGE = True

# Timer from pyb
tim = Timer(4, freq=10)
tim.callback(tick)


# --- Main program ---
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.WVGA2)
sensor.skip_frames(time=2000)

init_sensor()

data_buf = []
image_count = 0
data_count = 0

time_offset = get_offset_time()
print(f"IMU offset time: {time_offset}s")


try:
    data_file = open(data_file_name, 'w')
    data_file.write("Timestamp, Data_type, Gyro x, Gyro y, Gyro z, Accel x, Accel y, Accel z, Filter yaw, Filter pitch, Filter roll, Image_name \n")
    image_file = open(image_file_name, 'wb') #wb = write binary
    print("Data files on sd card has been made")
except Exception as e:
    print(f"Error when creating data and image file: {e}")



time_start = time.ticks_us() / 1000000
time_offset += time_start
print("Data harvest started!")

while DESIRED_NUM_OF_IMG > image_count:
    if TAKE_IMAGE:
        timestamp = time.ticks_us() / 1000000 - time_start
        img = sensor.snapshot()
        image_file.write(img.bytearray())
        data_buf.append(f"{timestamp}, IMG_DATA, , , , , , , , , , Image_{image_count:05d} \n")
        image_count += 1
        data_count += 1
        TAKE_IMAGE = False


    fifo_status = read_reg(REG_FIFO_STATUS1, 2)
    fifo_count = fifo_status[0] | ((fifo_status[1] & 0x03) << 8)

    if fifo_count > 0:
        fifo_data = read_reg(REG_FIFO_DATA_OUT_TAG, fifo_count * 7)

        for i in range(0, len(fifo_data), 7):
            tag = (fifo_data[i] >> 3) & 0x1F

            x = fifo_data[i+1] | (fifo_data[i+2] << 8)
            y = fifo_data[i+3] | (fifo_data[i+4] << 8)
            z = fifo_data[i+5] | (fifo_data[i+6] << 8)

            if x > 32767: x -= 65536
            if y > 32767: y -= 65536
            if z > 32767: z -= 65536

            if tag == 0x01:
                gyro_data = [x * 0.004375, y * 0.004375, z * 0.004375]
                GYRO_FLAG = True
            elif tag == 0x02:
                accel_data = [x, y, z]
                ACCEL_FLAG = True
            elif tag == 0x04:
                imu_ticks = (fifo_data[i+4] << 24) | (fifo_data[i+3] << 16) | (fifo_data[i+2] << 8) | fifo_data[i+1]
                imu_timestamp = imu_ticks * 25 / 1000000
                timestamp = imu_timestamp - time_offset
                TIME_FLAG = True

            if (GYRO_FLAG and ACCEL_FLAG and TIME_FLAG):
                data_row = f"{timestamp}, IMU_DATA, {gyro_data[0]}, {gyro_data[1]}, {gyro_data[2]}, {accel_data[0]}, {accel_data[1]}, {accel_data[2]}, , , , , \n"
                data_buf.append(data_row)
                data_count += 1

                GYRO_FLAG = False
                ACCEL_FLAG = False
                TIME_FLAG = False

            if data_count % 50 == 0:
                send_and_empty_buffer(data_buf, data_file)

print("Data harvest has ended!")

send_and_empty_buffer(data_buf, data_file)
data_file.close()
image_file.close()
time.sleep(0.5)

print("Finished")

