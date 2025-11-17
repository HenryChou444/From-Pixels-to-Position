# ASM330LHH Gyroscope with Madgwick Filter
# ASM330LHH Gyroscope with Madgwick Filter

import sensor
import time
import os
from pyb import Pin, SPI, Timer


cs = Pin("P3", Pin.OUT_OD)
spi = SPI(2, SPI.MASTER, baudrate=10000000, polarity=1, phase=1)

CMD_READ = 0x80
REG_CTRL3_C = 0x12
REG_CTRL4_C = 0x13
REG_CTRL6_C = 0x15
REG_CTRL1_XL = 0x10
REG_CTRL2_G = 0x11
REG_CTRL9_XL = 0x18
REG_CTRL10_C = 0x19
REG_FIFO_CTRL3 = 0x09
REG_FIFO_CTRL4 = 0x0A
REG_FIFO_STATUS1 = 0x3A
REG_FIFO_DATA_OUT_TAG = 0x78

REG_TIMESTAMP0_REG 		= 0x40
REG_TIMESTAMP1_REG 		= 0x41
REG_TIMESTAMP2_REG 		= 0x42
REG_TIMESTAMP3_REG 		= 0x43

ACCEL_FLAG = False
GYRO_FLAG  = False
TIME_FLAG  = False

TAKE_IMAGE = False

DESIRED_NUM_OF_IMG = 300


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


    SAMPLERATE_208HZ  = 0x5


    # Set desired sampling frequency
    GYRO_SAMPLERATE = SAMPLERATE_208HZ
    XL_SAMPLERATE	= SAMPLERATE_208HZ


    write_reg(REG_CTRL3_C, 0x01)
    time.sleep_ms(100)
    write_reg(REG_CTRL4_C, 0x06)
    time.sleep_ms(100)
    write_reg(REG_CTRL4_C, 0x06)
    write_reg(REG_CTRL6_C, 0x05)
    write_reg(REG_FIFO_CTRL3, 0x99)
    write_reg(REG_CTRL1_XL, 0x90)
    write_reg(REG_CTRL2_G, 0x92)
    write_reg(REG_CTRL10_C, 0x20)
    write_reg(REG_FIFO_CTRL4, 0x56)
    write_reg(REG_FIFO_CTRL3, ((GYRO_SAMPLERATE << 4) | XL_SAMPLERATE))
    write_reg(REG_CTRL1_XL, (XL_SAMPLERATE << 4))
    write_reg(REG_CTRL2_G, ((GYRO_SAMPLERATE << 4) | (0x1 << 1)))
    write_reg(REG_CTRL10_C, 0x20)
    write_reg(REG_FIFO_CTRL4, 0x56)
    write_reg(REG_CTRL9_XL, 0x02)

def send_and_empty_buffer(buf, data_file):
    for i in buf:
        data_file.write(i)
    data_file.flush()   #Make sure all data get written
    buf.clear()         #Empty data buffer


# --- Timer/Counter Logic ---
def tick(timer):
    global TAKE_IMAGE
    TAKE_IMAGE = True

# Timer from pyb
tim = Timer(4, freq=10)
#Freq is in Hz, can be changed to anything
# '4' is the timer ID, other available timers are : 2, 3, 4, 7 , 8, 12-17 (from docs)
tim.callback(tick)


# --- Main program ---
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.WVGA2)
sensor.skip_frames(time=2000)

init_sensor()
madgwick = Madgwick()
last_time = time.ticks_ms()
MOVEMENT_THRESHOLD = 2.0
last_angles = [0.0, 0.0, 0.0]
data_buf = []
image_count = 0
data_count = 0

time_start = time.ticks_us() / 1000000;
timestamp0 = read_reg(REG_TIMESTAMP0_REG)
timestamp1 = read_reg(REG_TIMESTAMP1_REG)
timestamp2 = read_reg(REG_TIMESTAMP2_REG)
timestamp3 = read_reg(REG_TIMESTAMP3_REG)
time_end = time.ticks_us() / 1000000;

imu_ticks = (timestamp3[0] << 24) | (timestamp2[0] << 16) | (timestamp1[0] << 8) | timestamp0[0]
imu_timestamp = imu_ticks * 25 / 1000000
time_offset = imu_timestamp - (time_start - (time_end - time_start) / 2)
print(f"IMU offset time: {time_offset}s")

data_file_name = "/data.csv"
image_file_name = "/image.bin"
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
                #data structure for CSV: Timestamp, Data_type, Gyro x, Gyro y, Gyro z, Accel x, Accel y, Accel z, Filter yaw, Filter pitch, Filter roll, Image_name
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
os.sync()
time.sleep(0.5)
os.umount("/")
time.sleep(0.5)

print("Finished")

