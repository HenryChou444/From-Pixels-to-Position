# ASM330LHH Gyroscope with Madgwick Filter
# ASM330LHH Gyroscope with Madgwick Filter

import sensor
import time
import math
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

DESIRED_NUM_OF_IMG = 150


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
    write_reg(REG_FIFO_CTRL3, 0x99)
    write_reg(REG_CTRL1_XL, 0x90)
    write_reg(REG_CTRL2_G, 0x92)
    write_reg(REG_CTRL10_C, 0x20)
    write_reg(REG_FIFO_CTRL4, 0x56)
    write_reg(REG_CTRL9_XL, 0x02)

class Madgwick:
    def __init__(self):
        self.q = [1.0, 0.0, 0.0, 0.0]
        self.beta = 0.1

    def update(self, gx, gy, gz, ax, ay, az, dt):
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0: return
        ax, ay, az = ax/norm, ay/norm, az/norm

        q1, q2, q3, q4 = self.q

        s1 = 2*q3*ax + 2*q4*ay - 2*q2*az
        s2 = 2*q2*ax + 2*q4*az - 2*q3*ay
        s3 = 2*q1*ax - 2*q2*ay + 2*q4*az
        s4 = 2*q2*ax + 2*q3*ay - 2*q1*az

        norm = math.sqrt(s1*s1 + s2*s2 + s3*s3 + s4*s4)
        if norm != 0:
            s1, s2, s3, s4 = s1/norm, s2/norm, s3/norm, s4/norm

        q1 += (0.5*(-q2*gx - q3*gy - q4*gz) - self.beta*s1) * dt
        q2 += (0.5*(q1*gx + q3*gz - q4*gy) - self.beta*s2) * dt
        q3 += (0.5*(q1*gy - q2*gz + q4*gx) - self.beta*s3) * dt
        q4 += (0.5*(q1*gz + q2*gy - q3*gx) - self.beta*s4) * dt

        norm = math.sqrt(q1*q1 + q2*q2 + q3*q3 + q4*q4)
        self.q = [q1/norm, q2/norm, q3/norm, q4/norm]

    def get_angles(self):
        q1, q2, q3, q4 = self.q
        roll = math.atan2(2*(q1*q2 + q3*q4), 1 - 2*(q2*q2 + q3*q3)) * 57.3
        pitch = math.asin(2*(q1*q3 - q4*q2)) * 57.3
        yaw = math.atan2(2*(q1*q4 + q2*q3), 1 - 2*(q3*q3 + q4*q4)) * 57.3
        return roll, pitch, yaw

def send_and_empty_buffer(buf, data_file):
    for i in buf:
        data_file.write(i)
    data_file.flush()   #Make sure all data get written
    buf.clear()         #Empty data buffer

def calibrate_gyro(gyro_offset):
    counter = 0
    gyro_data = [0,0,0]

    while(counter < 500):
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
                    gyro_data += [x, y, z]
                    counter += 1             
        time.sleep_ms(5)
    gyro_offset = gyro_data / counter

def sync_timestamp():
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

def create_data_files(data_name, image_name):
    global data_file
    global image_file

    data_file_name = f"/{data_name}.csv"
    image_file_name = f"/{image_name}.bin"
    try:
        data_file = open(data_file_name, 'w')
        data_file.write("Timestamp, Data_type, Gyro x, Gyro y, Gyro z, Accel x, Accel y, Accel z, Filter yaw, Filter pitch, Filter roll, Image_name \n")
        image_file = open(image_file_name, 'wb') #wb = write binary
        print("Data files on sd card has been made")

    except Exception as e:
        print(f"Error when creating data and image file: {e}")

def take_and_save_image(img_file, data_buf, img_count, data_count):
        global time_start

        timestamp = time.ticks_us() / 1000000 - time_start
        img = sensor.snapshot()
        img_file.write(img.bytearray())
        data_buf.append(f"{timestamp}, IMG_DATA, , , , , , , , , , Image_{img_count:05d} \n")
        img_count += 1
        data_count += 1

# --- Timer/Counter Logic ---
def tick(timer):
    global TAKE_IMAGE
    TAKE_IMAGE = True

# Timer from pyb
tim = Timer(4, freq=10)
tim.callback(tick)


MOVEMENT_THRESHOLD = 2.0
last_angles = [0.0, 0.0, 0.0]
gyro_offset = [0,0,0]
time_offset = 0
data_buf = []
image_count = 0
data_count = 0


# --- Main program ---
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.WVGA2)
sensor.skip_frames(time=2000)

init_sensor()
madgwick = Madgwick()
last_time = time.ticks_ms()
sync_timestamp(time_offset)
calibrate_gyro(gyro_offset)
create_data_files("data", "image")

time_start = time.ticks_us() / 1000000
time_offset += time_start

print("Data harvest started!")
while DESIRED_NUM_OF_IMG > image_count:
    if TAKE_IMAGE:
        take_and_save_image(image_file, data_buf, image_count, data_count)
        TAKE_IMAGE = True

    fifo_status = read_reg(REG_FIFO_STATUS1, 2)
    if(fifo_status[1] & 0x40):
        print("!!!! FIFO Overflow detected !!!!")
        
    fifo_count = fifo_status[0] | ((fifo_status[1] & 0x03) << 8)

    if fifo_count > 0:
        fifo_data = read_reg(REG_FIFO_DATA_OUT_TAG, fifo_count * 7)
        
        # Process each sample in the FIFO
        gyro_data = None
        accel_data = None

        for i in range(0, len(fifo_data), 7):
            tag = (fifo_data[i] >> 3) & 0x1F

            x = fifo_data[i+1] | (fifo_data[i+2] << 8)
            y = fifo_data[i+3] | (fifo_data[i+4] << 8)
            z = fifo_data[i+5] | (fifo_data[i+6] << 8)

            if x > 32767: x -= 65536
            if y > 32767: y -= 65536
            if z > 32767: z -= 65536

            if tag == 0x01:
                gyro_data = [(x - gyro_offset[0]) * 0.004375, (y - gyro_offset[1]) * 0.004375, (z - gyro_offset[2]) * 0.004375]
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

            # Update filter when we have both gyro and accel data
            if gyro_data is not None and accel_data is not None:
                current_time = time.ticks_ms()
                dt = time.ticks_diff(current_time, last_time) / 1000.0
                last_time = current_time
            # Update filter when we have both gyro and accel data
            if gyro_data is not None and accel_data is not None:
                current_time = time.ticks_ms()
                dt = time.ticks_diff(current_time, last_time) / 1000.0
                last_time = current_time

                total_gyro = abs(gyro_data[0]) + abs(gyro_data[1]) + abs(gyro_data[2])
                total_gyro = abs(gyro_data[0]) + abs(gyro_data[1]) + abs(gyro_data[2])

                if total_gyro > MOVEMENT_THRESHOLD:
                    gx = gyro_data[0] * 0.01745
                    gy = gyro_data[1] * 0.01745
                    gz = gyro_data[2] * 0.01745
                if total_gyro > MOVEMENT_THRESHOLD:
                    gx = gyro_data[0] * 0.01745
                    gy = gyro_data[1] * 0.01745
                    gz = gyro_data[2] * 0.01745

                    madgwick.update(gx, gy, gz, accel_data[0], accel_data[1], accel_data[2], dt)
                    roll, pitch, yaw = madgwick.get_angles()
                    madgwick.update(gx, gy, gz, accel_data[0], accel_data[1], accel_data[2], dt)
                    roll, pitch, yaw = madgwick.get_angles()

                    angle_change = abs(roll - last_angles[0]) + abs(pitch - last_angles[1]) + abs(yaw - last_angles[2])
                    if angle_change > 0.5:
                        #print("Roll: %.1f  Pitch: %.1f  Yaw: %.1f" % (roll, pitch, yaw))
                        last_angles = [roll, pitch, yaw]
                
                # Reset after processing this pair
                gyro_data = None
                accel_data = None

print("Data harvest has ended!")

send_and_empty_buffer(data_buf, data_file)
data_file.close()
image_file.close()
os.sync()
time.sleep(0.5)
os.umount("/")
time.sleep(0.5)

print("Finished")


