import math

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



### Part in the data loop
 # # Update filter when we have both gyro and accel data
            # if gyro_data is not None and accel_data is not None:
            #     current_time = time.ticks_ms()
            #     dt = time.ticks_diff(current_time, last_time) / 1000.0
            #     last_time = current_time
            # # Update filter when we have both gyro and accel data
            # if gyro_data is not None and accel_data is not None:
            #     current_time = time.ticks_ms()
            #     dt = time.ticks_diff(current_time, last_time) / 1000.0
            #     last_time = current_time

            #     total_gyro = abs(gyro_data[0]) + abs(gyro_data[1]) + abs(gyro_data[2])
            #     total_gyro = abs(gyro_data[0]) + abs(gyro_data[1]) + abs(gyro_data[2])

            #     if total_gyro > MOVEMENT_THRESHOLD:
            #         gx = gyro_data[0] * 0.01745
            #         gy = gyro_data[1] * 0.01745
            #         gz = gyro_data[2] * 0.01745
            #     if total_gyro > MOVEMENT_THRESHOLD:
            #         gx = gyro_data[0] * 0.01745
            #         gy = gyro_data[1] * 0.01745
            #         gz = gyro_data[2] * 0.01745

            #         madgwick.update(gx, gy, gz, accel_data[0], accel_data[1], accel_data[2], dt)
            #         roll, pitch, yaw = madgwick.get_angles()
            #         madgwick.update(gx, gy, gz, accel_data[0], accel_data[1], accel_data[2], dt)
            #         roll, pitch, yaw = madgwick.get_angles()

            #         angle_change = abs(roll - last_angles[0]) + abs(pitch - last_angles[1]) + abs(yaw - last_angles[2])
            #         if angle_change > 0.5:
            #             #print("Roll: %.1f  Pitch: %.1f  Yaw: %.1f" % (roll, pitch, yaw))
            #             last_angles = [roll, pitch, yaw]

            #     # Reset after processing this pair
            #     gyro_data = None
            #     accel_data = None
