# Untitled - By: nikolajkristiansen - Mon Oct 27 2025

# Example that we have been using for SD card manegement, have been found here:
# https://github.com/openmv/openmv/blob/master/scripts/examples/02-Image-Processing/03-Frame-Differencing/on_disk_basic_frame_differencing.py

# Example for csv file that have been used in this project, have been found here:
# https://www.hackster.io/rajivcodelab/csv-data-logger-read-and-write-csv-files-with-pi-pico-w-1f14d7

import sensor
import os
import time
from pyb import millis
from utime import sleep_ms



FPS = 10
IMAGES = 10



def save_image(image, timestamp):
    global save_image_first_time

    try:
        if save_image_first_time:
            #Checking if folder exists
            if not "Image_data" in os.listdir("/"):
                os.mkdir("/Image_data")
            save_image_first_time = False

        #Save image onto SD_card
        full_path = f"/Image_data/{timestamp}.jpg"
        image.save(full_path)

        #Save image log into csv file
        log_data_to_csv(timestamp, "image", " ", " ", " ", " ", " ", " ", f"{timestamp}.jpg")

    except Exception as e:
        print(f"Error in 'save_image' function: {e}")

def log_data_to_csv(timestamp, data_type, a_x, a_y, a_z, g_x, g_y, g_z, image_name):
    global log_data_first_time

    try:
        data_filename = "data.csv"

        if log_data_first_time:
            #Checking if file exists
            if not data_filename in os.listdir("/"):
                # 'w' for write
                with open(f"/{data_filename}", 'w') as file:
                    file.write('timestamp, data_type, a_x, a_y, a_z, g_x, g_y, g_z, image_name\n')
            log_data_first_time = False

        #Data structure: timestamp, data type, a_x, a_y, a_z, g_x, g_y, g_z, image_name
        data_row = f"{timestamp}, {data_type}, {a_x}, {a_y}, {a_z}, {g_x}, {g_y}, {g_z}, {image_name}\n"

        #Add data into csv file - ('a' for append)
        with open(f"/{data_filename}", 'a') as file:
            file.write(data_row)
    except Exception as e:
        print(f"Error in 'log_data_to_csv' function: {e}")



# Initialize camera
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
clock = time.clock()

# Variables
starttime = millis()
image_taken = 0

save_image_first_time = True
log_data_first_time = True


# We need to use a timer and interrupt to make sure each image
# is taken every x-amount of seconds, defined by FPS

while (image_taken < IMAGES):
    try:
        # Wait 1sec
        sleep_ms(1000)
        # Take image
        timestamp = millis() - starttime
        image = sensor.snapshot()

        save_image(image, timestamp)

        print("image saved, and data logged")
        image_taken = image_taken + 1
        print(image_taken)



    except Exception as e:
        print(f"Error in main loop: {e}")
        break


print("unmount begin")
sleep_ms(1000)
os.umount("/")
print("program is finished")


