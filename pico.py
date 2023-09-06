import time
import board
import busio
import adafruit_lsm9ds1

# Create sensor object, communicating over the board's default I2C bus
i2c = busio.I2C(board.GP15, board.GP14)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

# Main loop will read the acceleration, magnetometer, gyroscope, Temperature
# values every second and print them out.
while True:
    # Read acceleration, magnetometer, gyroscope, temperature.
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro

    # Print values (only prints accel data currently)
    print("{0:0.3f},{1:0.3f},{2:0.3f}".format(accel_x, accel_y, accel_z))
    
    # Delay for a second.
    time.sleep(0.1)


#,{0:0.3f},{1:0.3f},{2:0.3f},{0:0.3f},{1:0.3f},{2:0.3f}
#, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z))