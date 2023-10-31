import serial
import time
import collections
import numpy as np
import math
from ahrs.filters import QUEST
from ahrs.filters import Madgwick
from ahrs import Quaternion

class Config:

    start_time = time.time()
    sensor = 'COM4'
    samples_per_second = 10
    samples = 5
    max_samples = 5 * samples_per_second

def main():
    
    sensor = Sensor(Config.sensor)
    
    try:
        while True:
            try:
                sensor_data = sensor.read_data()
                if sensor_data:
                    accel_state = sensor_data.accel_state()
                    mag_state = sensor_data.mag_state()
                    gyro_state = sensor_data.accel_state()
                    del_time = sensor_data.delta_time()
                    pitch_direct = DirectCalc.yz_offset(accel_state)
                    roll_direct = DirectCalc.xz_offset(accel_state)
                    q_angle = QuaternionEstimation.quaternion_estimate(accel_state, mag_state)
                    #m_angle = MadgwickFilter.madgwick_estimate(accel_state, gyro_state)
                    print(pitch_direct,roll_direct,q_angle)


                else:
                    print("No data recieved")
            except Exception as e:
                print(e)
                break
    except KeyboardInterrupt:
        print('Program Interrupted')
    finally:
        sensor.serial.close()

class Sensor:

    def __init__(self, port):
        self.port = port
        self.serial = serial.Serial(port, 115200)
        self.serial.flushInput()

    def read_data(self):
        ser_bytes = self.serial.readline()  # Read the newest output from the Pico
        if ser_bytes:
            decoded_bytes = ser_bytes[0:len(ser_bytes)-2].decode("utf-8")  # Decode the bytes to a string
            raw_data = decoded_bytes.split(',')  # Assuming data is in the form 'accel_x,accel_y,accel_z'
            return self.preprocess_data(raw_data)   # Preprocesses data (will be used to adjust calibration)
        else:
            return None
    
    def preprocess_data(self, raw_data):
        processed_data = [float(i) for i in raw_data]
        current_state = State(processed_data)
        return current_state
        
class State:

    def __init__(self, processed_data):
        self.accel_x =  processed_data[0]
        self.accel_y =  processed_data[1]
        self.accel_z =  processed_data[2]
        self.mag_x =    processed_data[3] * 0.1
        self.mag_y =    processed_data[4] * 0.1
        self.mag_z =    processed_data[5] * 0.1
        self.gyro_x =   processed_data[6]
        self.gyro_y =   processed_data[7]
        self.gyro_z =   processed_data[8]
        self.time =     time.time()

    def accel_state(self):
        accel_state = np.array([self.accel_x, self.accel_y, self.accel_z], dtype=float)
        return accel_state

    def mag_state(self):
        mag_state = np.array([self.mag_x, self.mag_y, self.mag_z], dtype=float)
        return mag_state
    
    def gyro_state(self):
        gyro_state = np.array([self.gyro_x, self.gyro_y, self.gyro_z], dtype=float)
        return gyro_state
    
    def delta_time(self):
        del_time = self.time - Config.start_time
        return del_time

class DirectCalc:

    def __init__(self):
        pass

    def yz_offset(accel_state):
        yz_direct_offset = math.atan2(accel_state[1], accel_state[2])
        pitch_direct = (yz_direct_offset * 180) / 3.14
        return pitch_direct
    
    def xz_offset(accel_state):
        xz_direct_offset = math.atan2(accel_state[0], accel_state[2])
        roll_direct = (xz_direct_offset * 180) / 3.14
        return roll_direct
    
class QuaternionEstimation:
    
    def __init__(self):
        self.angles = np.zeros(1,3)
        self.time = time.time()

    def quaternion_estimate(accel_state, mag_state):
        samples = accel_state.shape[0]
        Q = np.zeros((samples, 4))
        q = np.zeros((samples, 4))
        Q[0] = [1.0, 0.0, 0.0, 0.0]
        quest = QUEST()
        for t in range(1, samples):
            Q[t] = quest.estimate(acc=accel_state, mag=mag_state)
            q = Quaternion(Q[t])
            q_angle = q.to_angles() * 180 / math.pi
        return q_angle
    
class MadgwickFilter:
    def __init__(self) -> None:
        pass
    
    def madgwick_estimate(accel_state, gyro_state):
            madgwick = Madgwick()
            num_samples = 1
            M = np.tile([1., 0., 0., 0.], (len(gyro_state), 1)) # Allocate for quaternions
            for t in range(1, num_samples):
                M[t] = madgwick.updateIMU(M[t-1], gyr=gyro_state[t], acc=accel_state[t])
                m = Quaternion(M[t])
                m_angle = m.to_angles() * 180 / math.pi
            return m_angle
if __name__ == "__main__":
    main()