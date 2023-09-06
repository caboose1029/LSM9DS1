import serial
import time
import collections

class Config:

    start_time = time.time()
    sensor = 'COM4'
    samples_per_second = 10
    max_samples = 5 * samples_per_second

def main():
    
    sensor = Sensor(Config.sensor)
    
    try:
        while True:
            try:
                processed_data = sensor.read_data()
                if processed_data:
                    state = SensorData(processed_data)
                    print(state.accel_x)
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
        SensorData(processed_data)
        return processed_data
    
class SensorData:
    
    def __init__(self, processed_data):
        self.accel_x =  processed_data[0]
        self.accel_y =  processed_data[1]
        self.accel_z =  processed_data[2]
        self.mag_x =    processed_data[3]
        self.mag_y =    processed_data[4]
        self.mag_z =    processed_data[5]
        self.gyro_x =   processed_data[6]
        self.gyro_y =   processed_data[7]
        self.gyro_z =   processed_data[8]
        self.time =     time.time()

    def accel_state(self):
        accel_x = self.accel_x
        accel_y = self.accel_y
        accel_z = self.accel_z

    def magnet_state(self):
        mag_x = self.mag_x
        mag_y = self.mag_y
        mag_z = self.mag_z

    def gyro_state(self):
        gyro_x = self.gyro_x
        gyro_y = self.gyro_y
        gyro_z = self.gyro_z
        

class DataLogger:
    
    def __init__(self):
        self.accel_x_values = collections.deque(maxlen=Config.max_samples)
        self.accel_y_values = collections.deque(maxlen=Config.max_samples)
        self.accel_z_values = collections.deque(maxlen=Config.max_samples)
        self.elapsed_times = collections.deque(maxlen=Config.max_samples)
        
    def add_data_point(self):
        self.accel_x_values.append(SensorData.accel_x)
        self.accel_y_values.append(SensorData.accel_y)
        self.accel_z_values.append(SensorData.accel_z)
        elapsed_time = time.time() - Config.start_time
        self.elapsed_times.append(elapsed_time)

    def save_to_file(self):
        pass

    def plot_data(self):
        pass
    
if __name__ == "__main__":
    main()
