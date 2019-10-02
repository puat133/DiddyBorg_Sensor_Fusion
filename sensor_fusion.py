import numpy as np
import matplotlib.pyplot as plt


#https://robotics.stackexchange.com/questions/1858/why-do-3-axis-accelerometers-seemingly-have-a-left-handed-coordinate-system
# The answer is that 3-axis accelerometers don't have a left handed coordinate system just for the gravity. In static condition (i.e. if the accelerometer is not accelerating with respect to any inertial frame) they measure the opposite of gravity acceleration, not the gravity acceleration itself.

# In more general terms, the accelerometers measure the difference between the actual acceleration of the sensor with respect to any inertial frame and the gravitational acceleration:
# a_accelerometer=a_sensorFrame−g
# This "acceleration" measured by the accelerometer is sometimes called proper acceleration.

# This can be easily verified by checking the measure of an accelerometer in free fall: as in that case the actual acceleration of the sensor will be equal to g, the accelerometer measure will be 0.

# class Stochastic_Dynamics:

#The Accelerometer follows right hand rule.
IMU_COLUMNS = [('accelerometer x','g'),
        ('accelerometer y','g'),
        ('accelerometer z','g'),
        ('roll angle','deg'),
        ('pitch angle','deg'),
        ('gyroscope x','rad/s?'),
        ('gyroscope z','rad/s?'),
        ('gyroscope y','rad/s?'),
        ('magnetometer x','rad?'),
        ('magnetometer y','rad?'),
        ('magnetometer z','rad?')
        ]

class Sensor:
    """
    meas_record should not include the column names. 
    meas_record first column should be time_stamp.
    meas_fun only include the deterministic part
    column
    """
    def __init__(self,name,column,meas_record_file,is_linear,meas_fun=None,meas_Jacobian=None,meas_variance=None):
        self.name = name
        self.column = column #<-- a List of tuple contains column name and unit
        self.column_num = len(column)#<-- column number excluding time stamp
        raw_meas_record = np.loadtxt(meas_record_file,delimiter=',')
        self.h = meas_fun
        self.H = meas_Jacobian
        self.is_linear = is_linear
        self.current_sample_index = 0
        
        #assuming that time_stamp is the first column
        raw_time = raw_meas_record[:,0]-raw_meas_record[0,0]
        self.time_sampling = np.int(np.ceil(raw_time[-1]/raw_time.shape[0]))
        self.time = np.arange(raw_time.shape[0])*self.time_sampling
        self.meas_record = raw_meas_record[:,1:]
        
        
        #Do some checking here
        min_column_length = self.column_num
        if self.meas_record.shape[1] < min_column_length:
            raise Exception('Invalid measurement record shape.\n Measurement record is expected to have at least {0} columns, which should include the time stamp at the beginning'.format(min_column_length))
        if meas_variance is None:
            self.meas_variance = self.get_variance()
        else:
            if meas_variance.dtype.kind not in ['f','i']:
                raise Exception('Measurement variance data type should be floating number or integer')
            else:
                self.meas_variance = meas_variance

    """
    this routine is used to obtain the sensor variance.
    in the case of IMU, we assume that the robot is in static position
    """
    def get_variance(self,assume_diag=False):
        R = np.cov(self.meas_record.T)
        if assume_diag:
            return np.diag(np.diag(R))
        else:
            return R


    def get_measurement(self):
        current_measurement = self.meas_record[self.current_sample_index,:]
        self.current_sample_index +=1
        return current_measurement





class Filter:
    def __init__(self,name,num_state,sensors,state_variance=None):
        self.name = name
        self.num_state = num_state
        
        #Do some checking
        if sensors is None:
            raise Exception('Please add at minimum one sensor!')
        else:
            for i in range(len(sensors)):
                if not isinstance(sensors[i],Sensor):
                    raise Exception('Please only use sensor_fusion.sensor object!')

        self.sensors = sensors
        
        self.evaluate_sensible_sampling_time()

        if state_variance is None:
            self.state_variance = 0.1*np.eye(self.num_state)
        else:
            if state_variance.dtype.kind not in ['f','i']:
                raise Exception('Measurement variance data type should be floating number or integer')
            else:
                self.state_variance = state_variance
        

    """
    obtain the greatest common division of each sensor time sampling
    """
    def evaluate_sensible_sampling_time(self):
        time_sampling = []
        for sensor in self.sensors:
            time_sampling.append(sensor.time_sampling)
        self.time_sampling = np.gcd.reduce(time_sampling)

    def predict(self):
        pass

    def update(self):
        pass





