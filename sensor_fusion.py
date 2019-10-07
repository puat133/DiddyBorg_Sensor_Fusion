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
        ('gyroscope x','deg/s'),
        ('gyroscope y','deg/s'),
        ('gyroscope z','deg/s'),
        ('magnetometer x','Gauss'),
        ('magnetometer y','Gauss'),
        ('magnetometer z','Gauss')]

GRAVITY_REF = 9.8
GAUSS_TO_MICRO_TESLA = 100


class Sensor:
    """
    meas_record should not include the column names. 
    meas_record first column should be time_stamp.
    meas_fun only include the deterministic part
    column
    """
    def __init__(self,name,column,meas_record_file,is_linear,meas_fun=None,meas_Jacobian=None,meas_variance=None):
        self.__name = name
        self.__column = column #<-- a List of tuple contains column name and unit
        self.__column_num = len(column)#<-- column number excluding time stamp
        raw_meas_record = np.loadtxt(meas_record_file,delimiter=',')
        self.__h = meas_fun
        self.__H = meas_Jacobian
        self.__is_linear = is_linear
        self.__current_sample_index = 0

        #by default use all columns
        self.__column_used = np.arange(self.__column_num)
        self.__column_changed = False
        
        #assuming that time_stamp is the first column
        raw_time = raw_meas_record[:,0]-raw_meas_record[0,0]
        self.__time_sampling = np.int(np.ceil(raw_time[-1]*1000/raw_time.shape[0]))#in milliseconds
        self.__time = raw_meas_record[:,0] #np.arange(raw_time.shape[0])*self.__time_sampling
        self.__meas_record = raw_meas_record[:,1:]
        
        
        #Do some checking here
        min_column_length = self.__column_num
        if self.__meas_record.shape[1] < min_column_length:
            raise Exception('Invalid measurement record shape.\n Measurement record is expected to have at least {0} columns, which should include the time stamp at the beginning'.format(min_column_length))
        if meas_variance is None:
            self.__meas_variance = self.__get_variance()
        else:
            self.meas_variance(meas_variance)
            

    """
    this routine is used to obtain the sensor variance.
    in the case of IMU, we assume that the robot is in static position
    """
    def __get_variance(self,assume_diag=False):
        R = np.cov(self.meas_record.T)
        if assume_diag:
            return np.diag(np.diag(R))
        else:
            return R
    
    def get_measurement(self):
        current_measurement = self.__meas_record[self.current_sample_index,self.__column_used]
        self.__current_sample_index +=1
        return current_measurement

    def reset_sampling_index(self):
        self.__current_sample_index = 0
    #GETTER
    @property
    def name(self):
        return self.__name
    
    @property
    def column(self):
        return self.__column
    
    @property
    def column_num(self):
        return self.__column_num

    @property
    def fun(self):
        return self.__h
    
    @property
    def jac(self):
        return self.__H

    @property
    def is_linear(self):
        return self.__is_linear

    @property
    def current_sample_index(self):
        return self.__current_sample_index
    
    @property
    def time_sampling(self):
        return self.__time_sampling
    
    @property
    def time(self):
        return self.__time
    
    @property
    def current_time(self):
        return self.time[self.__current_sample_index]

    @property
    def meas_record(self):
        return self.__meas_record[:,self.__column_used]
    
    @property
    def meas_variance(self):
        if self.__column_changed:
            self.__meas_variance = self.__get_variance()
            self.__column_changed = False
        return self.__meas_variance

    @meas_variance.setter
    def meas_variance(self,meas_variance):
        if meas_variance.dtype.kind not in ['f','i']:
            raise Exception('Measurement variance data type should be floating number or integer')
        else:
            try:
                test = np.linalg.cholesky(meas_variance)
            except np.linalg.LinAlgError:
                raise('Measurement Variance is not positive definite!')
            
            self.__meas_variance = meas_variance 

    @property
    def column_used(self):
        return self.__column_used
    
    @column_used.setter
    def column_used(self,column_used):
        if len(column_used)<= self.column_num:
            if isinstance(column_used,np.ndarray) and column_used.dtype.kind == 'i':
                if np.min(column_used)<0 or np.max(column_used)>= self.column_num:
                    raise Exception('column index invalid.')
                else:
                    self.__column_used = column_used
                    self.__column_changed = True
            else:
                raise Exception('column_used must be integer NDarray')
                
        else:
            raise Exception('Too many columns')






ROBOT_STATE_COLUMNS = [('body x velocity','m/s'),
        ('body y velocity','m/s'),
        ('body yaw angular velocity','deg/s'),
        ('body y velocity','m/s'),
        ('local x velocity','m'),
        ('local y velocity','m/s'),
        ('local yaw angle','deg')]
class Filter:
    def __init__(self,name,num_state,sensors,init_state,P_init,state_process_variance=None):
        self.__name = name
        self.__num_state = num_state
        self.__current_state = init_state
        self.__current_P = P_init
        self.__history_length = 10000
        self.__estimation_history = np.empty((self.__history_length,self.__num_state),dtype=np.float64)
        self.__P_history = np.empty((self.__history_length,self.__num_state,self.__num_state),
        dtype=np.float64)
        self.__I = np.eye(num_state)
        
        #Do some checking
        if sensors is None:
            raise Exception('Please add at minimum one sensor!')
        else:
            for i in range(len(sensors)):
                if not isinstance(sensors[i],Sensor):
                    raise Exception('Please only use sensor_fusion.sensor object!')

        self.__sensors = sensors
        
        self.evaluate_sensible_sampling_time()
        self.__current_sample_index = 0

        if state_process_variance is None:
            self.__state_process_variancee = 0.1*np.eye(self.num_state)
        else:
            if state_process_variance.dtype.kind not in ['f','i']:
                raise Exception('Measurement variance data type should be floating number or integer')
            else:
                self.__state_process_variance = state_process_variance
        

    """
    obtain the greatest common division of each sensor time sampling
    """
    def evaluate_sensible_sampling_time(self):
        time_sampling = []
        for sensor in self.__sensors:
            time_sampling.append(sensor.time_sampling)
        self.__time_sampling = np.gcd.reduce(time_sampling)

    def predict(self):
        pass

    def update(self,sensor):
        pass

    def run(self):
        for self.__current_sample_index in range(self.__history_length):
            self.predict()
            time = self.time_sampling*self.__current_sample_index
            for sensor in self.sensors:
                if self.current_time == sensor.current_time:
                    self.update(sensor)
            self.__estimation_history[self.__current_sample_index,:] = self.__current_state
            self.__P_history[self.__current_sample_index,:,:] = self.__current_P



    #GETTER
    @property
    def name(self):
        return self.__name
    
    @property
    def num_state(self):
        return self.__num_state

    @property
    def sensors(self):
        return self.__sensors
    
    
    @property
    def current_sample_index(self):
        return self.__current_sample_index

    @property
    def current_state(self):
        return self.__current_state
    
    @property
    def time_sampling(self):
        return self.__time_sampling
    
    @property
    def state_process_variance(self):
        return self.__state_process_variance

    @property
    def current_time(self):
        return self.__current_sample_index*self.__time_sampling
class KalmanFilter(Filter):
    def __init__(self,num_state,sensors,init_state,state_process_variance=None):
        super().__init__(self,"Kalman Filter",num_state,sensors,init_state,state_process_variance=state_process_variance)

   

class ExtendedKalmanFilter(Filter):
    def __init__(self,num_state,sensors,init_state,P_init,fun,jac,state_process_variance):
        super().__init__(self,"Extended Kalman Filter",num_state,sensors,init_state,state_process_variance=state_process_variance)
        self.__fun = fun
        self.__jac = jac
        self.__F = np.empty((self.__num_state,self.__num_state),dtype=np.float64)
    
    def predict(self):
        self.__F = self.__jac(self.__current_state)        
        self.__current_state = self.__fun(self.__current_state)
        self.__current_P =  self.__F@self.__current_P@self.__F.T + self.state_process_variance

    def update(self,sensor):
        deltaY = sensor.get_measurement()-sensor.fun(self.__current_state)
        H = sensor.jac(self.__current_state)
        S = H@self.__current_P@H.T + sensor.meas_variance
        K = np.linalg.solve(S,H@self.__current_P).T
        self.__current_state = self.__current_state + K@deltaY
        self.__current_P = (self.__I - K@H)@self.__current_P


"""
"""
def _rungeKutta(x,fun,params):
    dt = params['dt']
    k1 = fun(x,params)
    k2 = fun(x+(0.5*dt*k1),params)
    k3 = fun(x+(0.5**dt*k2),params)
    k4 = fun(x+(dt*k3),params)

    return x+ dt*(k1+2*k2+2*k3+k4)/6


H_IMU = np.array([0.,0.,0.,0.,0.,1.])

"""
x_dot = f(x) + B w
x_1 = global x position
x_2 = global y position
x_3 = yaw angle
x_4 = surge speed
x_5 = sway speed
x_6 = yaw speed
"""
def _robot_f(x):
    psi=x[2]
    u = x[3]
    v = x[4]
    psiDot = x[5]

    cPsi = np.cos(psi)
    sPsi = np.sin(psi)
    J = np.array([[cPsi,-sPsi],[sPsi,cPsi]])
    xdot = np.concatenate((J@x[:2],np.array([psiDot]),np.zeros(3)))
    return xdot

def _robot_dynamics(x,params):
    #u comprises ax,ay,and psiDotDot
    u = params['u']

    return _robot_f(x)+np.concatenate((np.zeros(3),u))

def _robot_jac(x):
    jac = np.zeros((x.shape[0],x.shape[0]))
    psi=x[2]
    u = x[3]
    v = x[4]
    psiDot = x[5]
    cPsi = np.cos(psi)
    sPsi = np.sin(psi)

    jac[0,2] = -sPsi*u - cPsi*v
    jac[0,3] = cPsi
    jac[0,4] = -sPsi

    jac[1,2] = cPsi*u - sPsi*v
    jac[1,3] = sPsi
    jac[1,4] = cPsi

    jac[2,5] = 1


