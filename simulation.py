#%%
%matplotlib auto
import numpy as np
import matplotlib.pyplot as plt
import sensor_fusion as sf
import robot_n_measurement_functions as rnmf
#%%
IMU = sf.Sensor('IMU',sf.IMU_COLUMNS,meas_record_file='C:/Users/Muhammad/Dropbox/09. Aalto Postdoc/DiddyBorg_experiment/test-run-imu.csv',is_linear=True,start_index=3686)
IMU_static = sf.Sensor('IMU_static',sf.IMU_COLUMNS,meas_record_file='C:/Users/Muhammad/Dropbox/09. Aalto Postdoc/DiddyBorg_experiment/static_position_IMU.csv',is_linear=True)
Motor_input = sf.Sensor('Motor_input',sf.MOTOR_COLUMNS,meas_record_file='C:/Users/Muhammad/Dropbox/09. Aalto Postdoc/DiddyBorg_experiment/test-run-Motor-Control.csv',is_linear=True)

#%%
bias_omega_z = np.mean(IMU_static.meas_record[:,7])
x_init = np.array([17,60,0.])
x = np.zeros((Motor_input.meas_record.shape[0]+IMU.meas_record.shape[0],3),dtype=np.float)
x[0,:] = x_init
t = np.zeros(x.shape[0])
t[0] = min(IMU.current_time,Motor_input.current_time)
u_now = np.zeros(3)
#initialize deltaT as IMU deltaT
IMU_measurement = IMU.get_measurement() #time change here
u_now[2] = IMU_measurement[7] 
dt = IMU.time_sampling/1000
params = {'u':u_now,
'dt':dt}

#%%
#Dead REckoning loop
for i in range(1,x.shape[0]):
    #update x to new value using robot_f dynamics
    x[i,:] = rnmf.rungeKutta(x[i-1,:],rnmf.robot_f,params)
    #determining next time stamp
    if(IMU.current_time<Motor_input.current_time):
        t[i] = IMU.current_time
        params['dt'] = t[i]-t[i-1]
        params['u'][2] = IMU.get_measurement()[7] - bias_omega_z #time change here
    else:
        t[i] = IMU.current_time
        params['dt'] = t[i]-t[i-1]
        params['u'][:2] = Motor_input.get_measurement()






#%%
