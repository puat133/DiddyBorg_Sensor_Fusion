#%%
%matplotlib auto
import numpy as np
import matplotlib.pyplot as plt
import sensor_fusion as sf
import robot_n_measurement_functions as rnmf
import pathlib
import seaborn as sns
import matplotlib.patches as mpatches
sns.set()
#%%
IMU = sf.Sensor('IMU',sf.IMU_COLUMNS,meas_record_file=pathlib.Path.home()/'Dropbox/09. Aalto Postdoc/DiddyBorg_experiment/test-run-imu.csv',is_linear=True,start_index=3686)
IMU_static = sf.Sensor('IMU_static',sf.IMU_COLUMNS,meas_record_file=pathlib.Path.home()/'Dropbox/09. Aalto Postdoc/DiddyBorg_experiment/static_position_IMU.csv',is_linear=True)
Motor_input = sf.Sensor('Motor_input',sf.MOTOR_COLUMNS,meas_record_file=pathlib.Path.home()/'Dropbox/09. Aalto Postdoc/DiddyBorg_experiment/test-run-Motor-Control.csv',is_linear=True)

#%%
IMU.reset_sampling_index()
Motor_input.reset_sampling_index()
IMU_static.reset_sampling_index()
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
#Dead REckoning loop --> Naive dynamics
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
# plt.figure()
#plt.plot(x[:,0],x[:,1],'-ok',linewidth=0.5,markersize=2)
skip=30
end_index=x.shape[0]//2
fig, ax = plt.subplots()
q = ax.quiver(x[:end_index:skip,0], x[:end_index:skip,1], -np.sin(x[:end_index:skip,2]), np.cos(x[:end_index:skip,2]),headwidth=1,width=0.0051,alpha=0.8,color='blue')
p = mpatches.Circle((x[0,0], x[0,1]), 1,color='red')
ax.add_patch(p)
p = mpatches.Circle((x[end_index,0], x[end_index,1]), 1,color='black')
ax.add_patch(p)
ax.plot(x[:end_index,0],x[:end_index,1],'-r',linewidth=4,alpha=0.5)

#%%
IMU.reset_sampling_index()
Motor_input.reset_sampling_index()
IMU_static.reset_sampling_index()
biases = np.mean(IMU_static.meas_record[:,[0,1,7]])
x_init = np.array([17,60,0.,0.,0.,0.])
x = np.zeros((IMU.meas_record.shape[0],6),dtype=np.float)
x[0,:] = x_init
t = np.zeros(x.shape[0])
t[0] = IMU.current_time #min(IMU.current_time,Motor_input.current_time)
u_now = np.zeros(3)
#initialize deltaT as IMU deltaT
u_now = IMU.get_measurement()[[0,1,7]] -biases#time change here
dt = IMU.time_sampling/1000
params = {'u':u_now,
'dt':dt,
'std_dev_x_6':1e-5}
#%%
for i in range(1,x.shape[0]):
    #update x to new value using robot_f dynamics
    x[i,:] = rnmf.rungeKutta(x[i-1,:],rnmf.robot_f_2,params)
    #determining next time stamp
    # if(IMU.current_time<Motor_input.current_time):
    t[i] = IMU.current_time
    params['dt'] = t[i]-t[i-1]
    params['u'] = IMU.get_measurement()[[0,1,7]] - biases #time change here
#%%    
plt.figure()
plt.plot(x[:,0],x[:,1],'-ok',linewidth=0.5,markersize=2)
#%%
