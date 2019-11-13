#%%
%matplotlib auto
import numpy as np
import matplotlib.pyplot as plt
import sensor_fusion as sf
import robot_n_measurement_functions as rnmf
import pathlib
import seaborn as sns
import matplotlib.patches as mpatches
from scipy.linalg import expm
sns.set()
#%%
IMU = sf.Sensor('IMU',sf.IMU_COLUMNS,meas_record_file=pathlib.Path.home()/'Dropbox/09. Aalto Postdoc/DiddyBorg_experiment/test-run-imu.csv',is_linear=True,start_index=3686)
IMU_static = sf.Sensor('IMU_static',sf.IMU_COLUMNS,meas_record_file=pathlib.Path.home()/'Dropbox/09. Aalto Postdoc/DiddyBorg_experiment/static_position_IMU.csv',is_linear=True)
Motor_input = sf.Sensor('Motor_input',sf.MOTOR_COLUMNS,meas_record_file=pathlib.Path.home()/'Dropbox/09. Aalto Postdoc/DiddyBorg_experiment/test-run-Motor-Control.csv',is_linear=True)
Camera = sf.Sensor('Camera',sf.CAMERA_COLUMNS,meas_record_file=pathlib.Path.home()/'Dropbox/09. Aalto Postdoc/DiddyBorg_experiment/test-run-camera.csv',is_linear=False,start_index=154)
#%%

bias_omega_z = np.mean(IMU_static.meas_record[:,7])
x_init = np.array([17,60,0.])
x = np.zeros((Motor_input.meas_record.shape[0]+IMU.meas_record.shape[0],3),dtype=np.float)
x[0,:] = x_init
t = np.zeros(x.shape[0])
t[0] = min(IMU.current_time,Motor_input.current_time)

u_now = np.zeros(3)
# #initialize deltaT as IMU deltaT
# IMU_measurement = IMU.get_measurement() #time change here
# u_now[2] = IMU_measurement.flatten()[7] 
# dt = IMU.time_sampling/1000
params = {'u':u_now,
'dt':0,
'x_sensors':np.zeros((1,2))}

I = np.eye(3)
P = I*1e-2
Q = np.diag(np.array([1,1,10]))*1e-3

#measurement_variance for one QR_Code (distance,angle)
R_one_diag = np.array([1,5])*1e1

#%%
IMU.reset_sampling_index()
Motor_input.reset_sampling_index()
IMU_static.reset_sampling_index()
Camera.reset_sampling_index()
#Assumption: t_imu, t_mtr, t_cam always different each other
#Dead REckoning loop --> Naive dynamics
for i in range(1,x.shape[0]):
    x_now = x[i-1,:]
    #determining next time stamp
    #and propagating
    if(IMU.current_time<min(Motor_input.current_time,Camera.current_time)):
        
        t[i] = IMU.current_time
        
        dt = t[i]-t[i-1]
        print('IMU t_i = {}, dt={}'.format(IMU.current_time,dt))
        params['dt'] =dt
        params['u'][2] = IMU.get_measurement().flatten()[7] - bias_omega_z #time change here

        #compute Jacobian
        A = expm(rnmf.robot_jac(x_now,params)*dt)
        #propagate P
        P = A@P@A.T + Q

        #apriori update x to new value using robot_f dynamics
        x[i,:] = rnmf.rungeKutta(x_now,rnmf.robot_f,params)

    elif(Motor_input.current_time<min(IMU.current_time,Camera.current_time)):
        
        t[i] = Motor_input.current_time
        
        dt = t[i]-t[i-1]
        print('Motor t_i = {}, dt={}'.format(Motor_input.current_time,dt))
        params['dt'] =dt
        params['u'][:2] = Motor_input.get_measurement()

        #compute Jacobian
        A = expm(rnmf.robot_jac(x_now,params)*dt)
        #propagate P
        P = A@P@A.T + Q

        #apriori update x to new value using robot_f dynamics
        x[i,:] = rnmf.rungeKutta(x_now,rnmf.robot_f,params)
    
    else:
        # print('EKF')
        t[i] = Camera.current_time
        dt = t[i]-t[i-1]
        print('Camera t_i = {}, dt ={}'.format(Camera.current_time,dt))
        params['dt'] = dt
        y_raw = Camera.get_measurement()
        n_qr_codes = y_raw.shape[0]
        y_cam = y_raw[:,5:].flatten()
        qr_pos = rnmf.QRCODE_LOCATIONS[y_raw[:,0].astype('int'),1:]
        params['x_sensors'] = qr_pos
        x_now = x[i-1,:]

        # #EXTENDED KALMAN FILTER HERE
         #compute Jacobian
        F = expm(rnmf.robot_jac(x_now,params)*dt)

         #propagate P
        P = F@P@F.T + Q

        # #JACOBIAN OF H
        H = rnmf.H_cam(x_now,params)
        R = np.diag(np.kron(np.ones(n_qr_codes),R_one_diag))
        S = H@P@H.T + R
        
        # #KALMAN GAIN
        K_t = np.linalg.solve(S.T,H@P)
        K = K_t.T
        
        

        # #get distance and direction
        y_est = rnmf.h_cam(x_now,params)


        

        #apriori update x to new value using robot_f dynamics
        x_next = rnmf.rungeKutta(x_now,rnmf.robot_f,params)
        
        # #aposteriori update
        x_next = x_next + K@(y_cam-y_est)
        P = (I-K@H)@P
        x[i,:] = x_next


        

        
    
    

#%%
# OLD CODE FOR DEAD RECKONING
IMU.reset_sampling_index()
Motor_input.reset_sampling_index()
IMU_static.reset_sampling_index()
Camera.reset_sampling_index()
x_d = np.zeros_like(x)
x_d[0,:] = x_init
for i in range(1,x_d.shape[0]):
    #update x to new value using robot_f dynamics
    x_d[i,:] = rnmf.rungeKutta(x_d[i-1,:],rnmf.robot_f,params)
    #determining next time stamp
    if(IMU.current_time<Motor_input.current_time):
        t[i] = IMU.current_time
        params['dt'] = t[i]-t[i-1]
        params['u'][2] = IMU.get_measurement().flatten()[7] - bias_omega_z #time change here
    else:
        t[i] = Motor_input.current_time
        params['dt'] = t[i]-t[i-1]
        params['u'][:2] = Motor_input.get_measurement()

# %%
skip=30
end_index=x.shape[0]//3
fig, ax = plt.subplots()
#%%
#Plot x_EKF
q = ax.quiver(x[:end_index:skip,0], x[:end_index:skip,1], -np.sin(x[:end_index:skip,2]), np.cos(x[:end_index:skip,2]),headwidth=1,width=0.0051,alpha=0.8,color='blue')
p = mpatches.Circle((x[0,0], x[0,1]), 1,color='red')
ax.add_patch(p)
p = mpatches.Circle((x[end_index,0], x[end_index,1]), 1,color='black')
ax.add_patch(p)
ax.plot(x[:end_index,0],x[:end_index,1],'-r',linewidth=4,alpha=0.5)
#%%
#Plot x_dead reckoning
q = ax.quiver(x_d[:end_index:skip,0], x_d[:end_index:skip,1], -np.sin(x[:end_index:skip,2]), np.cos(x[:end_index:skip,2]),headwidth=1,width=0.0051,alpha=0.8,color='green')
p = mpatches.Circle((x_d[0,0], x_d[0,1]), 1,color='red')
ax.add_patch(p)
p = mpatches.Circle((x_d[end_index,0], x_d[end_index,1]), 1,color='black')
ax.add_patch(p)
ax.plot(x_d[:end_index,0],x_d[:end_index,1],'-k',linewidth=4,alpha=0.5)


# %%
