import numpy as np

#QR-code/x/y
#the entry is QR-number, x, y
QRCODE_LOCATIONS = np.array([
[0,0,0],
[1,120,47.75],
[2,120,59.75],
[3,120,71.75],
[4,0,35.65],
[5,0,23.65],
[6,0,11.65],
[7,120,84.4],
[8,120,96.4],
[9,120,108.4],
[10,12.6,0],
[11,24.6,0],
[12,36.6,0],
[13,120,11.85],
[14,120,23.85],
[15,120,35.85],
[16,48.95,0],
[17,60.95,0],
[18,72.95,0],
[19,13.25,120],
[20,0,120],
[21,0,120],
[22,35.05,0],
[23,23.05,0],
[24,11.05,0],
[25,49.75,120],
[26,61.75,120],
[27,73.75,120],
[28,0,71.55],
[29,0,59.55],
[30,0,47.55],
[31,85.95,0],
[32,97.95,0],
[33,109.95,0],
[34,0,108],
[35,0,96],
[36,0,84]
]
)
GRAVITY_REF = 9.8192e3 #cm/s2 https://www.sensorsone.com/local-gravity-calculator/#latitude at sea level, latitude 60.18798125
GAUSS_TO_MICRO_TESLA = 100
MOTOR_FULL_SPEED = 6.9306/0.3 #cm/s full speed
DEG_TO_RAD = np.pi/180
RAD_TO_DEG = 180/np.pi

#Model
"""
Deviation angle of a QR-codes with respect to the camera (robot) y axis, all variables are given in global coordinates
"""
def phi_(x_c,y_c,x_i,y_i,psi):
    return np.arctan2((x_c-x_i),(y_i-y_c)) - psi

"""
camera distance to the detected QR_codes, all variables are given in global coordinates
"""
def dist_(x_c,y_c,x_i,y_i):
    return np.sqrt((x_c-x_i)**2+(y_c-y_i)**2)

"""
measurement function of cameras
"""
def h_cam(x,params):
    x_sensors = params['x_sensors']#x,y position of each QRcodes
    h = np.zeros(2*x_sensors.shape[0])
    x_c = x[0]
    y_c = x[1]
    psi = x[2]
    for i in range(x_sensors.shape[0]):
        # dist = np.sqrt((x_c-x_sensors[i,0])**2+(y_c-x_sensors[i,1])**2)
        # phi = np.arctan2((x_sensors[i,0]-x_c),(x_sensors[i,1]-y_c)) - psi
        h[i*2] = dist_(x_c,y_c,x_sensors[i,0],x_sensors[i,1])
        h[i*2+1] = phi_(x_c,y_c,x_sensors[i,0],x_sensors[i,1],psi)
    
    return h

"""
"""
def rungeKutta(x,fun,params):
    dt = params['dt']
    k1 = fun(x,params)
    k2 = fun(x+(0.5*dt*k1),params)
    k3 = fun(x+(0.5**dt*k2),params)
    k4 = fun(x+(dt*k3),params)

    return x+ dt*(k1+2*k2+2*k3+k4)/6




"""
x_dot = f(x,u) + w
x_1 = global x position
x_2 = global y position
x_3 = yaw angle
u_1 = motor input 1
u_2 = motor input 2
u_3 = gyrometer z
"""
def robot_f(x,params):
    u = params['u']
    psi=x[2]
    cPsi = np.cos(psi)
    sPsi = np.sin(psi)
    uBar = 0.5*(u[0]+u[1])*MOTOR_FULL_SPEED
    xdot = np.array([-sPsi*uBar,
    cPsi*uBar,
    DEG_TO_RAD*u[2]
    ])
    return xdot

def robot_jac(x,params):
    u = params['u']
    psi=x[2]
    cPsi = np.cos(psi)
    sPsi = np.sin(psi)
    uBar = 0.5*(u[0]+u[1])*MOTOR_FULL_SPEED

    jac = np.zeros((x.shape[0],x.shape[0]))
    jac[0,2] = -cPsi*uBar
    jac[1,2] = -sPsi*uBar
    return jac


"""
x_dot = f(x,u) 
x_1 = global x position
x_2 = global y position
x_3 = yaw angle
x_4 = sway speed
x_5 = surge speed
x_6 = yaw speed
u_1 = accelerometer_x
u_2 = accelerometer_y
u_3 = gyrometer z
"""
def robot_f_2(x,params):
    xdot = np.zeros(6)
    u = params['u']
    std_dev_x_6 = params['std_dev_x_6']
    psi=x[2]
    cPsi = np.cos(psi)
    sPsi = np.sin(psi)
    R = np.array([[cPsi,-sPsi],[sPsi,cPsi]])
    xdot[:2] = R@x[3:5]
    xdot[2] = DEG_TO_RAD*u[2]
    xdot[3:5] = GRAVITY_REF*u[:2]
    xdot[5] = 0#std_dev_x_6*np.random.randn()
    return xdot
    



