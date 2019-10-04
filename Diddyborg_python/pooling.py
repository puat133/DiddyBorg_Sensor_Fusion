import os                                                                       
from multiprocessing import Pool                                                
                                                                                
                                                                                
# processes = ('a.py','b.py', 'c.py')
processes = ('MotorControl.py','IMU.py')                                                               
def run_process(process):                                                             
    os.system('python3 {}'.format(process))                                       
                                                                                
                                                                                
pool = Pool(processes=2)                                                        
pool.map(run_process, processes)