import numpy as np
import matplotlib.pyplot as plt

def plot():    
    dt = 0.1

    loadfile = np.load('test4_delaystepresponse.npy')
    
    vels = loadfile[0,:]
    caraccls = loadfile[1,:]
    motor_cmds = loadfile[2,:]
    us_distances = loadfile[3,:]

    time_ar = np.linspace(0,dt*len(motor_cmds),len(motor_cmds)+1)[0:-1]
    
    plt.subplot(211)
    #print("shape vels {}").format(np.shape(vels))
    #print("shape time_ar {}").format(np.shape(time_ar))
    plt.plot(time_ar,vels,label="Car Vel",)
    plt.plot(time_ar,caraccls,label="Car Acc")
    plt.plot(time_ar,us_distances,label="Distance to other car")
    plt.legend()

    plt.subplot(212)
    plt.plot(time_ar,motor_cmds,label="Motor (PWM)")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    plot()
