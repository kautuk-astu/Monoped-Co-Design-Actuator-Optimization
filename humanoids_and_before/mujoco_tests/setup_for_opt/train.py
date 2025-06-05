
import numpy as np
import matplotlib.pyplot as plt
import cma
import pandas as pd
from scipy.signal import find_peaks
def run():
    env = SingleLegEnv()    
    env.reset()

    def get_reward(action):
        f=open('kp25_maxh_kdneg.txt',"a")
        reward=[]
        env.reset()
        base=[]
        
        for i in range(2400):   #for checking stability for longer period     
            r, dev, desired_angles_1,desired_angles_2,desired_angles_3, actual_pos, power, motor_position,torque,force,ee_pos_x,ee_pos_y=env.step(action)
            reward.append(r)
            base.append(motor_position[0] + 0.15)
        #b=base[1200+np.argmax(base[1200:2399])]
        b= base[np.argmax(base)]
        b1= base[np.argmax(base)]        
        r1= reward[np.argmin(reward)]
        r2= reward[np.argmin(reward)]
        ac=action[4]
        ac2=action[5]
        k = str(b).strip() + ' ' + str(r1).strip() + ' ' + str(ac).strip()  + ' ' + str(ac2).strip()
        
        f.write(k)
        f.write('\n')
        f.close()
        return r2 #anubhab

    fun = get_reward
    x0=np.array([65,2.5,65,25,25,20])
    '''
    sigma-step size

    '''
    sigma0= 1
    opts = cma.CMAOptions()
    opts.set({'tolfun' : 1e-1})
    opts.set({'tolx': 5e-2})    
    opts.set({'verb_disp' : 1000})
    opts.set({'verb_filenameprefix': 'kp25_maxh_kdneg'},force=True)
    opts.set({'bounds': [([10,0,10,0,3,0]),([100,100,100,100,100,100])]})

    #es = cma.CMAEvolutionStrategy(x0, sigma0,{'bounds': [([150,0,150,0,300,0]),([1400,4,1400,4,10000,10])]})
    #es.opts.set({'verb_filenameprefix': 'outcmaes_2400'},force=True)
    #es.opts.set({'verb_filenameprefix': 'outcmaes_2400'})
    #es.opts.set({'popsize':6})
    #es.opts.set({'tolfacupx': 1e4})

    
    #es.opts.set({'tolfun' : 1e-3})
    #es.opts.set({'tolx': 1e-3})
    x,es=cma.fmin2(fun,x0, sigma0, opts)
    #cma.plot()
    #cma.fmin2(fun,x0, sigma0,{'bounds': [([150,0,150,0,300,0]),([1400,4,1400,4,10000,10])]})
    #cma.plot()
    #res = es.result
    #while not es.stop():
        #solutions = es.ask()
        #es.tell(solutions, [cma.ff.rosen(s) for s in solutions])
        #es.disp()
    #es.result_pretty()
    
    #x,es=cma.fmin2(fun,x0,sigma0,{'bounds': [([150,0,150,0,300,0]),([1400,4,1400,4,10000,10])]})
    print(es.result)