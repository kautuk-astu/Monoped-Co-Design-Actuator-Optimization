import cma
import numpy as np
import mujoco as mj
import run_ctrl_para as rcp

#generate code to minimise height reward from rcp using cma
xml_path = 'test_pb/hopper_roc.xml'
m = mj.MjModel.from_xml_path('test_pb/hopper_roc.xml')
d = mj.MjData(m)


def get_reward(action):
    f=open('maxh_ini_std.txt',"a")   
    base_height_store, base_vel_store, reward_store = rcp.run(xml_path, m, d, action)
    base_max = np.max(base_height_store)
    reward = np.min(reward_store)  #since reward decreases with height
    ac=action[0]
        
    k = str(base_max).strip() + ' ' + str(reward).strip() + ' ' + str(ac).strip()  
        
    f.write(k)
    f.write('\n')
    f.close()
    return reward 
         
    
#CMAES implementation 
            
    

fun = get_reward
x0=np.array([30,0,1,5,1,0])
    
sigma0= 3
opts = cma.CMAOptions()
opts.set({'tolfun' : 2})
opts.set({'tolx': 1})    
opts.set({'verb_disp' : 1000})
opts.set({'verb_filenameprefix': 'maxh_ini_sig3_std'},force=True)
opts.set({'bounds': [([5,0,0,5,1,0]),([100,1,5,20,5,1])]})
    
x,es=cma.fmin2(fun,x0, sigma0, opts)
