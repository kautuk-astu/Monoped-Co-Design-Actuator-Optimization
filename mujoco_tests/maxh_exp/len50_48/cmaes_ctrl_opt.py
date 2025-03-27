import cma
import numpy as np
import mujoco as mj
import run_ctrl_para as rcp

#generate code to minimise height reward from rcp using cma
xml_path = '/home/stochlab/Aastha_pybullet/test_pb/maxh_exp/len50_48/modified_robot.xml'
m = mj.MjModel.from_xml_path(xml_path)
d = mj.MjData(m)


def get_reward(action):
    f=open('maxh_std_sig10.txt',"a")   
    base_height_store, height_reward, reward_store = rcp.run(xml_path, m, d, action)
    base_max = np.max(base_height_store)
    reward = np.min(height_reward) #since reward decreases with height
    ac=action[0]        
    k = str(base_max).strip() + ' ' + str(reward).strip() + ' ' + str(ac).strip()         
    f.write(k)
    f.write('\n')
    f.close()
    return reward 
         
    
#CMAES implementation 
            
    

fun = get_reward
x0=np.array([30,0,1,5,1,0])
    
sigma0= 10
opts = cma.CMAOptions()
opts.set({'tolfun' : 4})
opts.set({'tolx': 2})
opts.set({'maxiter': 70})    
opts.set({'verb_disp' : 1000})
opts.set({'verb_filenameprefix': 'maxh_std_sig10'},force=True)
opts.set({'bounds': [([5,0,0,5,1,0]),([100,1,5,20,5,1])]})
    
x,es=cma.fmin2(fun,x0, sigma0, opts)
