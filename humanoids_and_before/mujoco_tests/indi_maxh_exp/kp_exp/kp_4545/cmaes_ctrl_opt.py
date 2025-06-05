import cma
import numpy as np
import mujoco as mj
import run_ctrl_para as rcp

#generate code to minimise height reward from rcp using cma
xml_path = '/home/stochlab/Aastha_pybullet/test_pb/indi_maxh_exp/kp_exp/kp_4545/modified_robot.xml'
m = mj.MjModel.from_xml_path(xml_path)
d = mj.MjData(m)


def get_reward(action):
    f=open('maxh_std_kp20_test.txt',"a")   
    base_height_store, height_reward, power_store = rcp.run(xml_path, m, d, action)
    base_max = np.max(base_height_store)
    height_rew = np.min(height_reward)
    reward = height_rew  
    ac=action[0]    
    k = str(base_max).strip() + ' ' + str(height_rew).strip() + ' ' + str(ac).strip()         
    f.write(k)
    f.write('\n')
    f.close()
    return reward 
         
    
#CMAES implementation 
            
    

fun = get_reward
x0=np.array([30])
    
sigma0= 10
opts = cma.CMAOptions()
opts.set({'tolfun' : 0.5})
opts.set({'tolx': 1})
opts.set({'maxiter': 20})    
opts.set({'verb_disp' : 1000})
opts.set({'verb_filenameprefix': 'maxh_std_kp_maxiter20_test'},force=True)
opts.set({'bounds': [([5]),([100])]})
x,es=cma.fmin2(fun,x0, sigma0, opts)
