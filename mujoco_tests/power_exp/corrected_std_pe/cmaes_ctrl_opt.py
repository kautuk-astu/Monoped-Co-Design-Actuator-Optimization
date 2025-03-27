import cma
import numpy as np
import mujoco as mj
import run_ctrl_para as rcp

#generate code to minimise height reward from rcp using cma
xml_path = '/home/stochlab/Aastha_pybullet/test_pb/power_exp/corrected_std_pe/hopper_roc.xml'
m = mj.MjModel.from_xml_path('/home/stochlab/Aastha_pybullet/test_pb/power_exp/corrected_std_pe/hopper_roc.xml')
d = mj.MjData(m)


def get_reward(action):
    f=open('cpe_ini_std_sig10.txt',"a")   
    base_height_store, reward_store, power_store = rcp.run(xml_path, m, d, action)
    base_max = np.max(base_height_store)
    reward = np.min(reward_store)
    power_reward = power_store  #since reward decreases with height
    ac=action[0]        
    k = str(base_max).strip() + ' ' + str(power_reward).strip() + ' ' + str(ac).strip()          
    f.write(k)
    f.write('\n')
    f.close()
    return power_reward  #minimising power reward
         
    
#CMAES implementation 
            
    

fun = get_reward
x0=np.array([30,0,1,5,1,0])
    
sigma0= 10
opts = cma.CMAOptions()
opts.set({'tolfun' : 1})
opts.set({'tolx': 1})    
opts.set({'verb_disp' : 1000})
opts.set({'verb_filenameprefix': 'cpe_ini_sig10_std'},force=True)
opts.set({'bounds': [([5,0,0,5,1,0]),([100,1,5,20,5,1])]})

x,es=cma.fmin2(fun,x0, sigma0, opts)
