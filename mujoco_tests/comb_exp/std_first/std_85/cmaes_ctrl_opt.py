import cma
import numpy as np
import mujoco as mj
import run_ctrl_para as rcp

#generate code to minimise height reward from rcp using cma
xml_path = '/home/stochlab/Aastha_pybullet/test_pb/maxh_exp/sig10_correct/hopper_roc.xml'
m = mj.MjModel.from_xml_path('/home/stochlab/Aastha_pybullet/test_pb/maxh_exp/sig10_correct/hopper_roc.xml')
d = mj.MjData(m)


def get_reward(action):
    f=open('comb_std_85_maxiter40.txt',"a")   
    base_height_store, height_reward, power_store = rcp.run(xml_path, m, d, action)
    base_max = np.max(base_height_store)
    height_rew = np.min(height_reward)
    reward = 0.85*height_rew + 0.15*power_store 
    ac=action[0]        
    k = str(base_max).strip() + ' ' + str(height_rew).strip() + ' ' + str(power_store).strip()+ ' ' + str(ac).strip()         
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
opts.set({'maxiter': 40})    
opts.set({'verb_disp' : 1000})
opts.set({'verb_filenameprefix': 'comb_std_85_maxiter40'},force=True)
opts.set({'bounds': [([5,0,0,5,1,0]),([100,1,5,20,5,1])]})
    
x,es=cma.fmin2(fun,x0, sigma0, opts)
