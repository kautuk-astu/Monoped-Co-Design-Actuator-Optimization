import cma
import numpy as np
import mujoco as mj
import run_ctrl_para as rcp

#generate code to minimise height reward from rcp using cma
xml_path = 'modified_robot.xml'
m = mj.MjModel.from_xml_path(xml_path)
d = mj.MjData(m)


def get_reward(action):
    f=open('30_75.txt',"a")   
    base_height_store, height_reward, reward_store = rcp.run(xml_path, m, d, action)
    base_max = np.max(base_height_store)
    reward = np.min(height_reward)
    total_cost = 0.75*reward + 0.25*reward_store #since reward decreases with height
    ac=action[0]        
    k = str(base_max).strip() + ' ' + str(reward).strip() + ' ' + str(reward_store).strip() + ' ' +  str(ac).strip()         
    f.write(k)
    f.write('\n')
    f.close()
    return total_cost
#CMAES implementation           
fun = get_reward
x0=np.array([30,0,1,1,1,0])    
sigma0= 5
opts = cma.CMAOptions()
opts.set({'tolfun' : 2})
opts.set({'tolx': 1})
opts.set({'maxiter': 100})    
opts.set({'verb_disp' : 1000})
opts.set({'verb_filenameprefix': '30_75'},force=True)
opts.set({'bounds': [([5,0,0,1,1,0]),([200,1,5,20,10,1])]})
    
x,es=cma.fmin2(fun,x0, sigma0, opts)
