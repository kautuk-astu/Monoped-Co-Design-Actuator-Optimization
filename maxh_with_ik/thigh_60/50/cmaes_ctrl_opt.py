import cma
import numpy as np
import mujoco as mj
import run_ctrl_para as rcp
import csv
xml_path = 'modified_robot.xml'
m = mj.MjModel.from_xml_path(xml_path)
d = mj.MjData(m)
csv_filename = "data.csv"

def get_reward(action):   
    #f=open('60.txt',"a")
    #print(d)   
    base_height_store, height_reward, te, he, ke, ht, kt = rcp.run(xml_path, m, d, action)
    base_max = np.max(base_height_store)
    reward = np.min(height_reward)
    max_hip_torque = np.max(ht)
    max_knee_torque = np.max(kt)
    
    ac1=action[0]
    ac2 = action[1]
    ac3 = action[2]
    ac4 = action[3]
    ac5 = action[4]
    ac6 = action[5]
    ac = [ac1, ac2, ac3, ac4, ac5, ac6]        
    k = str(base_max).strip() + ' ' + str(reward).strip() + ' ' + str(te).strip() + ' ' + str(ac).strip()         
    with open(csv_filename, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([base_max, reward, te, he, ke, max_hip_torque, max_knee_torque] + list(action))
    return reward          
    
#CMAES implementation       
fun = get_reward
x0 = np.array([30,0,1,1,1,0])    
sigma0= 5
opts = cma.CMAOptions()
opts.set({'tolfun' : 2})
opts.set({'tolx': 1})
opts.set({'maxiter': 40})    
opts.set({'verb_disp' : 1000})
opts.set({'verb_filenameprefix': '50'},force=True)
opts.set({'bounds': [([5,0,0,1,1,0]),([200,1,5,20,10,1])]})  

with open(csv_filename, "w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["max_height", "cost", "te", "he", "ke", "max_ht", "max_kt"] + [f"ac{i+1}" for i in range(len(x0))])

x, es = cma.fmin2(fun , x0, sigma0, opts)
