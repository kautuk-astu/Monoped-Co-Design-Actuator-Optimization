"""CMA-ES optimisation for jump_engine_v2; output isolated from baseline."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import cma
from jump_engine_v2 import evaluate,CSV_COLUMNS
ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--generations',type=int,default=30)
    p.add_argument('--population',type=int,default=12)
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--duration',type=float,default=1.4)
    p.add_argument('--min-flight',type=float,default=.08)
    p.add_argument('--motor-config',default='motor_config_v2.json')
    p.add_argument('--output',default='results/v2')
    a=p.parse_args()
    if a.generations<1 or a.population<4:p.error('generations>=1 and population>=4 required')
    cfg=json.loads((ROOT/a.motor_config).read_text())
    out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True)
    lo=np.array([4,4,20,.1,20,.1],float)
    hi=np.array([25,25,900,30,900,30],float)
    def decode(y):return lo+np.clip(y,0,1)*(hi-lo)
    es=cma.CMAEvolutionStrategy(np.full(6,.5),.22,
            {'seed':a.seed,'popsize':a.population,'bounds':[0,1],'verbose':-9})
    best_score=-float('inf');best_x=None;best_metrics=None;history=[]
    for generation in range(1,a.generations+1):
        ys=es.ask();losses=[]
        for y in ys:
            x=decode(y)
            score,metrics,_=evaluate(x,cfg,duration=a.duration,min_flight=a.min_flight)
            losses.append(-score)
            if score>best_score:
                best_score=score;best_x=x.copy();best_metrics=metrics
        es.tell(ys,losses)
        history.append([generation,best_score,best_metrics.get('com_rise_m',0),int(best_metrics.get('valid_jump',False))])
        print(f'Generation {generation:3d}: score={best_score:.5f} COM rise={best_metrics.get("com_rise_m",0):.4f}m valid={best_metrics.get("valid_jump",False)} ratios={best_x[:2].round(3)}',flush=True)
    _,final,rows=evaluate(best_x,cfg,duration=a.duration,min_flight=a.min_flight,record=True)
    result={'hip_ratio':float(best_x[0]),'knee_ratio':float(best_x[1]),
            'hip_Kp':float(best_x[2]),'hip_Kd':float(best_x[3]),
            'knee_Kp':float(best_x[4]),'knee_Kd':float(best_x[5]),
            'metrics':final,'motor_parameters':cfg,
            'model_status':'Exploratory gearbox and actuator assumptions; not hardware validated'}
    (out/'best.json').write_text(json.dumps(result,indent=2))
    for name,columns,values in [('trajectory.csv',CSV_COLUMNS,rows),
          ('convergence.csv',['generation','best_score','best_com_rise_m','valid_jump'],history)]:
        with (out/name).open('w',newline='') as f:
            w=csv.writer(f);w.writerow(columns);w.writerows(values)
    print(json.dumps(result,indent=2))
    print('Saved best.json, trajectory.csv, convergence.csv in',out)
if __name__=='__main__':main()
