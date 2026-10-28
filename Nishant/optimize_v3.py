"""CMA-ES gear ratio and PD co-design for jump_engine_v3."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import cma
from jump_engine_v3 import evaluate,CSV_COLUMNS
ROOT=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--generations',type=int,default=75)
    p.add_argument('--population',type=int,default=20)
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--duration',type=float,default=1.4)
    p.add_argument('--min-flight',type=float,default=.08)
    p.add_argument('--motor-config',default='motor_config_v2.json')
    p.add_argument('--output',default='results/v3')
    p.add_argument('--height-only',action='store_true',help='Maximize jump height instead of energy-adjusted score')
    p.add_argument('--reject-speed-exceedance',action='store_true',help='Penalize any samples above configured speed threshold')
    a=p.parse_args()
    if a.generations<1 or a.population<4:p.error('generations>=1 and population>=4 required')
    cfg=json.loads((ROOT/a.motor_config).read_text())
    out=ROOT/a.output;out.mkdir(parents=True,exist_ok=True)
    lo=np.array([4,4,20,.1,20,.1],float)
    hi=np.array([25,25,900,30,900,30],float)
    def decode(y):return lo+np.clip(y,0,1)*(hi-lo)
    es=cma.CMAEvolutionStrategy(np.full(6,.5),.22,
        {'seed':a.seed,'popsize':a.population,'bounds':[0,1],'verbose':-9})
    best_obj=-float('inf');best_x=None;best_metrics=None
    best_height=-float('inf');best_height_x=None;best_height_metrics=None
    history=[]
    for gen in range(1,a.generations+1):
        ys=es.ask();losses=[]
        for y in ys:
            x=decode(y)
            score,m,_=evaluate(x,cfg,duration=a.duration,min_flight=a.min_flight)
            speed_violation=sum(m.get('speed_exceeded_steps_each',[]))
            # This is a soft penalty, not a claim of hardware-safe speed.
            penalty=.0002*speed_violation if a.reject_speed_exceedance else 0.
            obj=(m['com_rise_m'] if a.height_only else score)-penalty
            losses.append(-obj)
            if obj>best_obj:
                best_obj=obj;best_x=x.copy();best_metrics=m
            if m['valid_jump'] and m['com_rise_m']>best_height:
                best_height=m['com_rise_m'];best_height_x=x.copy();best_height_metrics=m
        es.tell(ys,losses)
        history.append([gen,best_obj,best_metrics['com_rise_m'],
                        best_height if np.isfinite(best_height) else 0,
                        int(best_metrics['valid_jump'])])
        print(f'Generation {gen:3d}: objective={best_obj:.5f} best score-design rise={best_metrics["com_rise_m"]:.4f}m max rise={max(0.,best_height):.4f}m ratios={best_x[:2].round(3)}',flush=True)

    def save_design(name,x):
        if x is None:return
        score,metrics,rows=evaluate(x,cfg,duration=a.duration,min_flight=a.min_flight,record=True)
        result={'hip_ratio':float(x[0]),'knee_ratio':float(x[1]),
                'hip_Kp':float(x[2]),'hip_Kd':float(x[3]),
                'knee_Kp':float(x[4]),'knee_Kd':float(x[5]),
                'metrics':metrics,'motor_parameters':cfg,
                'model_status':'Exploratory V3; not hardware validated'}
        (out/f'{name}.json').write_text(json.dumps(result,indent=2))
        with (out/f'{name}_trajectory.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(CSV_COLUMNS);w.writerows(rows)
        print(f'Saved {name}.json and {name}_trajectory.csv')
    save_design('best_score',best_x)
    save_design('best_height',best_height_x)
    with (out/'convergence.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['generation','best_objective','best_score_design_rise_m','best_height_m','valid_score_design']);w.writerows(history)
    print('Saved convergence.csv in',out)

if __name__=='__main__':main()
