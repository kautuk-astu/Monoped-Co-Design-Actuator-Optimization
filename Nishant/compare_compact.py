"""Compare four fixed COMPAct gearbox candidates with CMA-ES PD tuning.
Run: python3 compare_compact.py --generations 25 --population 12
The electrical motor constants remain unverified placeholders.
"""
import argparse, csv, json
from pathlib import Path
import numpy as np
import cma
from jump_engine import evaluate, CSV_COLUMNS

ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--generations',type=int,default=25)
    parser.add_argument('--population',type=int,default=12)
    parser.add_argument('--duration',type=float,default=1.4)
    parser.add_argument('--seed',type=int,default=42)
    args=parser.parse_args()
    if args.generations<1 or args.population<4: parser.error('generations >=1, population >=4 required')
    motor=json.loads((ROOT/'motor_config.json').read_text())
    candidates=sorted((ROOT/'compact_candidates').glob('?_*.json'))
    if len(candidates)!=4: raise SystemExit('Expected four candidate JSON files in compact_candidates/')
    out=ROOT/'results_compact';out.mkdir(exist_ok=True)
    lo=np.array([20.,0.1,20.,0.1]);hi=np.array([900.,30.,900.,30.])
    summary=[]
    for i,path in enumerate(candidates):
        candidate=json.loads(path.read_text())
        hip=candidate['actuators']['hip'];knee=candidate['actuators']['knee']
        ratios=[hip['gear_ratio'],knee['gear_ratio']]
        cfg=dict(motor,gearbox_efficiencies=[hip['efficiency'],knee['efficiency']],
                 actuator_masses_kg=[hip['actuator_mass_kg'],knee['actuator_mass_kg']])
        def decode(y):return lo+np.clip(y,0,1)*(hi-lo)
        es=cma.CMAEvolutionStrategy(np.full(4,.5),.22,{'seed':args.seed+i,'popsize':args.population,'bounds':[0,1],'verbose':-9})
        best=-float('inf');best_x=None;history=[]
        print('\nEvaluating',path.stem,flush=True)
        for generation in range(1,args.generations+1):
            ys=es.ask();losses=[]
            for y in ys:
                x=np.r_[ratios,decode(y)]
                score,metrics,_=evaluate(x,cfg,duration=args.duration)
                losses.append(-score)
                if score>best:
                    best=score;best_x=x.copy()
            es.tell(ys,losses)
            history.append([generation,best])
            print(f'  generation {generation}/{args.generations}: best score {best:.5f}',flush=True)
        score,metrics,rows=evaluate(best_x,cfg,duration=args.duration,record=True)
        result={'candidate':path.stem,'ratios':ratios,'pd_gains':{'hip_Kp':float(best_x[2]),'hip_Kd':float(best_x[3]),'knee_Kp':float(best_x[4]),'knee_Kd':float(best_x[5])},'metrics':metrics,'actuator_masses_kg':cfg['actuator_masses_kg'],'efficiencies':cfg['gearbox_efficiencies'],'motor_constants_verified':False}
        (out/f'{path.stem}_best.json').write_text(json.dumps(result,indent=2))
        for filename,header,content in [(f'{path.stem}_trajectory.csv',CSV_COLUMNS,rows),(f'{path.stem}_convergence.csv',['generation','best_score'],history)]:
            with (out/filename).open('w',newline='') as f:
                writer=csv.writer(f);writer.writerow(header);writer.writerows(content)
        summary.append({'candidate':path.stem,'hip_ratio':ratios[0],'knee_ratio':ratios[1],'actuator_mass_kg':sum(cfg['actuator_masses_kg']),'valid_jump':metrics['valid_jump'],'com_rise_m':metrics['com_rise_m'],'hip_rise_m':metrics['hip_rise_m'],'energy_proxy_J':metrics['energy_proxy_J'],'peak_current_A':metrics['peak_current_A'],'peak_motor_speed_rad_s':metrics['peak_motor_speed_rad_s'],'overspeed_time_s':metrics['overspeed_time_s'],'score':score})
    with (out/'comparison.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(summary[0]));writer.writeheader();writer.writerows(summary)
    print('\nCOMPARISON (research model; unverified motor constants)')
    for r in sorted(summary,key=lambda x:x['score'],reverse=True):
        print(f"{r['candidate']:15s} COM rise={r['com_rise_m']:.4f} m, valid={r['valid_jump']}, overspeed={r['overspeed_time_s']:.3f} s")
    print('Saved:',out/'comparison.csv')

if __name__=='__main__':main()
