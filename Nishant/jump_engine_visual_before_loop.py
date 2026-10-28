"""Shared MuJoCo evaluation for a two-link slider-constrained monoped.
Motor parameters are placeholders, NOT verified MAD MC12 specifications.
"""
import math
import numpy as np
import mujoco
import mujoco.viewer
import time
from pathlib import Path
ROOT = Path(__file__).resolve().parent
CROUCH = np.array([0.6, -1.2], dtype=float)
CSV_COLUMNS = ['time_s','hip_z_m','com_z_m','hip_vz_m_s','hip_q_rad','knee_q_rad',
               'hip_target_rad','knee_target_rad','hip_torque_Nm','knee_torque_Nm',
               'hip_current_A','knee_current_A','foot_contact','ground_normal_force_N','phase']

def evaluate(x, cfg, duration=1.4, hold=0.10, push=0.16, min_flight=0.08, record=False):
    nh,nk,kph,kdh,kpk,kdk = np.asarray(x,dtype=float)
    ratios=np.array([nh,nk]);kp=np.array([kph,kpk]);kd=np.array([kdh,kdk])
    model=mujoco.MjModel.from_xml_path(str(ROOT/'monoped.xml'))
    # Approximate hip-mounted actuator mass update. MuJoCo stores compiled
    # body masses; MjModel does not expose geom_mass.
    if 'actuator_masses_kg' in cfg:
        carriage = model.body('carriage').id
        delta = sum(float(v) for v in cfg['actuator_masses_kg']) - 0.40
        model.body_mass[carriage] += delta
        if model.body_mass[carriage] <= 0:
            raise ValueError('Invalid carriage mass after COMPAct update')
        model.body_inertia[carriage, 0] += delta * 0.055**2
        model.body_inertia[carriage, 2] += delta * 0.055**2
    data=mujoco.MjData(model)
    foot_id=model.geom('foot').id; floor_id=model.geom('floor').id
    carriage_id=model.body('carriage').id
    foot_radius=float(model.geom_size[foot_id,0])
    initial_hip=0.2*math.cos(CROUCH[0])+0.2*math.cos(CROUCH.sum())+foot_radius-0.001
    data.qpos[:]=[initial_hip-0.36,*CROUCH]
    data.qvel[:]=0
    mujoco.mj_forward(model,data)
    def foot_contact_indices():
        return [j for j in range(data.ncon) if {int(data.contact[j].geom1),int(data.contact[j].geom2)}=={foot_id,floor_id}]
    if not foot_contact_indices():
        raise RuntimeError('Initial foot contact missing; verify monoped.xml slider range and crouch.')
    def com_z():
        return float(np.dot(model.body_mass[1:],data.xipos[1:,2])/sum(model.body_mass[1:]))
    def contact_normal_force():
        total=0.
        for j in foot_contact_indices():
            f=np.zeros(6)
            mujoco.mj_contactForce(model,data,j,f)
            total+=max(0.,float(f[0]))
        return total
    dt=float(model.opt.timestep)
    contact_prev=True; off_steps=0; on_steps=0; flight=None; complete=None
    flights=[]; phase='stance'; rows=[]
    max_current=0.; energy_proxy=0.; max_grf=0.; max_hip=initial_hip; peak_motor_speed=0.; overspeed_time=0.
    first_air_time=None; max_com=float(com_z()); limit_duration=0.; contact_count=0
    viewer = mujoco.viewer.launch_passive(model, data)
    viewer.cam.distance = 1.5
    viewer.cam.elevation = -15

    for step in range(round(duration/dt)):
        t=step*dt
        if phase=='landed':
            target=np.array([0.6,-1.2])
        elif t<hold:
            target=CROUCH.copy()
        elif t<hold+push:
            s=(t-hold)/push
            target=CROUCH+s*(np.array([0.05,-0.10])-CROUCH)
        else:
            target=np.array([0.35,-0.70])
        q=data.qpos[1:3].copy();dq=data.qvel[1:3].copy()
        requested=kp*(target-q)-kd*dq
        kt=float(cfg['kt_Nm_per_A']);R=float(cfg['resistance_ohm'])
        eta=np.asarray(cfg.get('gearbox_efficiencies', [cfg['gearbox_efficiency']]*2), dtype=float); vmax=float(cfg['bus_voltage_V'])
        omega=np.abs(dq*ratios)
        avail=np.minimum(float(cfg['max_current_A']),np.maximum(0.,(vmax-kt*omega)/max(R,1e-9)))
        peak_motor_speed=max(peak_motor_speed,float(omega.max()))
        overspeed_time+=dt*float(np.any(omega>float(cfg['max_speed_rad_s'])))
        avail=np.where(omega<=float(cfg['max_speed_rad_s']),avail,0.)
        cap=ratios*eta*kt*avail
        torque=np.clip(requested,-cap,cap)
        data.ctrl[:]=torque
        current=np.abs(torque)/(ratios*eta*max(kt,1e-9))
        max_current=max(max_current,float(current.max()))
        # A positive work + resistive loss PROXY, not verified battery energy.
        energy_proxy+=float(np.sum(current**2*R+np.maximum(torque*dq,0.)/np.maximum(eta,1e-9))*dt)
        mujoco.mj_step(model,data)
        # Recompute contacts and constraint forces for the NEW state. mj_step's
        # contact forces otherwise correspond to the previous integration state.
        mujoco.mj_forward(model,data)
        if viewer.is_running():
            viewer.sync()
            time.sleep(dt)
        contact=bool(foot_contact_indices());contact_count+=int(contact)
        grf=contact_normal_force()
        max_grf=max(max_grf,grf)
        z=float(data.xpos[carriage_id,2]);vz=float(data.qvel[0]);cz=com_z()
        max_hip=max(max_hip,z);max_com=max(max_com,cz)
        if not np.all(np.isfinite(data.qpos)) or not np.isfinite(grf):
            return -1e3,{'failure':'nonfinite state'},rows
        if abs(data.qpos[1])>1.09 or data.qpos[2]<-1.79:limit_duration+=dt
        if not contact:
            off_steps+=1;on_steps=0
        else:
            on_steps+=1;off_steps=0
        # 5 ms contact-loss debounce; use first missing-contact state as takeoff.
        if phase=='stance' and off_steps>=5 and t>=hold and vz>0.05:
            phase='flight';first_air_time=t-(off_steps-1)*dt
            flight={'takeoff_time_s':round(first_air_time,6),'takeoff_hip_m':z-vz*(off_steps-1)*dt,
                    'takeoff_com_m':cz-vz*(off_steps-1)*dt,'takeoff_vz_m_s':vz,
                    'apex_hip_m':z,'apex_com_m':cz}
        if phase=='flight':
            flight['apex_hip_m']=max(flight['apex_hip_m'],z)
            flight['apex_com_m']=max(flight['apex_com_m'],cz)
            if on_steps>=5:
                landing_time=t-(on_steps-1)*dt
                flight['landing_time_s']=round(landing_time,6)
                flight['flight_duration_s']=float(landing_time-flight['takeoff_time_s'])
                flight['hip_rise_m']=max(0.,flight['apex_hip_m']-flight['takeoff_hip_m'])
                flight['com_rise_m']=max(0.,flight['apex_com_m']-flight['takeoff_com_m'])
                flight['qualified']=bool(flight['flight_duration_s']>=min_flight)
                flights.append(flight)
                if flight['qualified']:
                    complete=flight.copy();phase='landed'
                else:
                    phase='stance'
                flight=None
        if record and step%10==0:
            rows.append([round(t+dt,6),z,cz,vz,float(data.qpos[1]),float(data.qpos[2]),
                         float(target[0]),float(target[1]),float(torque[0]),float(torque[1]),
                         float(current[0]),float(current[1]),int(contact),grf,phase])
        contact_prev=contact
    if flight is not None:
        flight['incomplete']=True
        flights.append(flight)
    qualified=complete is not None
    rise=complete['com_rise_m'] if qualified else 0.
    # Positive height is the primary objective; avoid reward for incomplete flights.
    score=rise-0.00005*energy_proxy-1.0*limit_duration-0.5*overspeed_time-(0.3 if not qualified else 0.)
    metrics={'valid_jump':qualified,'selected_flight':complete,'flight_events':flights,
             'com_rise_m':rise,'hip_rise_m':complete['hip_rise_m'] if qualified else 0.,
             'max_hip_m':max_hip,'max_com_m':max_com,'initial_hip_m':initial_hip,
             'peak_ground_normal_force_N':max_grf,'peak_current_A':max_current,
             'energy_proxy_J':energy_proxy,'joint_limit_duration_s':limit_duration,
             'contact_steps':contact_count,'score':score, 'peak_motor_speed_rad_s':peak_motor_speed, 'overspeed_time_s':overspeed_time,
             'motor_model':'UNVERIFIED placeholder motor constants; NOT MAD MC12 validated'}
    print("\nSimulation complete.")
    print("Close the MuJoCo window to exit.")

    while viewer.is_running():
        viewer.sync()
        time.sleep(0.05)

    viewer.close()

    return score,metrics,rows
