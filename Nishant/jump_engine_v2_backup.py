"""Two-link vertical monoped with configurable *assumed* gearbox properties.
Not a validated MAD M6C12 or gearbox design. Uses MuJoCo joint torque actuators.
"""
import math
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parent
CROUCH = np.array([0.6, -1.2], dtype=float)
CSV_COLUMNS = ['time_s','hip_z_m','com_z_m','hip_vz_m_s','hip_q_rad','knee_q_rad',
               'hip_target_rad','knee_target_rad','hip_torque_Nm','knee_torque_Nm',
               'hip_current_A','knee_current_A','foot_contact','ground_normal_force_N','phase',
               'hip_joint_speed_rad_s','knee_joint_speed_rad_s','hip_motor_voltage_V',
               'knee_motor_voltage_V','hip_motor_power_W','knee_motor_power_W']


def evaluate(x, cfg, duration=1.4, hold=.10, push=.16, min_flight=.08, record=False):
    nh,nk,kph,kdh,kpk,kdk = np.asarray(x, dtype=float)
    ratios=np.array([nh,nk]); kp=np.array([kph,kpk]); kd=np.array([kdh,kdk])
    model=mujoco.MjModel.from_xml_path(str(ROOT/'monoped.xml'))
    data=mujoco.MjData(model)
    foot_id=model.geom('foot').id; floor_id=model.geom('floor').id
    carriage_id=model.body('carriage').id
    # Explicit assumptions: each gearbox is mounted on the carriage, and its
    # output inertia is added at the corresponding joint. Rotor inertia is
    # reflected through the reduction. No detailed transmission mechanics.
    gm0=float(cfg.get('gearbox_mass_base_kg',.08))
    gm_per=float(cfg.get('gearbox_mass_per_ratio_kg',.004))
    gb_mass=gm0+gm_per*ratios
    model.body_mass[carriage_id] += float(gb_mass.sum())
    rotor_J=float(cfg.get('motor_rotor_inertia_kgm2',2e-5))
    gb_J=float(cfg.get('gearbox_output_inertia_kgm2',1e-4))
    for joint,ratio in zip(('hip','knee'),ratios):
        dof=model.jnt_dofadr[model.joint(joint).id]
        model.dof_armature[dof] += rotor_J*ratio**2+gb_J
    # Conservative proxy: reduction efficiency decreases with ratio.
    eta0=float(cfg.get('gearbox_efficiency',.9))
    eta_slope=float(cfg.get('gearbox_efficiency_drop_per_ratio',.002))
    eta=np.clip(eta0-eta_slope*(ratios-4),.5,1.)
    torque_capacity=float(cfg.get('gearbox_output_torque_limit_Nm',60.))
    # Recompute MuJoCo mass matrices and constants after model changes.
    mujoco.mj_setConst(model,data)
    radius=float(model.geom_size[foot_id,0])
    initial_hip=.2*math.cos(CROUCH[0])+.2*math.cos(CROUCH.sum())+radius-.001
    data.qpos[:]=[initial_hip-.36,*CROUCH];data.qvel[:]=0
    mujoco.mj_forward(model,data)
    def contact_ids():
        return [j for j in range(data.ncon) if {int(data.contact[j].geom1),int(data.contact[j].geom2)}=={foot_id,floor_id}]
    if not contact_ids():
        raise RuntimeError('Initial foot contact missing; check monoped.xml.')
    def com_z():
        return float(np.dot(model.body_mass[1:],data.xipos[1:,2])/np.sum(model.body_mass[1:]))
    def grf_now():
        total=0.
        for j in contact_ids():
            f=np.zeros(6);mujoco.mj_contactForce(model,data,j,f)
            total+=max(0.,float(f[0]))
        return total
    dt=float(model.opt.timestep)
    off_steps=0;on_steps=0;flight=None;complete=None;flights=[];phase='stance';rows=[]
    max_current=0.; energy=0.; max_grf=0.;max_hip=initial_hip
    max_com=com_z();limit_duration=0.;contact_count=0
    kt=float(cfg['kt_Nm_per_A']);R=float(cfg['resistance_ohm'])
    bus=float(cfg['bus_voltage_V']);imax=float(cfg['max_current_A'])
    max_speed=float(cfg['max_speed_rad_s'])
    for step in range(round(duration/dt)):
        t=step*dt
        if phase=='landed':target=CROUCH.copy()
        elif t<hold:target=CROUCH.copy()
        elif t<hold+push:
            s=(t-hold)/push;target=CROUCH+s*(np.array([.05,-.10])-CROUCH)
        else:target=np.array([.35,-.70])
        q=data.qpos[1:3].copy();dq=data.qvel[1:3].copy()
        requested=kp*(target-q)-kd*dq
        motor_omega=ratios*dq
        # Signed back-EMF / winding-voltage constraint:
        # -bus <= R*I + kt*omega <= bus, and -imax <= I <= imax.
        low=np.maximum(-imax,(-bus-kt*motor_omega)/R)
        high=np.minimum(imax,(bus-kt*motor_omega)/R)
        # For states above rated no-load speed, the feasible current interval
        # may not include zero. A real drive can brake/regenerate; the model
        # allows the physically consistent current interval.
        low=np.minimum(low,high)
        current_request=requested/(ratios*eta*kt)
        signed_current=np.clip(current_request,low,high)
        # Approximate bidirectional efficiency: same eta both directions.
        torque=np.clip(ratios*eta*kt*signed_current,-torque_capacity,torque_capacity)
        signed_current=torque/(ratios*eta*kt)
        voltage=R*signed_current+kt*motor_omega
        data.ctrl[:]=torque
        max_current=max(max_current,float(np.max(np.abs(signed_current))))
        motor_power=voltage*signed_current
        energy+=float(np.sum(np.maximum(motor_power,0))*dt)
        mujoco.mj_step(model,data);mujoco.mj_forward(model,data)
        contact=bool(contact_ids());contact_count+=int(contact)
        grf=grf_now();max_grf=max(max_grf,grf)
        z=float(data.xpos[carriage_id,2]);vz=float(data.qvel[0]);cz=com_z()
        max_hip=max(max_hip,z);max_com=max(max_com,cz)
        if not np.all(np.isfinite(data.qpos)) or not np.isfinite(grf):
            return -1e3,{'failure':'nonfinite state'},rows
        if abs(data.qpos[1])>1.09 or data.qpos[2]<-1.79:limit_duration+=dt
        if not contact:off_steps+=1;on_steps=0
        else:on_steps+=1;off_steps=0
        if phase=='stance' and off_steps>=5 and t>=hold and vz>.05:
            phase='flight';first=t-(off_steps-1)*dt
            flight={'takeoff_time_s':round(first,6),
                    'takeoff_hip_m':z-vz*(off_steps-1)*dt,
                    'takeoff_com_m':cz-vz*(off_steps-1)*dt,
                    'takeoff_vz_m_s':vz,'apex_hip_m':z,'apex_com_m':cz}
        if phase=='flight':
            flight['apex_hip_m']=max(flight['apex_hip_m'],z)
            flight['apex_com_m']=max(flight['apex_com_m'],cz)
            if on_steps>=5:
                landing=t-(on_steps-1)*dt
                flight['landing_time_s']=round(landing,6)
                flight['flight_duration_s']=float(landing-flight['takeoff_time_s'])
                flight['hip_rise_m']=max(0.,flight['apex_hip_m']-flight['takeoff_hip_m'])
                flight['com_rise_m']=max(0.,flight['apex_com_m']-flight['takeoff_com_m'])
                flight['qualified']=bool(flight['flight_duration_s']>=min_flight)
                flights.append(flight)
                if flight['qualified']:complete=flight.copy();phase='landed'
                else:phase='stance'
                flight=None
        if record:
            rows.append([round(t+dt,6),z,cz,vz,float(data.qpos[1]),float(data.qpos[2]),
                         *map(float,target),*map(float,torque),*map(float,np.abs(signed_current)),
                         int(contact),grf,phase,*map(float,dq),*map(float,voltage),*map(float,motor_power)])
    if flight is not None:
        flight['incomplete']=True;flights.append(flight)
    valid=complete is not None;rise=complete['com_rise_m'] if valid else 0.
    score=rise-.00005*energy-limit_duration-(0.3 if not valid else 0.)
    metrics={'valid_jump':valid,'selected_flight':complete,'flight_events':flights,
             'com_rise_m':rise,'hip_rise_m':complete['hip_rise_m'] if valid else 0.,
             'max_hip_m':max_hip,'max_com_m':max_com,'initial_hip_m':initial_hip,
             'peak_ground_normal_force_N':max_grf,'peak_current_A':max_current,
             'electrical_energy_proxy_J':energy,'joint_limit_duration_s':limit_duration,
             'contact_steps':contact_count,'score':score,
             'gearbox_mass_kg_each':gb_mass.tolist(),
             'gearbox_efficiency_each':eta.tolist(),
             'gearbox_torque_limit_Nm':torque_capacity,
             'model_status':'Assumed gearbox mass/inertia/efficiency; unvalidated rotor inertia and transmission'}
    return score,metrics,rows
