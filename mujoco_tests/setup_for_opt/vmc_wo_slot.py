import mujoco as mj
import numpy as np
import time
import xml.etree.ElementTree as ET

class Controller:
    def __init__(self, xml_path, m, d, ori_l = 10.0, ori_theta=0):
        self.xml_path = xml_path
        #m = mj.MjModel.from_xml_path(xml_path)
        #d = mj.MjData(m)
        self.m = m
        self.d = d
        self.ori_l = ori_l
        self.ori_theta = ori_theta
        # Get the joint index for the slider joint
        base_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "rootz")
        self.base_joint_id = base_joint_id
        self.base_force_index = m.jnt_dofadr[base_joint_id]
        # Get the joint index for the thigh/hip joint
        thigh_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "thigh_joint")
        self.thigh_joint_id = thigh_joint_id
        self.hip_force_index = m.jnt_dofadr[thigh_joint_id]
        # Get the joint index for the knee joint
        knee_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "leg_joint")
        self.knee_joint_id = knee_joint_id
        self.knee_force_index = m.jnt_dofadr[knee_joint_id]

    def test_init(self):
        print("base_joint_id:", self.base_joint_id)
        print("thigh_joint_id:", self.thigh_joint_id)
        print("knee_joint_id:", self.knee_joint_id)
        print("base_force_index:", self.base_force_index)
        print("hip_force_index:", self.hip_force_index)
        print("knee_force_index:", self.knee_force_index)

    def get_ground_contact_forces(self):

        """
  Extracts ground contact forces from MuJoCo simulation data.

  Args:
    model: MuJoCo model object.
    data: MuJoCo data object.

  Returns:
    A list of contact forces, where each element is a tuple 
    containing the contact force vector and the corresponding 
    body index.
  """
        xml_path = self.xml_path
        #m = mj.MjModel.from_xml_path(xml_path)
        #d = mj.MjData(m)
        m = self.m
        d = self.d
        ground_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_GEOM, "floor")  # Assuming "slot" is the ground geom
        ground_contact_forces = []
        for i in range(d.ncon):  # ncon is the number of contacts
            con = d.contact[i]   # contact[i] is the i-th contact
            if con.geom1 == ground_id or con.geom2 == ground_id:
        # Calculate the contact force vector
                force = np.zeros(6, dtype=np.float64)
                mj.mj_contactForce(m, d, i, force) 

        # Extract force (first 3 elements)
                contact_force = force[:3]

        # Get the body index 
                body_id = con.geom1 if con.geom1 != ground_id else con.geom2

                ground_contact_forces.append((contact_force, body_id))

        return ground_contact_forces
    
    def print_contact_force(self):
        contact_force = self.get_ground_contact_forces()
        for force, body_id in contact_force:
            print(f"Contact force on body {body_id}: {force}")
        return force 
    
    def body_names(self):
        # Get the number of bodies in the model
        num_bodies = self.m.nbody
        # Get the names of all bodies    
        #get body names usig bodyname2id
        body_names = [mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_BODY, i) for i in range(num_bodies)]
        return body_names
    
    #to check for order of different bodies in array of bodies
    
    def body_dict(self):
        body_names = self.body_names()
        # get body index for bodies and make a dictionary
        body_id = {body_names[i]: i for i in range(len(body_names))}
        return body_id
    
    #following functions are to obtain velocity of the leg link in world frame 

    def body_cvel(self):
        body_names = self.body_names()
        body_id = self.body_dict()
        # make a list of linear velocities of all bodies
        body_velocity = [self.d.cvel[body_id[body_name]]for body_name in body_names]
        return body_velocity
    
    def leg_cvel(self):
        #TODO: check the name for leg link using body_names    
        body_id = self.body_dict()
        # make a list of linear velocities of all bodies
        body_velocity = self.body_cvel()
        # get the index of the leg link from dictionary of body_id
        leg_index = body_id['leg']
        # get the linear velocity of the leg link
        leg_velocity = body_velocity[leg_index]
        return leg_velocity
    
    #following functions are to obtain position of the leg link in world frame

    #function to obtain thigh joint position
    def thigh_joint_position(self):
        # get the index of thigh joint
        thigh_joint_id = self.thigh_joint_id
        # get the position of thigh joint
        thigh_joint_position = self.d.qpos[thigh_joint_id]
        return thigh_joint_position
    
    def rotmat_thigh(self):
        # get the position of thigh joint
        thigh_position = self.thigh_position()
        # get the rotation matrix of thigh joint
        rotmat_thigh = np.array([[np.cos(thigh_position), -np.sin(thigh_position)], [np.sin(thigh_position), np.cos(thigh_position)]])
        return rotmat_thigh
    
    def leg_world_velocity(self):
        # get the linear velocity of the leg link
        leg_velocity = self.leg_cvel()
        leg_xz_velocity = np.array([leg_velocity[0], leg_velocity[2]])
        # get the rotation matrix of thigh joint
        rotmat_thigh = self.rotmat_thigh()
        # get the linear velocity of the leg link in world frame
        leg_world_velocity = np.dot(rotmat_thigh, leg_xz_velocity)
        return leg_world_velocity  
    
    #function to obtain base linear velocity in world frame
    def base_cvel(self):
        # get the linear velocity of the base link
        body_id = self.body_dict()
        base_index = body_id['torso']
        base_velocity = self.d.cvel[base_index]
        base_z_velocity = base_velocity[2]
        print("base_index", base_index) #gives 2 as the index as world/slot are 0/1
        return base_z_velocity
    
    #function to obtain base jpint velocity (rootz)
    def rootz_velocity(self):
        # get the index of base joint
        base_joint_id = self.base_joint_id
        # get the position of base joint
        base_velocity= self.d.qvel[base_joint_id]
        return base_velocity
    
    #function to obtain number of joints in the model
    def num_joints(self):
        num_joints = self.m.njnt
        return num_joints

    #function to obtain dictionary of names of joints in the model
    # get the names of all joints
    def joint_names(self):
        num_joints = self.num_joints()
        #make a dictionary of joint names
        joint_names = {mj.mj_id2name(self.m, mj.mjtObj.mjOBJ_JOINT, i): i for i in range(num_joints)}
        return joint_names       
        
      
    def lin_sd_var(self):
        #TODO: CHECK FOR LOGIC AT REACHING GROUND AGAIN 
        base_velocity = self.base_cvel()
        if base_velocity>= 0:
            lin_kp = 1800
            lin_kd = 0
        elif base_velocity<0:
            lin_kp = 0
            lin_kd = 50
        return lin_kp, lin_kd
    
    def rot_sd_var(self):
        #TODO: CHECK FOR LOGIC AT REACHING GROUND AGAIN 
        base_velocity = self.base_cvel()
        if base_velocity>= 0:
            rot_kp = 0.2
            rot_kd = 0
        elif base_velocity<0:
            rot_kp = 0.01
            rot_kd = 50
        return rot_kp, rot_kd
    
    def rootz_position(self):
        # get the index of base joint
        base_joint_id = self.base_joint_id
        # get the position of base joint
        rootz_position = self.d.qpos[base_joint_id]
        return rootz_position

    def base_inertial_pos(self):
        body_id = self.body_dict()
        torso_index = body_id['torso']
        base_inertial_pos = self.d.xipos[torso_index]
        return base_inertial_pos 

    def hip_position(self):
        thigh_joint_id = self.thigh_joint_id
        hip_position = self.d.qpos[thigh_joint_id]
        return hip_position     

    def torso_bottom_position(self):
        rootz_position = self.rootz_position()
        torso_z_position = rootz_position - 0.05
        torso_x_position = 0
        torso_y_position = 0
        torso_bottom_position = np.array([torso_x_position, torso_y_position, torso_z_position])
        return torso_bottom_position
    
    def knee_position(self):
        knee_joint_id = self.knee_joint_id
        knee_position = self.d.qpos[knee_joint_id]
        return knee_position
    
    def xml_parser(self):
        xml = self.xml_path
        tree = ET.parse(xml)
        root = tree.getroot()
        return root, tree    
    
    def thigh_length_found(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):
            print("worldbody found")
            for slot in link.findall('body'):
                print("body found")
                if slot.get('name')== 'slot':
                    print("slot found")
                    for new in slot.findall('body'):
                        if new.get('name') == 'torso':
                            print("torso found")
                            for element in new.findall('body'):
                                if element.get('name') == 'thigh':
                                    print("thigh found")
                                    for ele in element.findall('geom'):
                                        print("geom found")
                                        if ele.get('name')=='thigh_geom':
                                            print("thigh_geom found")
                                            fromto = ele.get('fromto')
                                            print("fromto:", fromto) 
                                            thigh_start = float(fromto.split()[2])
                                            print("thigh_start:", thigh_start)
                                            thigh_end = float(fromto.split()[5])    
                                            print("thigh_end:", thigh_end)
                                            thigh_length = thigh_end - thigh_start
                                            print("thigh_length:", thigh_length)                                       
                                            return thigh_length
                       
        return thigh_length
    
    def thigh_length_xml(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):            
                                
                    for new in link.findall('body'):
                        if new.get('name') == 'torso':                           
                            for element in new.findall('body'):
                                if element.get('name') == 'thigh':                                    
                                    for ele in element.findall('geom'):                                       
                                        if ele.get('name')=='thigh_geom':                                            
                                            fromto = ele.get('fromto')                                            
                                            thigh_start_z = float(fromto.split()[2])
                                             
                                            thigh_start_x = float(fromto.split()[0])
                                            
                                            thigh_start_y = float(fromto.split()[1])
                                                                                      
                                            thigh_end_z = float(fromto.split()[5])    
                                            
                                            thigh_end_x = float(fromto.split()[3])
                                            
                                            thigh_end_y = float(fromto.split()[4])
                                            
                                            thigh_length = np.linalg.norm(np.array([thigh_end_x, thigh_end_y, thigh_end_z]) - np.array([thigh_start_x, thigh_start_y, thigh_start_z]))
                                                                                 
                                            return thigh_length
                       
        return thigh_length
    
    def leg_length_found(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):
            print("worldbody found")
            for slot in link.findall('body'):
                print("body found")
                if slot.get('name')== 'slot':
                    print("slot found")
                    for new in slot.findall('body'):
                        if new.get('name') == 'torso':
                            print("torso found")
                            for element in new.findall('body'):
                                if element.get('name') == 'thigh':
                                    print("thigh found")
                                    for ele in element.findall('body'):
                                        if ele.get('name') == 'leg': 
                                            print("leg found")
                                            for legs in ele.findall('geom'):
                                                print("geom found")
                                                fromto = legs.get('fromto')
                                                print("fromto:", fromto)
                                                leg_start = float(fromto.split()[2])
                                                print("leg_start:", leg_start)
                                                leg_end = float(fromto.split()[5])
                                                print("leg_end:", leg_end)
                                                leg_length = leg_end - leg_start
                                                print("leg_length:", leg_length)
                                                return leg_length                                                
                                   
        return leg_length
    
    def leg_length_xml(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):            
                              
                    for new in link.findall('body'):
                        if new.get('name') == 'torso':                            
                            for element in new.findall('body'):
                                if element.get('name') == 'thigh':                                    
                                    for ele in element.findall('body'):
                                        if ele.get('name') == 'leg':                                             
                                            for legs in ele.findall('geom'):                                                
                                                fromto = legs.get('fromto')                                                
                                                leg_start_z = float(fromto.split()[2])     
                                                leg_start_x = float(fromto.split()[0])
                                                
                                                leg_start_y = float(fromto.split()[1])
                                                                                       
                                                leg_end_z = float(fromto.split()[5])
                                                
                                                leg_end_x = float(fromto.split()[3])
                                                
                                                leg_end_y = float(fromto.split()[4])
                                                
                                                leg_length = np.linalg.norm(np.array([leg_end_x, leg_end_y, leg_end_z]) - np.array([leg_start_x, leg_start_y, leg_start_z]))
                                                
                                                return leg_length                                                
                                   
        return leg_length
    
    def leg_width_found(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):
            print("worldbody found")
            for slot in link.findall('body'):
                print("body found")
                if slot.get('name')== 'slot':
                    print("slot found")
                    for new in slot.findall('body'):
                        if new.get('name') == 'torso':
                            print("torso found")
                            for element in new.findall('body'):
                                if element.get('name') == 'thigh':
                                    print("thigh found")
                                    for ele in element.findall('body'):
                                        if ele.get('name') == 'leg': 
                                            print("leg found")
                                            for legs in ele.findall('geom'):
                                                print("geom found")
                                                leg_radius = legs.get('size')
                                                print("size:", leg_radius)                                                                                            
                                                return leg_radius                                               
                                   
        return leg_radius
    
    def leg_width_xml(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):           
                              
                    for new in link.findall('body'):
                        if new.get('name') == 'torso':                            
                            for element in new.findall('body'):
                                if element.get('name') == 'thigh':                                    
                                    for ele in element.findall('body'):
                                        if ele.get('name') == 'leg':                                             
                                            for legs in ele.findall('geom'):                                                
                                                leg_radius = legs.get('size')                                                                                            
                                                return leg_radius                                               
                                   
        return leg_radius
    
    def thigh_width_found(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):
            print("worldbody found")
            for slot in link.findall('body'):
                print("body found")
                if slot.get('name')== 'slot':
                    print("slot found")
                    for new in slot.findall('body'):
                        if new.get('name') == 'torso':
                            print("torso found")
                            for element in new.findall('body'):
                                if element.get('name') == 'thigh':
                                    print("thigh found")
                                    for ele in element.findall('geom'):
                                        print("geom found")
                                        thigh_radius = ele.get('size')
                                        print("size:", thigh_radius)                                                                                            
                                        return thigh_radius                                              
                                   
        return thigh_radius
    
    def thigh_width_xml(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):            
                            
                    for new in link.findall('body'):
                        if new.get('name') == 'torso':                            
                            for element in new.findall('body'):
                                if element.get('name') == 'thigh':                                    
                                    for ele in element.findall('geom'):                                       
                                        thigh_radius = ele.get('size')                                                                                            
                                        return thigh_radius                                               
                                   
        return thigh_radius
    
    def torso_width_found(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):
            print("worldbody found")
            for slot in link.findall('body'):
                print("body found")
                if slot.get('name')== 'slot':
                    print("slot found")
                    for new in slot.findall('body'):
                        if new.get('name') == 'torso':
                            print("torso found")
                            for element in new.findall('geom'):
                                print("geom found")
                                torso_radius = element.get('size')
                                print("size:", torso_radius)                                                                                            
                                return torso_radius                                              
                                   
        return torso_radius
    
    def torso_width_xml(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):            
                              
                    for new in link.findall('body'):
                        if new.get('name') == 'torso':                            
                            for element in new.findall('geom'):                                
                                torso_radius = element.get('size')                                                                                            
                                return torso_radius                                               
                                   
        return torso_radius

    
    def ee_pos(self):
        base_inertial_pos = self.base_inertial_pos()
        base_z_pos = base_inertial_pos[2]
        base_x_pos = base_inertial_pos[0]
        base_y_pos = base_inertial_pos[1]        
        #leg_length = (self.leg_length_xml()).astype(np.float64)
        leg_length = self.leg_length_xml()
        knee_pos = self.knee_position() - 1.57
        hip_pos = self.hip_position() + (1.57/2)
        #thigh_length = (self.thigh_length_xml()).astype(np.float64)
        thigh_length = self.thigh_length_xml()
        #torso_width = (self.torso_width_xml()).astype(np.float64)  
        torso_width = float(self.torso_width_xml()) 
        thigh_width = float(self.thigh_width_xml())
        leg_width = float(self.leg_width_xml())   
        base_bottom_z = base_z_pos - leg_width
        base_bottom = np.array([base_x_pos, base_y_pos, base_bottom_z])
        ee_x_pos = base_bottom[0] + thigh_length*np.sin(hip_pos) + leg_length*np.sin(knee_pos)
        ee_z_pos = base_bottom[2] - thigh_length*np.cos(hip_pos) - leg_length*np.cos(knee_pos) -0.05
        ee_y_pos = base_bottom[1]
        ee_pos = np.array([ee_x_pos, ee_y_pos, ee_z_pos])
        return ee_pos
    
    #distance calculated between INERTIAL pos of base and circular end of foot 
    
    def distance(self):
        ee_pos = self.ee_pos()
        base_inertial_pos = self.base_inertial_pos()
        distance = np.linalg.norm(ee_pos - base_inertial_pos)
        return distance
    
    def equivalent_orientation(self):
        ee_pos = self.ee_pos()
        base_inertial_pos = self.base_inertial_pos()
        distance = self.distance()
        z_axis_vector = np.array([0, 0, -1])
        base_to_foot_unit_vector = (ee_pos - base_inertial_pos) / distance

    # Calculate the angle between the base_to_foot_unit_vector and z-axis using the dot product
        dot_product = np.dot(base_to_foot_unit_vector, z_axis_vector)
        angle_with_z = np.arccos(np.clip(dot_product, -1.0, 1.0))
        return angle_with_z
    
    def linear_spring_force (self):
        distance = self.distance()        
        kp, kd = self.lin_sd_var()        
        Fls =kp * (self.ori_l - distance)     
        return Fls
    
    def torsional_spring_force(self):
        angle_with_z = self.equivalent_orientation()
        kp, kd = self.rot_sd_var()
        Fts = kp * (self.ori_theta - angle_with_z)
        return Fts
    
    def fz_by_ee(self):
        equivalent_orientation = self.equivalent_orientation()
        torsional_spring_force = self.torsional_spring_force()
        linear_spring_force = self.linear_spring_force()
        distance = self.distance()
        fz_wo_gravity = -linear_spring_force*np.cos(equivalent_orientation) + (torsional_spring_force/distance)*np.sin(equivalent_orientation)
        weight = 107.8
        fz_by_ee = fz_wo_gravity - weight
        return fz_by_ee
    
    def fx_by_ee(self):
        equivalent_orientation = self.equivalent_orientation()
        torsional_spring_force = self.torsional_spring_force()
        linear_spring_force = self.linear_spring_force()
        distance = self.distance()
        fx_wo_gravity = -linear_spring_force*np.sin(equivalent_orientation) - (torsional_spring_force/distance)*np.cos(equivalent_orientation)
        fx_by_ee = fx_wo_gravity
        return fx_by_ee
    
    def force_applied_ground(self):
        fz_by_ee = -self.fz_by_ee()
        fx_by_ee = -self.fx_by_ee()
        force_applied_ground = np.array([fx_by_ee, fz_by_ee])
        return force_applied_ground
    
    def jacobian(self):      
        leg_length = self.leg_length_xml()
        knee_pos = self.knee_position()- 1.57
        hip_pos = self.hip_position() + (1.57/2)
        thigh_length = self.thigh_length_xml()
        torso_width = float(self.torso_width_xml()) 
        thigh_width = float(self.thigh_width_xml())
        leg_width = float(self.leg_width_xml())
        leg_total_length = leg_length + leg_width
        J11 =  thigh_length*np.cos(hip_pos)+leg_total_length*np.cos(knee_pos+hip_pos)
        J12 = leg_total_length*np.cos(knee_pos+hip_pos)
        J21 = thigh_length*np.sin(hip_pos) + leg_total_length*np.sin(knee_pos + hip_pos)
        J22 = leg_total_length*np.sin(knee_pos + hip_pos)
        jacobian = np.array([[J11, J12], [J21, J22]])    
        #jacobian = np.array([[thigh_length*np.cos(hip_pos)+leg_length*np.cos(knee_pos+hip_pos), leg], [0, 1, 0], [thigh_length*np.sin(hip_pos) + leg_length*np.sin(knee_pos), 0, 0]])
        return jacobian
    
    def jacobian_transpose(self):
        jacobian = self.jacobian()
        jacobian_transpose = np.transpose(jacobian)
        return jacobian_transpose
    
    def joint_torque(self):
        jacobian_transpose = self.jacobian_transpose()
        force_applied_ground = self.force_applied_ground()
        joint_torque = np.dot(jacobian_transpose, force_applied_ground)
        return joint_torque