import numpy as np
import pybullet as p
import pybullet_data
import xml.etree.ElementTree as ET

class Controller:
    def __init__(self, urdf, robot_id, hip_joint_index=1, knee_joint_index=2, kl=1
                 , cl=10, kt=1
                 , ct=10,base_joint_index=1, ori_l = 1.0, ori_theta=0):
        self.urdf = urdf
        self.robot_id = robot_id
        self.hip_joint_index = hip_joint_index
        self.knee_joint_index = knee_joint_index
        self.base_joint_index = base_joint_index
        self.ori_l = ori_l
        self.ori_theta = ori_theta
        
        # PD gains for each joint
        self.kl = kl
        self.cl = cl
        self.kt = kt    #might have to change this
        self.ct = ct


    def test(self):
        print("robot_id: ", self.robot_id)
        print("urdf", self.urdf)
    
    
    def calculate_distance_base_to_foot(self, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
        """Calculate the distance from the base link to the foot link in the world frame."""
        # TESTED AND WORKING
        base_link_state = p.getLinkState(self.robot_id, base_link_index)
        base_position = np.array(base_link_state[0]) # Position is the first element
        base_position[2] = base_position[2] -0.025  
        base_quat = base_link_state[1]         # Orientation is the second element
        base_orientation = p.getEulerFromQuaternion(base_quat)  

        # Get the world position and orientation of the calf link
        calf_link_state = p.getLinkState(self.robot_id, calf_link_index)
        calf_position = np.array(calf_link_state[0])    # position of inertial frame w.r.t. world frame
        calf_quat = calf_link_state[1]           # World orientation (quaternion) of intertial frame
        calf_orientation = p.getEulerFromQuaternion(calf_quat)  # Convert quaternion to Euler angles

        knee_joint_state = p.getJointState(self.robot_id, knee_joint_index) #theta2
        knee_position = knee_joint_state[0]

        hip_joint_state = p.getJointState(self.robot_id, hip_joint_index)   #theta1
        hip_position = hip_joint_state[0]

        # Calculate the foot coordinates in the world frame using calf link position and orientation
        foot_x = calf_position[0]
        foot_y = calf_position[1] + 0.1 * np.sin(knee_position + hip_position)
        foot_z = calf_position[2] - 0.1 * np.cos(knee_position + hip_position)
        foot_offset_world = [foot_x, foot_y, foot_z]

        
        foot_position_world = np.array(foot_offset_world)
        
        # Calculate the Euclidean distance from the base to the foot position
        distance = np.linalg.norm(foot_position_world - base_position)
        z_axis_vector = np.array([0, 0, -1])
        base_to_foot_unit_vector = (foot_position_world - base_position) / distance

    # Calculate the angle between the base_to_foot_unit_vector and z-axis using the dot product
        dot_product = np.dot(base_to_foot_unit_vector, z_axis_vector)
        angle_with_z = np.arccos(np.clip(dot_product, -1.0, 1.0))
        
        return distance,angle_with_z
    
    def obtain_base_posorn(self, base_link_index = 0):
        base_link_state = p.getLinkState(self.robot_id, base_link_index)
        base_position = np.array(base_link_state[0]) # Position is the first element
        base_position[2] = base_position[2] - 0.025  
        base_quat = base_link_state[1]         # Orientation is the second element
        base_orientation = p.getEulerFromQuaternion(base_quat)
        return base_position, base_orientation
    
    def obtain_calf_posorn(self, calf_link_index = 2):
        calf_link_state = p.getLinkState(self.robot_id, calf_link_index)
        calf_position = np.array(calf_link_state[0])    # position of inertial frame w.r.t. world frame
        calf_quat = calf_link_state[1]           # World orientation (quaternion) of intertial frame
        calf_orientation = p.getEulerFromQuaternion(calf_quat)
        return calf_position, calf_orientation
    
    def obtain_thigh_posorn(self, thigh_link_index = 1):
        thigh_link_state = p.getLinkState(self.robot_id, thigh_link_index)
        thigh_position = np.array(thigh_link_state[0])    # position of inertial frame w.r.t. world frame
        thigh_quat = thigh_link_state[1]           # World orientation (quaternion) of intertial frame
        thigh_orientation = p.getEulerFromQuaternion(thigh_quat)
        return thigh_position, thigh_orientation
    
    def obtain_base_velocities(self, base_joint_index = 0):
        base_joint_state = p.getJointState(self.robot_id, base_joint_index)
        base_velocity = base_joint_state[1]
        return base_velocity
    
    #function to detect contact between calf adn ground
    def detect_contact(self, calf_link_index = 2):
        contact_points = p.getContactPoints(bodyA=self.robot_id, linkIndexA=calf_link_index)
        if len(contact_points) > 0:
            return True
        else:
            return False
    def obtain_hip_velocities(self, hip_joint_index = 1):
        hip_joint_state = p.getJointState(self.robot_id, hip_joint_index)
        hip_velocity = hip_joint_state[1]
        return hip_velocity
    
    def obtain_knee_velocities(self, knee_joint_index = 2):
        knee_joint_state = p.getJointState(self.robot_id, knee_joint_index)
        knee_velocity = knee_joint_state[1]
        return knee_velocity
    
    def obtain_hip_positions(self, hip_joint_index = 1):
        hip_joint_state = p.getJointState(self.robot_id, hip_joint_index)
        hip_position = hip_joint_state[0]
        return hip_position
    
    def obtain_knee_positions(self, knee_joint_index = 2):
        knee_joint_state = p.getJointState(self.robot_id, knee_joint_index)
        knee_position = knee_joint_state[0]
        return knee_position
    
    def obtain_calf_velocities(self, calf_link_index = 2):
        calf_link_state = p.getLinkState(self.robot_id, calf_link_index, computeLinkVelocity = 1)
        calf_velocity = calf_link_state[6]
        return calf_velocity
    
    def obtain_thigh_velocities(self, thigh_link_index = 1):
        thigh_link_state = p.getLinkState(self.robot_id, thigh_link_index, computeLinkVelocity = 1)
        thigh_velocity = thigh_link_state[6]
        return thigh_velocity
    
    def foot_pos(self, calf_link_index = 2, hip_joint_index = 1, knee_joint_index = 2):
        calf_position, calf_orientation = self.obtain_calf_posorn(calf_link_index)
        knee_joint_state = p.getJointState(self.robot_id, knee_joint_index)
        knee_position = knee_joint_state[0]
        hip_joint_state = p.getJointState(self.robot_id, hip_joint_index)
        hip_position = hip_joint_state[0]
        foot_x = calf_position[0]
        foot_y = calf_position[1] + 0.1 * np.sin(knee_position + hip_position)
        foot_z = calf_position[2] - 0.1 * np.cos(knee_position + hip_position)
        return foot_x, foot_y, foot_z   
    
    def calculate_spring_forces (self, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
        distance,angle_with_z = self.calculate_distance_base_to_foot(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        Fls = self.kl * (self.ori_l - distance)     
        return Fls
    
    def diff_yee (self, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
        foot_x, foot_y, foot_z = self.foot_pos(calf_link_index, hip_joint_index, knee_joint_index)
        base_position, base_orientation = self.obtain_base_posorn(base_link_index)
        distance, angle_with_z = self.calculate_distance_base_to_foot(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        dl_dyee= (foot_y - base_position[1])/distance
        calf_velocity = self.obtain_calf_velocities(calf_link_index)
        calf_ydot = calf_velocity[1]
        theta1_dot = self.obtain_hip_velocities(hip_joint_index)
        theta2_dot = self.obtain_knee_velocities(knee_joint_index)
        theta1 = self.obtain_hip_positions(hip_joint_index)
        theta2 = self.obtain_knee_positions(knee_joint_index)
        dyee_dt = calf_ydot + 0.1 * (np.cos(theta1 + theta2) * theta1_dot + np.cos(theta1 + theta2) * theta2_dot)
        diff_yee = dl_dyee * dyee_dt
        return diff_yee
    
    def diff_yb(self, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
        foot_x, foot_y, foot_z = self.foot_pos(calf_link_index, hip_joint_index, knee_joint_index)
        base_position, base_orientation = self.obtain_base_posorn(base_link_index)
        distance, angle_with_z = self.calculate_distance_base_to_foot(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        dl_dyb= (base_position[1] - foot_y)/distance
        base_velocity = self.obtain_base_velocities(base_link_index)
        dyb_dt = base_velocity
        diff_yb = dl_dyb * dyb_dt
        return diff_yb
    
    def diff_zee (self, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
        foot_x, foot_y, foot_z = self.foot_pos(calf_link_index, hip_joint_index, knee_joint_index)
        base_position, base_orientation = self.obtain_base_posorn(base_link_index)
        distance, angle_with_z = self.calculate_distance_base_to_foot(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        dl_dzee= (foot_z - base_position[2])/distance
        calf_velocity = self.obtain_calf_velocities(calf_link_index)
        calf_zdot = calf_velocity[2]
        theta1_dot = self.obtain_hip_velocities(hip_joint_index)
        theta2_dot = self.obtain_knee_velocities(knee_joint_index)
        theta1 = self.obtain_hip_positions(hip_joint_index)
        theta2 = self.obtain_knee_positions(knee_joint_index)
        dzee_dt = calf_zdot + 0.1 * (np.sin(theta1 + theta2) * theta1_dot + np.sin(theta1 + theta2) * theta2_dot)
        diff_zee = dl_dzee * dzee_dt
        return diff_zee
    
    def diff_zb(self, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
        foot_x, foot_y, foot_z = self.foot_pos(calf_link_index, hip_joint_index, knee_joint_index)
        base_position, base_orientation = self.obtain_base_posorn(base_link_index)
        distance, angle_with_z = self.calculate_distance_base_to_foot(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        dl_dzb= (base_position[2] - foot_z)/distance
        base_velocity = self.obtain_base_velocities(base_link_index)
        dzb_dt = base_velocity
        diff_zb = dl_dzb * dzb_dt
        return diff_zb
    
    def calculate_damping_forces (self, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
        diff_yee = self.diff_yee(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        diff_yb = self.diff_yb(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        diff_zee = self.diff_zee(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        diff_zb = self.diff_zb(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        dl_dt = diff_yee + diff_yb + diff_zee + diff_zb
        Flc = -self.cl * dl_dt
        return Flc
    
    def calculate_spring_torque (self, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
        distance,angle_with_z = self.calculate_distance_base_to_foot(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        Tls = self.kt * (self.ori_theta - angle_with_z)
        return Tls
    
    def total_sd_force(self, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
        Fls = self.calculate_spring_forces(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        Flc = self.calculate_damping_forces(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        Fl = Fls + Flc
        return Fl
    
    def total_sd_torque(self, base_link_index = 0, calf_link_index =2, hip_joint_index = 1, knee_joint_index = 2):
        Tt = self.calculate_spring_torque(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        return Tt

    #TODO: Add damping torque calculation function
    def foot_force(self, base_link_index = 0, calf_link_index = 2, hip_joint_index = 1, knee_joint_index = 3):
        #distance,angle_with_z = self.calculate_distance_base_to_foot(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        distance,angle_with_z = self.calculate_distance_base_to_foot()
        Fl = self.total_sd_force()
        Tt = self.total_sd_torque()
        Fz = Fl * np.cos(angle_with_z) + (Tt/distance) * np.sin(angle_with_z)
        Fy = Fl * np.sin(angle_with_z) - (Tt/distance) * np.cos(angle_with_z)
        return Fz,Fy
    
    def urdf_parser(self, axis = 'z'):
        tree = ET.parse(self.urdf)   # Parse an XML file into an ElementTree object
        root = tree.getroot() 
        return root, tree 

    def obtain_calf_length(self, axis = 'z'):
        root, tree = self.urdf_parser(self.urdf)
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        axis_index = axis_map[axis]  # Get the index of the specified axis
        for link in root.findall('link'):
            if link.get('name') == 'calf_link':
                    for element in link.findall('visual'):
                        geometry = element.find('geometry')
                        if geometry is not None:
                            cylinder = geometry.find('cylinder')
                            if cylinder is not None:
                                length = cylinder.get('length') #because cylinder has length and radius attribute
                                length = float(length)
                                return length
        return length

    def obtain_thigh_length(self, axis = 'z'):
        root, tree = self.urdf_parser(self.urdf)
        axis_map = {'x': 0, 'y': 1, 'z': 2}
        axis_index = axis_map[axis]  # Get the index of the specified axis
        for link in root.findall('link'):
            if link.get('name') == 'thigh_link':
                    for element in link.findall('visual'):
                        geometry = element.find('geometry')
                        if geometry is not None:
                            cylinder = geometry.find('cylinder')
                            if cylinder is not None:
                                length = cylinder.get('length')
                                length = float(length) #because cylinder has length and radius attribute
                                return length
                            
        return length

    def jacobian(self, base_link_index = 0, calf_link_index = 2, hip_joint_index = 1, knee_joint_index = 2):
        theta1 = self.obtain_hip_positions(hip_joint_index)
        theta2 = self.obtain_knee_positions(knee_joint_index)
        l1 = self.obtain_thigh_length()
        l2 = self.obtain_calf_length()
        J11 = l1*np.sin(theta1) + l2*np.sin(theta1 + theta2)
        J12 = l2*np.sin(theta1 + theta2)
        J21 = l1*np.cos(theta1) + l2*np.cos(theta1 + theta2)
        J22 = l2*np.cos(theta1 + theta2)
        J = np.array([[J11, J12],
                      [J21, J22]])
        return J
    
    def jacobian_transpose(self, base_link_index = 0, calf_link_index = 2, hip_joint_index = 1, knee_joint_index = 2):
        J = self.jacobian(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        J_transpose = np.transpose(J)
        return J_transpose
    
    def joint_torques(self, base_link_index = 0, calf_link_index = 2, hip_joint_index = 1, knee_joint_index = 2):
        Fz, Fy = self.foot_force(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        F = np.array([[Fz], [Fy]])
        J_transpose = self.jacobian_transpose(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        tau = np.dot(J_transpose, F)
        return tau
    
    def apply_joint_torques(self, base_link_index = 0, calf_link_index = 2, hip_joint_index = 1, knee_joint_index = 2):
        tau = self.joint_torques(base_link_index, calf_link_index, hip_joint_index, knee_joint_index)
        p.setJointMotorControl2(
            bodyIndex=self.robot_id,
            jointIndex=hip_joint_index,
            controlMode=p.VELOCITY_CONTROL,  # Turn off any velocity or position control
            force=0  # Set force to 0 to disable it initially
        )

        p.setJointMotorControl2(
            bodyIndex=self.robot_id,
            jointIndex=knee_joint_index,
            controlMode=p.VELOCITY_CONTROL,  # Turn off any velocity or position control
            force=0  # Set force to 0 to disable it initially
        )

        p.setJointMotorControl2(
            bodyIndex=self.robot_id,
            jointIndex=hip_joint_index,
            controlMode=p.TORQUE_CONTROL,
            force=tau[0]
        )
        p.setJointMotorControl2(
            bodyIndex=self.robot_id,
            jointIndex=knee_joint_index,
            controlMode=p.TORQUE_CONTROL,
            force=tau[1]
        )

    def step(self):
        """Step function to apply control torques for both hip and knee."""
        # Compute and apply control torques for the hip joint
        self.apply_joint_torques(self)
        base_position, base_orientation = self.obtain_base_posorn()
        base_height = base_position[2]
        return base_height

