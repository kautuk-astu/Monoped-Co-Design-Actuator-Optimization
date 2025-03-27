#NOTE: joint names chosen on the basis of hopper_roc.xml 
import mujoco as mj
import numpy as np
import xml.etree.ElementTree as ET
import time
import mujoco.viewer as mj_viewer

class len_rad_id:
    def __init__(self, xml_path,m,d):
        self.xml_path = xml_path
        self.m = m
        self.d = d 
        #NOTE: joint names chosen on the basis of hopper_roc.xml         
        self.joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "slidez")
        self.force_index = m.jnt_dofadr[self.joint_id]
        self.hip_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "hinge1")
        self.hip_force_index = m.jnt_dofadr[self.hip_joint_id]
        self.knee_joint_id = mj.mj_name2id(m, mj.mjtObj.mjOBJ_JOINT, "hinge2")
        self.knee_force_index = m.jnt_dofadr[self.knee_joint_id]


    def xml_parser(self):
            xml = self.xml_path
            tree = ET.parse(xml)
            root = tree.getroot()
            return root, tree 

    def thigh_length_xml(self):
            root, tree = self.xml_parser()
            # Get the index of the specified axis
            for link in root.findall('worldbody'):            
                for slot in link.findall('body'):               
                    if slot.get('name')== 'base':                    
                        for new in slot.findall('body'):
                            if new.get('name') == 'link1':                           
                                for element in new.findall('geom'):                                                                           
                                    fromto = element.get('fromto')                                            
                                    thigh_start_z = float(fromto.split()[2])                                    
                                    thigh_start_x = float(fromto.split()[0])
                                    
                                    thigh_start_y = float(fromto.split()[1])
                                                                                
                                    thigh_end_z = float(fromto.split()[5])    
                                    
                                    thigh_end_x = float(fromto.split()[3])
                                    
                                    thigh_end_y = float(fromto.split()[4])
                                    
                                    thigh_length = np.linalg.norm(np.array([thigh_end_x, thigh_end_y, thigh_end_z]) - np.array([thigh_start_x, thigh_start_y, thigh_start_z]))
                                                                            
                                    return thigh_length
            return thigh_length 

    def thigh_radius_xml(self):    
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):            
            for slot in link.findall('body'):               
                if slot.get('name')== 'base':                    
                    for new in slot.findall('body'):
                        if new.get('name') == 'link1':                           
                            for element in new.findall('geom'):                                                                           
                                radius = element.get('size')                                            
                                thigh_radius = float(radius)
        return thigh_radius

    def calf_length_xml(self):
            root, tree = self.xml_parser()
            # Get the index of the specified axis
            for link in root.findall('worldbody'):            
                for slot in link.findall('body'):               
                    if slot.get('name')== 'base':                    
                        for new in slot.findall('body'):
                            if new.get('name') == 'link1':                           
                                for element in new.findall('body'):
                                    if element.get('name') == 'link2':
                                        for element in element.findall('geom'):                                                                           
                                            fromto = element.get('fromto')                                            
                                            knee_start_z = float(fromto.split()[2])                                    
                                            knee_start_x = float(fromto.split()[0])
                                            
                                            knee_start_y = float(fromto.split()[1])
                                                                                    
                                            knee_end_z = float(fromto.split()[5])    
                                            
                                            knee_end_x = float(fromto.split()[3])
                                            
                                            knee_end_y = float(fromto.split()[4])
                                            
                                            calf_length = np.linalg.norm(np.array([knee_end_x, knee_end_y, knee_end_z]) - np.array([knee_start_x, knee_start_y, knee_start_z]))
                                                                                    
                                            return calf_length
            return calf_length

    def calf_radius_xml(self):
        root, tree = self.xml_parser()
        # Get the index of the specified axis
        for link in root.findall('worldbody'):            
            for slot in link.findall('body'):               
                if slot.get('name')== 'base':                    
                    for new in slot.findall('body'):
                        if new.get('name') == 'link1':                           
                            for element in new.findall('body'):
                                if element.get('name') == 'link2':
                                    for element in element.findall('geom'):                                                                           
                                        radius = element.get('size')                                            
                                        calf_radius = float(radius)
        return calf_radius

    #assuming total length for thigh and calf hemispherical end to be within thigh(hence not included)
    def thigh_total_length(self):
        thigh_length = self.thigh_length_xml()
        thigh_radius = self.thigh_radius_xml()
        total_length = thigh_length- thigh_radius
        return total_length

    def calf_total_length(self):
        calf_length = self.calf_length_xml()
        calf_radius = self.calf_radius_xml()
        total_length = calf_length + calf_radius
        return total_length