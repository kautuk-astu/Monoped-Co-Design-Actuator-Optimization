import xml.etree.ElementTree as ET
import numpy as np

# Parse an XML file into an ElementTree object
# tree = ET.parse('/home/stochlab/Aastha_Coopt_Monoped/maxh_iter100_exp/modified_robot.xml')

# # Get the root element of the parsed XML
# root = tree.getroot()  # This is the <robot> element

def modify_link_length(xml_file, link_name, new_length, axis='z', output_file='/home/stochlab/Aastha_Coopt_Monoped/maxh_iter100_exp/modified_robot.xml'):
    # Parse the URDF file
    tree = ET.parse(xml_file)   # Parse an XML file into an ElementTree object
    root = tree.getroot()    

    # Axis index mapping (x: 0, y: 1, z: 2)
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    axis_index = axis_map[axis]  # Get the index of the specified axis

    # Find the specific link by its name
    for link in root.findall('worldbody'):            
        for slot in link.findall('body'):        
            for new in slot.findall('body'):
                if new.get('name') == link_name:
                    pos = new.get('pos').split()
                    pos[axis_index] = str(-new_length)
                    new.set('pos', ' '.join(pos))                                              
                    for element in new.findall('geom'):                                                                           
                        fromto = element.get('fromto').split() 
                        fromto[5] = str(new_length)
                        element.set('fromto', ' '.join(fromto)) 
                    for joint in new.findall('joint'):
                        pos = joint.get('pos').split()
                        pos[axis_index] = str(new_length)
                        joint.set('pos', ' '.join(pos))
                else:
                    for link in new.findall('body'):
                        if link.get('name') == link_name:
                            pos = link.get('pos').split()
                            pos[axis_index] = str(-new_length)
                            link.set('pos', ' '.join(pos))                                              
                            for element in link.findall('geom'):                                                                           
                                fromto = element.get('fromto').split() 
                                fromto[5] = str(new_length)
                                element.set('fromto', ' '.join(fromto)) 
                            for joint in link.findall('joint'):
                                pos = joint.get('pos').split()
                                pos[axis_index] = str(new_length)
                                joint.set('pos', ' '.join(pos))                   
        

    
    tree.write(output_file)
    print(f"Modified URDF saved to '{output_file}'")


#xml_file = 'test_pb/hopper_roc.xml'
link_name = 'thigh_link'  # The name of the link you want to modify
new_length = 0.4  # The new length for the link
axis = 'z'  # Axis along which the link is elongated (typically z-axis)

for i in range(1, 10):
    new_length = 0.55
    xml_file = '/home/stochlab/Aastha_Coopt_Monoped/mujoco_tests/hopper_roc.xml'
    link_name = 'link1'  # The name of the link you want to modify
    axis = 'z'  # Axis along which the link is elongated (typically z-axis)
    modify_link_length(xml_file, link_name, new_length, axis)
    link_name = 'link2'
    new_length = 0.5
     # The name of the link you want to modify
    xml_file = '/home/stochlab/Aastha_Coopt_Monoped/maxh_iter100_exp/modified_robot.xml'
    modify_link_length(xml_file, link_name, new_length, axis)

