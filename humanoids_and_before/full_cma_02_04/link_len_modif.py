import xml.etree.ElementTree as ET

def modify_link_length(xml_file, link_name, new_length, axis='z', output_file=None):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    axis_map = {'x': 0, 'y': 1, 'z': 2}
    axis_index = axis_map[axis]

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

    if output_file:
        tree.write(output_file)
        print(f"Modified URDF saved to '{output_file}'")

# Base XML path
base_xml = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/full_cma_02_04/modified_robot.xml'

# Output directory
output_dir = '/home/stochlab/repo/Aastha_Coopt_Monoped/Monoped-optimization/full_cma_02_04/'

# Loop through desired link lengths
for thigh in [0.20, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27, 0.28, 0.29, 
              0.30, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.37, 0.38, 0.39, 0.40]:
    for calf in [0.20, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27, 0.28, 0.29, 
                 0.30, 0.31, 0.32, 0.33, 0.34, 0.35, 0.36, 0.37, 0.38, 0.39, 0.40]:
        temp_file = f"modified_robot_{int(thigh * 1000)}_{int(calf * 1000)}.xml"
        modify_link_length(base_xml, 'link1', thigh, axis='z', output_file=temp_file)
        modify_link_length(temp_file, 'link2', calf, axis='z', output_file=temp_file)

