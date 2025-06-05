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
# for thigh in [round(0.30 + 0.01 * i, 2) for i in range(21)]:  # 0.30 to 0.40
#     for calf in [round(0.30 + 0.01 * i, 2) for i in range(21)]:
#         temp_file = f"modified_robot_{int(thigh * 1000)}_{int(calf * 1000)}.xml"
#         modify_link_length(base_xml, 'link1', thigh, axis='z', output_file=temp_file)
#         modify_link_length(temp_file, 'link2', calf, axis='z', output_file=temp_file)

for thigh in [0.4]:  # 0.30 to 0.40
    for calf in [0.4]:
        temp_file = f"modified_robot_{int(thigh * 1000)}_{int(calf * 1000)}.xml"
        modify_link_length(base_xml, 'link1', thigh, axis='z', output_file=temp_file)
        modify_link_length(temp_file, 'link2', calf, axis='z', output_file=temp_file)



