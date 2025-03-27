import xml.etree.ElementTree as ET
import numpy as np

# Parse an XML file into an ElementTree object
tree = ET.parse('test_pb/hopper_roc.xml')

# Get the root element of the parsed XML
root = tree.getroot()  # This is the <robot> element



print(root.findall('link'))

# Find the <link> element with the name 'thigh' 
thigh_link = root.find(".//link[@name='thigh']")
print(thigh_link)

def modify_link_length(urdf_file, link_name, new_length, axis='z', output_file='modified_robot.urdf'):
    # Parse the URDF file
    tree = ET.parse(urdf_file)   # Parse an XML file into an ElementTree object
    root = tree.getroot()    

    # Axis index mapping (x: 0, y: 1, z: 2)
    axis_map = {'x': 0, 'y': 1, 'z': 2}
    axis_index = axis_map[axis]  # Get the index of the specified axis

    # Find the specific link by its name
    for link in root.findall('link'):    #prints all elements with tag 'link' with location in file 
        if link.get('name') == link_name:
            
            # Update geometry dimensions (for visual and collision tags)
            for tag in ['visual', 'collision']:
                for element in link.findall(tag):
                    geometry = element.find('geometry')
                    if geometry is not None:
                        cylinder = geometry.find('cylinder')
                        if cylinder is not None:
                            # Modify cylinder length
                            cylinder.set('length', str(new_length))
                        box = geometry.find('box')
                        if box is not None:
                            # Modify box dimensions (assuming along z-axis)
                            size = box.get('size').split()
                            size[axis_index] = str(new_length)
                            box.set('size', ' '.join(size))

                    # Adjust the origin of the visual/collision frame if present
                    origin = element.find('origin')
                    if origin is not None and 'xyz' in origin.attrib:
                        xyz = origin.get('xyz').split()
                        # Adjust the position along the specified axis
                        xyz[axis_index] = str((-new_length) / 2)
                        origin.set('xyz', ' '.join(xyz))  

            # Update the inertial tag (if present)
            inertial = link.find('inertial')
            if inertial is not None:
                # Adjust the position of the inertial frame
                origin = inertial.find('origin')
                if origin is not None and 'xyz' in origin.attrib:
                    xyz = origin.get('xyz').split()
                    # Adjust the position along the specified axis
                    xyz[axis_index] = str((-new_length)/ 2)
                    origin.set('xyz', ' '.join(xyz))

    # Write the modified URDF back to the file (or a new file)
    #tree.write('modified_' + urdf_file)
    tree.write(output_file)
    print(f"Modified URDF saved to '{output_file}'")


urdf_file = '/home/stochlab/Aastha_pybullet/test_pb/final.urdf'
link_name = 'thigh_link'  # The name of the link you want to modify
new_length = 0.3  # The new length for the link
axis = 'z'  # Axis along which the link is elongated (typically z-axis)

for i in range(1, 10):
    new_length = 0.2 + i * 0.01
    urdf_file = '/home/stochlab/Aastha_pybullet/test_pb/final.urdf'
    link_name = 'thigh_link'  # The name of the link you want to modify
    axis = 'z'  # Axis along which the link is elongated (typically z-axis)
    modify_link_length(urdf_file, link_name, new_length, axis)


