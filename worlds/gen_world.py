import random

from argparse import ArgumentParser

arg_parser = ArgumentParser()
arg_parser.add_argument('f',help='filename')

args = arg_parser.parse_args()

filename = args.f


sdf_content = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
        name="gz::sim::systems::Imu">
    </plugin>

    
    <include>
      <uri>https://fuel.gazebosim.org/1.0/hexarotor/models/grasspatch</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <include>
      <uri>
        https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun
      </uri>
    </include>

    <scene>
      <shadows>false</shadows>
    </scene>

    <!-- Tree grid -->
'''

tree_rows=3
tree_cols=5
rows_spacing=5.0
cols_spacing=2.5
shift_range = 1.0

gap_col=5

# Add grid of trees
for row in range(tree_rows):
    for col in range(tree_cols+1):
        if col == gap_col:
            continue

        x_pos = (col + 1) * cols_spacing - 8
        y_pos = row * rows_spacing - 5
        x_pos += round(random.uniform(-shift_range, shift_range), 3) 
        y_pos += round(random.uniform(-shift_range, shift_range), 3) 

        sdf_content += f'''
    <include>
        <name>tree_{row}_{col}</name>
        <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Oak tree</uri>
        <pose>{x_pos} {y_pos} 0 0 0 0</pose>
    </include>
'''

sdf_content += '''
    </world>
</sdf>
'''


# Save the generated SDF content to a file
with open(filename, "w") as sdf_file:
    sdf_file.write(sdf_content)
print("World has been saved into ", filename)