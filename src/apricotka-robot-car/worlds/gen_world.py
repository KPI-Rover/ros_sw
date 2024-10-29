import random

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
      <uri>model://ground_plane</uri>
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

tree_rows=5
tree_cols=10
rows_spacing=3.0
cols_spacing=1.5
shift_range = 0.1

gap_col=5

# Add grid of pine trees
for row in range(tree_rows):
    for col in range(tree_cols+1):
        if col == gap_col:
            continue

        x_pos = (col + 1) * cols_spacing
        y_pos = row * rows_spacing
        x_pos += round(random.uniform(-shift_range, shift_range), 3) 
        y_pos += round(random.uniform(-shift_range, shift_range), 3) 

        sdf_content += f'''
    <include>
        <name>tree_{row}_{col}</name>
        <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree</uri>
        <pose>{x_pos} {y_pos} 0 0 0 0</pose>
    </include>
'''

sdf_content += '''
    </world>
</sdf>
'''

# Save the generated SDF content to a file
with open("empty.world", "w") as sdf_file:
    sdf_file.write(sdf_content)
print("World has been generated")