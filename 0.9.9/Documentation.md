# CARLA 0.9.9 Bike Simulator Tools Documentation

# Building a World
## Generation of World 3D Geometry
The 3D geometry can be generated from a SUMO net file using the SumoNetVis library. See the full SumoNetVis
documentation for full details, but the following snippet should be a good starting point:

```python
from SumoNetVis import Net
net = Net('../Sample/AtCity_Study2.net.xml')
carla_materials = {
    "pedestrian_lane": "Pathway",
    "pedestrian_connection": "Pathway",
    "other_lane": "M_Road1",
    "junction": "M_Road1",
    "w_marking": "Assets_Markings",
    "bicycle_lane": "M_Road1_bike",
    "none_lane": "M_Road1",
    "terrain": "M_Grass01"
}
with open("../Sample/AtCity_Study2.obj", "w") as f:
    f.write(net.generate_obj_text(terrain_distance=100, material_mapping=carla_materials, apply_netOffset=True))
```

The generated OBJ file can then be imported into Blender (be sure to set axis configuration to Y-Forward, Z-Up).
The geometry can then be manually cleaned up in Blender. It is also recommended to use the following blender script:

```python
import bpy

# Loops through all selected mesh objects and merges vertices that are close together and triangulates all faces
for obj in bpy.context.selected_objects:
    if obj.type == "MESH":
        bpy.ops.object.editmode_toggle()
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.mesh.remove_doubles(threshold=0.001)
        bpy.ops.mesh.quads_convert_to_tris(quad_method="BEAUTY", ngon_method="BEAUTY")
        bpy.ops.object.editmode_toggle()
```

## Importing 3D Geometry into CARLA
TODO

# Running a Simulation
TODO