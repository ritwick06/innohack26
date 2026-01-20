
from controller import Supervisor
import math
import random

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

world_info = robot.getFromDef("WORLD_INFO")
if world_info:
    world_info.getField("basicTimeStep").setSFFloat(16.0)

# DEF References
lake_node = robot.getFromDef("lake_physics")
water_app = robot.getFromDef("water_appearance")
visual_surface = robot.getFromDef("water_trans") 

# Field Access
stream_vel_f = lake_node.getField("streamVelocity")
surface_trans_f = visual_surface.getField("translation")
surface_rot_f = visual_surface.getField("rotation")
lake_rot_f = lake_node.getField("rotation")
tex_trans_f = water_app.getField("textureTransform").getSFNode().getField("translation")

bottle_names = [f"trash{i}" for i in range(0, 20)] 
bottle_nodes = []

for name in bottle_names:
    node = robot.getFromDef(name)
    if node:
        bottle_nodes.append(node)
        
        physics_f = node.getField("physics")
        if physics_f and physics_f.getSFNode():
            p_node = physics_f.getSFNode()
            p_node.getField("density").setSFFloat(500.0) 
            
            damping_f = p_node.getField("damping")
            if damping_f and damping_f.getSFNode():
                d_node = damping_f.getSFNode()
                d_node.getField("linear").setSFFloat(0.9) 
                d_node.getField("angular").setSFFloat(0.8)
        
        immersion_f = node.getField("immersionProperties")
        if immersion_f and immersion_f.getCount() > 0:
            i_node = immersion_f.getMFNode(0)
            i_node.getField("dragForceCoefficients").setSFVec3f([20, 20, 20])
            i_node.getField("viscousResistanceForceCoefficient").setSFFloat(0.01)
            i_node.getField("referenceArea").setSFString("immersed area")

        t_f = node.getField("translation")
        t_f.setSFVec3f([random.uniform(-70, 70), random.uniform(-70, 70), 5.0])
        node.resetPhysics()

heave_amp = 0.05      
freq = 0.25       
tilt_limit = 0.003
z_baseline = 5.0
offset_x = 0.0

while robot.step(timestep) != -1:
    t = robot.getTime()

    z_final = z_baseline + (math.sin(t * freq) * heave_amp)
    pitch = (math.sin(t * 0.4) * tilt_limit)
    roll = (math.cos(t * 0.5) * tilt_limit)
    rot_vec = [pitch, roll, 0.0, math.sqrt(pitch**2 + roll**2)]
    
    surface_trans_f.setSFVec3f([0.0, 0.0, z_final])
    surface_rot_f.setSFRotation(rot_vec)
    lake_rot_f.setSFRotation(rot_vec)

    vx = 0.05 + random.uniform(-0.01, 0.01)
    stream_vel_f.setSFVec3f([vx, 0.0, 0.0])
    
    offset_x += vx * 0.0005 
    tex_trans_f.setSFVec2f([offset_x, 0.0])

    for node in bottle_nodes:
        pos = node.getField("translation").getSFVec3f()
        if pos[0] > 75.0:
            node.getField("translation").setSFVec3f([-75.0, random.uniform(-70, 70), 5.0])
            node.resetPhysics()