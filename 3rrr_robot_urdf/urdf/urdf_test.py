from urdfpy import URDF
robot = URDF.load('3rrr_robot_urdf.urdf')   
for joint in robot.actuated_joints:
    print(joint.name)


robot.show(cfg={
    'joint_bras_ax12_0': 0.5,
    'joint_bras_ax12_2': 0.4,
    'joint_bras_triangle_0': -1.0,
    'joint_bras_triangle_1': 1.0,
    'joint_bras_triangle_2': -1.0,
    'joint_triangle': 0.0
})