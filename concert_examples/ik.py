import rospy

from trac_ik_python.trac_ik import IK

rospy.init_node('test_script')
# Get your URDF from somewhere
with open('/home/mlei/concert_ws/ros_src/modularbot/urdf/ModularBot.urdf', 'r') as file:
    urdf=file.read()


ik_solver = IK("base_link",
               "ee_A", urdf_string=urdf)

lower_bound, upper_bound = ik_solver.get_joint_limits()

print(lower_bound)
print(upper_bound)

seed_state = [0.0] * ik_solver.number_of_joints

solution = ik_solver.get_ik(seed_state,
                0.45, 0.1, 0.3,
                0.0, 0.0, 0.0, 1.0,
                0.0, 0.01, 0.01,  # X, Y, Z bounds
                100, 1000, 100)  # Rotation X, Y, Z bounds

print(solution)
                