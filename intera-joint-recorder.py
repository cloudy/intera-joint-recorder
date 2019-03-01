#!/usr/bin/env python2

import numpy as np
import rospy
import intera_interface
from args import arg

rospy.init_node("intera_joint_recorder")

# Create an object to interface with the arm
limb = intera_interface.Limb('right')
gripper = intera_interface.Gripper('right')
cuff = intera_interface.Cuff('right')
lights = intera_interface.Lights()

# Move the robot to the starting point
angles = limb.joint_angles()
angles['right_j0'] = np.radians(0)
angles['right_j1'] = np.radians(-50)
angles['right_j2'] = np.radians(0)
angles['right_j3'] = np.radians(120)
angles['right_j4'] = np.radians(0)
angles['right_j5'] = np.radians(0)
angles['right_j6'] = np.radians(0)
limb.move_to_joint_positions(angles)

lights.set_light_state('right_hand_red_light')

while not cuff.lower_button():
    pass

print('Recording of trajectory has started!')
lights.set_light_state('right_hand_green_light')
lights.set_light_state('right_hand_red_light', False)

q = []
t = []
while not cuff.lower_button():
    joint_positions = limb.joint_angles()
    q.append([float(joint_positions[i]) for i in limb.joint_names()])
    t.append(rospy.get_time())
    rospy.sleep(0.01)

lights.set_light_state('right_hand_red_light')
lights.set_light_state('right_hand_green_light', False)

# Save trajectory
traj_final = np.concatenate((np.array(q), np.multiply(np.ones((len(q), 1)), 0.0402075604203)), axis=1)
time = np.linspace(0, t[-1] - t[0], len(q)).reshape((len(q), 1))
traj_final = np.concatenate((time, traj_final), axis=1)

# Save trajectory
print('Saving trajectory as %s' % arg.output_file)
header = 'time,right_j0,right_j1,right_j2,right_j3,right_j4,right_j5,right_j6,right_gripper'
np.savetxt(arg.output_file, traj_final, delimiter=',', header=header, comments='', fmt="%1.12f")

lights.set_light_state('right_hand_red_light', False)
