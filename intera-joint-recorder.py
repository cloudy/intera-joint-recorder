#!/usr/bin/env python2

import numpy as np
import rospy
import intera_interface
import argparse


rospy.init_node("intera_joint_recorder")

parser = argparse.ArgumentParser(description="Record joint trajectories")
parser.add_argument('-of', '--output-file', type=str, default='output.txt')
arg = parser.parse_args()

limb = intera_interface.Limb('right')
cuff = intera_interface.Cuff('right')
lights = intera_interface.Lights()

angles = dict(zip(limb.joint_names(), np.radians([0,-50,0,120,0,0,0]))) # Start pose
limb.move_to_joint_positions(angles)

lights.set_light_state('right_hand_red_light')
while not cuff.lower_button():
    pass

lights.set_light_state('right_hand_green_light')
lights.set_light_state('right_hand_red_light', False)

q, t = [], []
while not cuff.lower_button():
    joint_positions = limb.joint_angles()
    q.append([float(joint_positions[i]) for i in limb.joint_names()])
    t.append(rospy.get_time())
    rospy.sleep(0.01)

lights.set_light_state('right_hand_red_light')
lights.set_light_state('right_hand_green_light', False)

traj_final = np.concatenate((np.array(t).reshape((len(t),1))-t[0], np.array(q), np.ones((len(q),1)*0.0402), axis=1)
header = 'time,right_j0,right_j1,right_j2,right_j3,right_j4,right_j5,right_j6,right_gripper'
np.savetxt(arg.output_file, traj_final, delimiter=',', header=header, comments='', fmt="%1.12f")

lights.set_light_state('right_hand_red_light', False)
