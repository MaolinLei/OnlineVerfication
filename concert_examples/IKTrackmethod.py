import numpy as np
import rospy
import rospkg
import scipy.io
import yaml
from os.path import join, dirname, abspath
import os
import argparse
from math import cos, sin, trunc

from trac_ik_python.trac_ik import IK

from xbot_interface import xbot_interface as xbi
from xbot_interface import config_options as co
from cartesian_interface.pyci_all import *


import rospy


class coll_avoid_demo(sequence):
    def __init__(self):

        # change ci to basic stack
        global ci
        ci = ci_basic
        ci.reset(time)
        ci.update(time, dt)

        # relevant tasks
        lhand = ci.getTask('arm1_8')
        rhand = ci.getTask('arm2_8')
        com = ci.getTask('com')
        flwheel = ci.getTask('wheel_1')

        # list of poses
        states = list()

        # front polygon side
        wh_poses = [model.getPose('wheel_' + str(i+1)) for i in range(4)]
        tgt_pos = (wh_poses[0].translation + wh_poses[1].translation)/2.0
        T = Affine3(pos=tgt_pos)

        # l start post
        rstart = model.getPose('arm2_8')
        lstart = model.getPose('arm1_8')

        # old man watching construction site
        Tr = rstart.copy()
        Tr.translation[0] -= 1.0
        Tr.translation[1] -= 0.15

        Tl = lstart.copy()
        Tl.translation[0] -= 1.0
        Tl.translation[1] = 0.15

        # states.append(lambda Tr=Tr, Tl=Tl: goto([lhand, rhand], [Tl, Tr], [6.0, 6.0]))
        # states.append(lambda: wait_converged(lambda: True))

        # states.append(lambda Tr=rstart, Tl=lstart: goto([lhand, rhand], [Tl, Tr], [6.0, 6.0]))
        # states.append(lambda: wait_converged(lambda: True))

        # disable right, go with left
        def l_ee_off():
            lhand.setActivationState(pyci.ActivationState.Disabled)
            rhand.setActivationState(pyci.ActivationState.Enabled)
            return True

        def r_ee_off():
            rhand.setActivationState(pyci.ActivationState.Disabled)
            lhand.setActivationState(pyci.ActivationState.Enabled)
            return True

        def lr_ee_on():
            rhand.setActivationState(pyci.ActivationState.Enabled)
            lhand.setActivationState(pyci.ActivationState.Enabled)
            return True

        states.append(lambda: r_ee_off)
        states.append(lambda T=T: goto(lhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=lstart: goto(lhand, T, 6.0))

        states.append(lambda: l_ee_off)
        states.append(lambda T=T: goto(rhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=rstart: goto(rhand, T, 6.0))

        states.append(lambda: wait_converged(lambda: True))

        # extreme right hand motions
        T = rstart.copy()
        T.translation[0] += 1.0
        states.append(lambda T=T: goto(rhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=rstart: goto(rhand, T, 8.0))

        T = rstart.copy()
        T.translation[1] += 1.5
        states.append(lambda T=T: goto(rhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=rstart: goto(rhand, T, 8.0))

        T = rstart.copy()
        T.translation[2] += 1.0
        states.append(lambda T=T: goto(rhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=rstart: goto(rhand, T, 8.0))

        # note: tested after collision model update
        T = rstart.copy()
        T.translation[0] -= 1.5
        states.append(lambda T=T: goto(rhand, T, 6.0))
        states.append(lambda: wait_time(1.0, lambda: True))
        states.append(lambda T=rstart: goto(rhand, T, 8.0))

        # com left-right
        states.append(lambda: wait_converged(lambda: True))
        states.append(lambda: lr_ee_on)

        comstart = Affine3(pos=model.getCOM())

        for _ in range(2):
            T = comstart.copy()
            T.translation[1] += 0.15
            states.append(lambda T=T: goto(com, T, 2.0))

            T = comstart.copy()
            T.translation[1] -= 0.15
            states.append(lambda T=T: goto(com, T, 2.0))

        states.append(lambda: wait_converged(lambda: True))

        # raise wheel
        whstart = wh_poses[0]
        T = whstart.copy()
        T.translation[2] += 0.45
        states.append(lambda T=T: goto(flwheel, T, 3.0))
        states.append(lambda: wait_converged(lambda: True))

        states.append(lambda T=whstart: goto(flwheel, T, 3.0))
        states.append(lambda T=comstart: goto(com, T, 3.0))
        states.append(lambda: wait_converged(lambda: True))

        sequence.__init__(self, states, lambda: wait_time(3.0, lambda: mission_complete()))



def update_ik(ci, model, time, dt):
    ci.update(time, dt)
    q = model.getJointPosition()
    qdot = model.getJointVelocity()
    q += qdot * dt
    model.setJointPosition(q)
    model.update()
    return q, qdot

def get_xbot_cfg(urdf, srdf):
    cfg = co.ConfigOptions()
    cfg.set_urdf(urdf)
    cfg.set_srdf(srdf)
    cfg.generate_jidmap()
    cfg.set_string_parameter('model_type', 'RBDL')
    cfg.set_string_parameter('framework', 'ROS')
    cfg.set_bool_parameter('is_model_floating_base', False)
    return cfg

# some utility functions
def get_ci(model, ikpb, dt):
    return pyci.CartesianInterface.MakeInstance('OpenSot', ikpb, model, dt)


def get_ikpb_basic():
    ikpath = join(dirname(abspath(__file__)), '..', 'configs', 'collision_avoidance_stack.yaml')
    with open(ikpath, 'r') as f:
        return f.read()



model_cfg = get_xbot_cfg(rospy.get_param('/xbotcore/robot_description'),
                        rospy.get_param('/xbotcore/robot_description_semantic'))
model = xbi.ModelInterface(model_cfg)

qref = 
model.setJointPosition(qref)
model.update()

# get ci
dt = 0.01
ci_basic = get_ci(model, get_ikpb_basic(), dt)
ci = ci_basic

# define tasks
time = 0.0
done = False


update_ik(ci, model, time, dt)


