#!/usr/bin/python3

from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.utils import utils
from horizon.ros import replay_trajectory
import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn
import casadi as cs
import rospy
import rospkg
import numpy as np
import subprocess
import os
from cartesian_interface.pyci_all import *


xbot_mode = True

rospy.init_node('concert_obstacle_avoidance')
roscpp.init('concert_obstacle_avoidance', [])

# get from ros param the urdf and srdf
urdf = rospy.get_param(param_name='/xbotcore/robot_description', default='')
if urdf == '':
    raise print('urdf not set')

srdf = rospy.get_param(param_name='/xbotcore/robot_description_semantic', default='')
if srdf == '':
    raise print('srdf not set')

file_dir = os.getcwd()

'''
Initialize Horizon problem
'''
ns = 50
T = 3
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

kin_dyn = casadi_kin_dyn.CasadiKinDyn(urdf)

'''
Build ModelInterface and RobotStatePublisher
'''


print('XBot-RobotInterface not created.\n Using initial q default values.\n')
q_init = {'J1_A': 0.0,
          'J_wheel_A': 0.0,
          'J1_B': 0.0,
          'J_wheel_B': 0.0,
          'J1_C': 0.0,
          'J_wheel_C': 0.0,
          'J1_D': 0.0,
          'J_wheel_D': 0.0,
          'J1_E': 0.0,
          'J2_E': -0.5,
          'J3_E': 0.0,
          'J4_E': 0.5,
          'J5_E': 0.0,
          'J6_E': -0.5
          }


# this is a problem: if not starting at exactly zero, the solver won't move the base
q_init['J1_A'] = 0.01
q_init['J1_B'] = 0.01
q_init['J1_C'] = 0
q_init['J1_D'] = 0

base_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

wheel_radius = 0.16
FK = kin_dyn.fk('J_wheel_A')
init_pos_wheel = FK(q=kin_dyn.mapToQ(q_init))['ee_pos']
base_init[2] = -init_pos_wheel[2] + wheel_radius

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_init
                                 )


print(kin_dyn.joint_names())
# create robot description if robot is not found
if True: #not robot:
    rospy.set_param('mpc/robot_description', urdf)
    robot_state_pub_command = 'rosrun robot_state_publisher robot_state_publisher robot_description:=mpc/robot_description'
    robot_state_pub_process = subprocess.Popen(robot_state_pub_command.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml('concert_config.yaml')

ti.model.q.setBounds(ti.model.q0, ti.model.q0, nodes=0)
ti.model.v.setBounds(ti.model.v0, ti.model.v0, nodes=0)
ti.model.v.setBounds(ti.model.v0, ti.model.v0, nodes=ns)
# ti.model.a.setBounds(np.zeros([model.a.shape[0], 1]), np.zeros([model.a.shape[0], 1]), nodes=0)
ti.model.q.setInitialGuess(ti.model.q0)
ti.model.v.setInitialGuess(ti.model.v0)

prb.createResidual('max_q', 1e1 * utils.barrier(kin_dyn.q_max()[7:] - model.q[7:]))
prb.createResidual('min_q', 1e1 * utils.barrier1(kin_dyn.q_min()[7:] - model.q[7:]))

vel_lims = model.kd.velocityLimits()
prb.createResidual('max_vel', 1e2 * utils.barrier(vel_lims[7:] - model.v[7:]))
prb.createResidual('min_vel', 1e1 * utils.barrier1(-1 * vel_lims[7:] - model.v[7:]))

final_base_xy = ti.getTask('final_base_xy')
final_base_xy.setRef(np.array([[1., 1., 0, 0, 0, 0, 1]]).T)

# print(model.q)
model.q[13].setInitialGuess(np.pi/2, nodes=2)
model.q[15].setInitialGuess(np.pi/2, nodes=2)
model.q[17].setInitialGuess(np.pi/2, nodes=2)
model.q[19].setInitialGuess(np.pi/2, nodes=2)
# finalize taskInterface and solve bootstrap problem
ti.finalize()
ti.bootstrap()
ti.load_initial_guess()
solution = ti.solution

rate = rospy.Rate(1 / dt)

contact_list_repl = list(model.cmap.keys())

repl = replay_trajectory.replay_trajectory(dt, model.kd.joint_names(),
                                           solution['q'],
                                           {cname: solution[f.getName()] for cname, f in ti.model.fmap.items()},
                                           model.kd_frame, model.kd,
                                           trajectory_markers=contact_list_repl)
                                           # future_trajectory_markers={'base_link': 'world', 'J_wheel_D': 'world'})

repl.replay(is_floating_base=True)