#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Feb  7 13:44:49 2021

@author: nvilla
"""
import numpy as np
from example_robot_data.robots_loader import getModelPath
import os

np.set_printoptions(precision=15, linewidth=500, suppress=True)

############################################################################### SWITCHES
                    ## ~ WB control ~ ##
use_feedback = False # in joints
compensate_deflections = True

                    ## ~ tracker ~ ## 
full_information = False
state_updated_at_every_step = False
exact_deflection = False

                    ## ~ reference motion ~ ##
reference_online = True
closed_loop_MPC = True
always_horizontal_ground = True
rised_arms = True 
with_terminal_constraint = True
with_orientation = False

maker_name = (
#                "gecko"
                "bindings"
#                "wrapper"
        )

                    ## ~ environment ~ ##
solid_obstacle = False

############################################################################### SIMULATION

simu_period = 1 / 1000  # 1/600#       # [s] simulation sampling period
simu_duration = 8390  # 2400   # [iterations] Total duration of the simulation

############################################################################### REFERENCE

system = (
#        "dP->CCP"
        "J->CCC"
#        "P->CC"
        )

#    --->      FOR THE MAKER:

# constraints

feet_sep = 0.08  # [m] minimum separation between feet
foot_rising = 0.3  # [m]
foot_corner = np.array([0.1, 0.05])
stepping_corner = np.array([0.3, 0.1])
stepping_center = np.array([0, 2 * foot_corner[1] + stepping_corner[1] + feet_sep])


cop_safety_margin = 0.02

# Initial state  (for systems ->CCC) 
strt_y = feet_sep/2 + foot_corner[1]
xk = np.array([0, 0, 0]).T
yk = np.array([strt_y, 0, 0]).T
zk = np.array([0.87, 0, 0]).T

LF_translation = np.array([0., strt_y, 0]).T
RF_translation = np.array([0, -strt_y, 0]).T
LF_rotation = np.eye(3)
RF_rotation = np.eye(3)

# timing 

num_steps = 2

mpc_period = 0.1  # [s] nominal sampling period
mpc_ratio = int(round(mpc_period / simu_period, 0))

ss_duration = round(11*mpc_period, 5)
ds_duration = mpc_period

takeoff_delay = 0.05  
landing_advance = 0.05 + ds_duration

regular_step_duration = round(ss_duration + ds_duration, 5)  # number of nominal periods for step
step_samples = int(round(regular_step_duration / mpc_period ))
horizon_lenght = num_steps * step_samples  # number of iterations in preview horizon


# costs
#cost_weights = {"minimize jerk": 0.0001,
#                "track velocity": 0.01,
#                "relax ankles": 0.1}
#cost_weights = {"minimize jerk": 0.001,
#                "track velocity x": 0.05,
#                "track velocity y": 0.02,
#                "relax ankles x": 2,
#                "relax ankles y": 2, 
#                "terminal":1}

target_vel = np.array([1.8, 0, 0])


#optimization domain

cost_weights = {"minimize jerk": 0.001,
                "track velocity": 0.01,
                "relax ankles": 1,
                "terminal":0}


if closed_loop_MPC:
    optimization_domain = ["CoM_dddot_x", "Ds_x", "CoM_dddot_y", "Ds_y", "x0_x", "x0_y"]
    
else:
    optimization_domain = ["CoM_dddot_x", "Ds_x", "CoM_dddot_y", "Ds_y"]
     
#   --->       FOR THE READER:

Dt = 0.00005  # [s] time used to compute numerical derivatives
cs_name = (
             "step_in_place_quasistatic_REF.cs"
#             "step_in_place_REF.cs"
#             'walk_20cm_quasistatic_REF.cs'
#             "walk_20cm_REF.cs"
#             'talos_stairs10_REF.cs'
#             'platforms_REF.cs'
#             "com_motion_above_feet_REF.cs"
)

if cs_name[-18:-7] == "quasistatic":
    we_compute_the_angular_momentum = True
else:
    we_compute_the_angular_momentum = False

############################################################################### TRACKING

 #
if system == "P->CC":
    centGain = np.array([[3.6, 1.07]])
#    int_gainXY = np.array([0.005, 0])
elif system == "dP->CCP":
    centGain = np.array([[567.98368, 167.02088, -195.30274]])
#    int_gainXY = np.array([0.5, 0])
elif system == "Pd->CCP":
    centGain = np.array([[20.424, 20.424/3.344, -3.341940442366789], #33.476, 33.476/3.344, -3.228
                         [0    , 0    , 0]])
elif system == "J->CCC":
#    centGain = np.array([[-17754.810682462194 ,  -6926.7248917669485,   -493.0388860703101]])
    centGain = np.array([[-4719.669152989827, -2040.1393274991938, -194.82405448186182]])


tracking_period = 5 / 1000  # 1*(1/240)#17*(1/240),#0.01,# (4/1000)
tracking_ratio = int(round(tracking_period / simu_period, 0))

############################################################################### WBC

lz = 0.0 
mu = 1  # friction coefficient
gu = 1  # Tortioanal friction
fMin = 1.0  # minimum normal force
fMax = 1000.0  # maximum normal force
# direction of the normal to the contact surface
contactNormal = np.array([0.0, 0.0, 1.0])

rf_frame_name = "leg_right_sole_fix_joint"  # right foot frame name
lf_frame_name = "leg_left_sole_fix_joint"  # left foot frame name

# TASKS
w_com = 0  # 1.0        # weight of center of mass task
w_foot = 1  # weight of the foot motion task
w_posture = 3e-1  # weight of joint posture task
w_forceRef = 1e-1  # weight of force regularization task
w_rootOrientation = 10  # weight of rootOrientation task
w_am = 2e-2  # (negative means not used)# weight used for the tracking of the Angular momentum
w_waist = 100  # 100             # waist position task
w_cop = 1
w_contact = 0.0  # weight of foot in contact

YAW_ROT_GAIN = 1  # if smaller than 1, make easier turn on the z axis than in xy
# CONSTRAINTS
w_torque_bounds = 0.0  # weight of the torque bounds(it is a Constraint)
w_joint_bounds = 0.0  # (it is a Constraint)

# Weights used internally in the distribution of forces
w_reg = 1e-3  # 1e-2           #weight of force regularization
w_ForReg = 1e-2  # weight multiplyer to regulate force
w_TorReg = 3e3  # weight multiplyer to regulate torques
w_euler = 100  # weight of Euler Equation

level_rootOrientation = 0
level_am = 1
level_waist = 1
level_com = 1
level_posture = 1
level_foot = 0
level_cop = 1
level_contact = 1

# Tracking Gains
kp_contact = 10.0  # proportional gain of contact constraint
kp_foot = 100.0  # proportional gain of contact constraint
kd_foot = 2.0 * np.sqrt(kp_foot)
kp_rootOrientation = 1000.0  # proportional gain of the root's orientation task
# kp_com = np.array([0, 0, 10.0])#10.0 #          # proportional gain of center of mass task
kp_posture = 1.0  # proportional gain of joint posture task
kp_am = 2.0  # gain used for the tracking of the Angular momentum
kp_waist = 0#20  # gain use for the waist position task
kp_com = 0

gain_vector = [
    10.0,
    5.0,
    5.0,
    1.0,
    1.0,
    10.0,
] * 2  # legs  #low gain on axis along y and knee
gain_vector += [5000.0, 5000.0]  # chest
gain_vector += [40.0, 40.0, 40.0, 100.0, 150.0, 150.0, 500.0, 100.0] * 2  # arms
gain_vector += [100.0, 100.0]  # head
gain_vector = np.array(gain_vector)

masks_posture = np.ones(32)
# masks_posture[[4, 5, 10, 11]] = 0

initial_posture = "half_sitting"

tau_max_scaling = 1  # scaling factor of torque bounds
v_max_scaling = 1

viewer = None  # pin.visualize.GepettoVisualizer#

# Gains for the joint feedback #
kp = np.array(([2, 9, 9] + [0, 0, 0]) * 2 + [10] * 2  # Legs
              + ([10, 10, 10] + [10, 1, 1, 1, 0.1]) * 2  # Torso
              + [0, 0]  # Arms
              )  # Head2
kv = 2 * np.sqrt(kp)


# URDF address:
flex_urdf = "/talos_data/robots/talos_reduced_flex.urdf"  #
urdf = "/talos_data/robots/talos_reduced_corrected.urdf"  #
path = getModelPath(urdf)
urdf = path + urdf
srdf = path + "/talos_data/srdf/talos.srdf"
path = os.path.join(path, "../..")

# PHYSICAL SIMULATION
model_name = (
#        "talos"
        "talos_flex"
#        "talos_flex_act"
        )

# Flexibility Parameters

if model_name == "talos_flex" or model_name == "talos_flex_act":
    
    H_stiff = [2200, 2200, 6200, 6200]# [LH_pitch, LH_roll, RH_pitch, RH_roll]
    H_damp = 2 * np.sqrt(H_stiff)
    
    # Number of times that the flexibility is computed in each control period
    flex_ratio = round(4.5 + 8.5e-4 * (max(H_stiff)**2) / 10000)
    
elif model_name == "talos_damper":
    k_flex =  5000  # 
    d_flex = 2 * np.sqrt(k_flex)
    # Number of times that the flexibility is computed in each control period
    flex_ratio = round(4.5 + 8.5e-4 * (k_flex**2) / 10000)

else:
    H_stiff = [np.inf, np.inf, np.inf, np.inf]# [LH_pitch, LH_roll, RH_pitch, RH_roll]
    H_damp = [np.inf, np.inf, np.inf, np.inf]
    flex_ratio = 1
    
flex_esti_delay = 0.0  # [s]
flex_error = 0.0  # error fraction such that: estimation = real*(1-flex_error)

flex_esti_delay = 0.0  # [s]
flex_error = 0.0  # error fraction such that: estimation = real*(1-flex_error)



############################################################################### PHYSICS

# structure: [initial, final, fx, fy, fz]

planned_push = [[(0, 10000*simu_period)], 
                [np.zeros(6)], 
                ["base_link"]]

gravity = [0, 0, -9.81]
com_height = 0.877  # [m] CoM height
mass = 90.272  # [kg] robot mass # It is modified by the configuration of tsid to match the robot model mass.
omega = np.sqrt(-gravity[2] / com_height)  # [1/s] LIP time constant
gamma = 50  # [1/s] ground interaction time const  # TODO: BORRAR

