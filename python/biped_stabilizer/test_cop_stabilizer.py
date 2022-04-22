#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 15:38:12 2022

@author: nvilla
"""

import biped_stabilizer as bs
bs.switchToNumpyArray()
import numpy as np
import cricket.talos_conf as conf

import unittest

class StabilizerTestCase(unittest.TestCase):
    def setUp(self):
        
        settings = dict(
                      height = conf.zk[0],
                      foot_width = conf.foot_corner[1],
                      foot_length = conf.foot_corner[0],
                      robot_mass = conf.mass,
                      dt = conf.tracking_period,
                      cop_x_gains = conf.centGain[0],
                      cop_y_gains = conf.centGain[0],
                      cop_p_cc_gain = 2.0,         #the gain used is [k, k/w]
                      integral_gain = np.array([0.0, 0.0]),
                      g = -conf.gravity[2],
                      cop_control_type = "p_cc",
                      saturate_cop = True,
                      use_rate_limited_dcm = False,
                      )
        tracker = bs.CopStabilizer()
        tracker.configure(settings)
        
        self.tracker = tracker
        self.settings = settings
        
    def test_constructor(self):
        
        internal = self.tracker.settings

        self.assertEqual(internal.height, self.settings["height"])
        self.assertEqual(internal.foot_width, self.settings["foot_width"])
        self.assertEqual(internal.foot_length, self.settings["foot_length"])
        self.assertEqual(internal.robot_mass, self.settings["robot_mass"])
        self.assertEqual(internal.dt, self.settings["dt"])
        self.assertEqual(internal.cop_x_gains, self.settings["cop_x_gains"])
        self.assertEqual(internal.cop_y_gains, self.settings["cop_y_gains"])
        self.assertEqual(internal.cop_p_cc_gain, self.settings["cop_p_cc_gain"])
        self.assertEqual(internal.integral_gain, self.settings["integral_gain"])
        self.assertEqual(internal.g, self.settings["g"])
        self.assertEqual(internal.cop_control_type, self.settings["cop_control_type"])
        self.assertEqual(internal.saturate_cop, self.settings["saturate_cop"])
        self.assertEqual(internal.use_rate_limited_dcm, self.settings["use_rate_limited_dcm"])

if __name__ == "__main__":
    
    unittest.main()
    

#arguments = dict(
#        actual_com = np.array([0,0,conf.zk[0]]),
#        actual_com_vel = np.zeros(3),
#        actual_com_acc = np.zeros(3),
#        actual_cop = np.zeros(3),
#        reference_com = np.array([0,0,conf.zk[0]]),
#        reference_com_vel = np.zeros(3),
#        reference_com_acc = np.zeros(3),
#        reference_com_jerk = np.zeros(3),
#        desired_com = np.zeros(3),
#        desired_com_vel = np.zeros(3),
#        desired_com_acc = np.zeros(3),
#        desired_icp = np.zeros(3),
#        actual_icp = np.zeros(3),
#        desired_cop_reference = np.zeros(3),
#        desired_cop_computed = np.zeros(3)
#        )





