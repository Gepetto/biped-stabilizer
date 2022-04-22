#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 15:38:12 2022

@author: nvilla
"""

import biped_stabilizer
import numpy as np
import talos_conf as conf

import unittest
unittest.util._MAX_LENGTH = 2000

class TestCopStabilizer(unittest.TestCase):
    def setUp(self):
        # Create default settings        
        self.settings = biped_stabilizer.CopStabilizerSettings()
        self.settings.height = conf.zk[0]
        self.settings.foot_width = conf.foot_corner[1]
        self.settings.foot_length = conf.foot_corner[0]
        self.settings.robot_mass = conf.mass
        self.settings.dt = conf.tracking_period
        self.settings.cop_x_gains = conf.centGain[0]
        self.settings.cop_y_gains = conf.centGain[0]
        self.settings.cop_p_cc_gain = 2.0         #the gain used is [k, k/w]
        self.settings.integral_gain = np.array([0.0, 0.0])
        self.settings.g = -conf.gravity[2]
        self.settings.cop_control_type = "p_cc"
        self.settings.saturate_cop = True
        self.settings.use_rate_limited_dcm = False

        # Create default stab.
        self.stab = biped_stabilizer.CopStabilizer()
        self.stab.configure(self.settings)

        assert biped_stabilizer.CopStabilizerSettings() == biped_stabilizer.CopStabilizerSettings()
        
    def test_constructor(self):
        self.assertNotEqual(self.stab.get_settings(), biped_stabilizer.CopStabilizerSettings())
        self.assertEqual(self.stab.get_settings(), self.settings)

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
