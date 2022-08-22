#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 15:38:12 2022

@author: nvilla
"""

import biped_stabilizer as bs
from biped_stabilizer.biped_stabilizer_cpp import eMatrixHom
import numpy as np
import pinocchio as pin
from example_robot_data.robots_loader import load

import unittest

unittest.util._MAX_LENGTH = 2000


class TestCopStabilizer(unittest.TestCase):
    def setUp(self):
        # Create default settings
        settings = bs.CopStabilizerSettings()
        settings.height = 0.87  # [m]
        settings.foot_width = 0.1  # [m]
        settings.foot_length = 0.2  # [m]
        settings.robot_mass = 90.272  # [m]
        settings.dt = 0.002
        settings.g = 9.81
        settings.saturate_cop = True
        settings.use_rate_limited_dcm = False

        # Create default stab.
        stab = bs.CopStabilizer()

        self.stab = stab
        self.settings = settings

        LF = eMatrixHom()
        LF.set_translation(np.array([0.0, 0.09, 0.0]))
        LF.set_rotation(np.eye(3))

        RF = eMatrixHom()
        RF.set_translation(np.array([0.0, -0.09, 0.0]))
        RF.set_rotation(np.eye(3))
        arguments = dict(
            actual_stance_poses=[LF, RF],
            actual_com=np.array([0, 0, settings.height]),
            actual_com_vel=np.zeros(3),
            actual_com_acc=np.zeros(3),
            actual_cop=np.zeros(3),
            reference_com=np.array([0, 0, settings.height]),
            reference_com_vel=np.zeros(3),
            reference_com_acc=np.zeros(3),
            reference_com_jerk=np.zeros(3),
        )
        self.arguments = arguments

        # Pinocchio robot

        self.robot = load("talos")
        self.model = self.robot.model
        self.data = self.robot.data

    def test_constructor(self):
        assert bs.CopStabilizerSettings() == bs.CopStabilizerSettings()
        self.stab.configure(self.settings)

        self.assertNotEqual(self.stab.get_settings(), bs.CopStabilizerSettings())
        self.assertEqual(self.stab.get_settings(), self.settings)

    def test_pcc_stabilization(self):
        self.settings.cop_control_type = "p_cc"
        self.settings.cop_p_cc_gain = 3  # the gain used is [k, k/w]

        self.stab.configure(self.settings)
        w2 = self.settings.g / self.settings.height
        w = np.sqrt(w2)

        ## ~~~~ WITHOUT SATURATION ~~~~ ##
        error = [0.02, 0.02]
        self.arguments["actual_com"][:2] = error
        self.arguments["actual_cop"][:2] = error
        desired = stab_loop(self.stab, self.arguments, 1)

        self.assertTrue((desired["com"][:2] < error).all())
        self.assertTrue(
            (desired["cop"][:2] == self.settings.cop_p_cc_gain * np.array(error)).all()
        )

        n = desired["n"]
        self.assertTrue((np.abs(n) == 0).all())

        desired_1000 = stab_loop(self.stab, self.arguments, 1000)

        self.assertTrue(
            (desired_1000["com"][:2] + desired_1000["dcom"][:2] / w < 1e-6).all()
        )
        self.assertTrue((desired_1000["cop"][:2] < 1e-6).all())

        ## ~~~~ WITH STABLE SATURATION ~~~~ ##
        error2 = [0.07, 0.12]
        self.arguments["actual_com"][:2] = error2
        self.arguments["actual_cop"][:2] = error2
        self.arguments["actual_com_vel"] = np.zeros(3)
        self.arguments["actual_com_acc"] = np.zeros(3)
        desired = stab_loop(self.stab, self.arguments, 1)

        self.assertTrue((desired["com"][:2] < error2).all())
        self.assertTrue(
            (desired["cop"][:2] < self.settings.cop_p_cc_gain * np.array(error2)).all()
        )

        n = desired["n"]
        self.assertTrue((np.abs(n) == 0).all())

        desired_1000 = stab_loop(self.stab, self.arguments, 1000)

        self.assertTrue(
            (desired_1000["com"][:2] + desired_1000["dcom"][:2] / w < 1e-4).all()
        )
        self.assertTrue((desired_1000["cop"][:2] < 1e-4).all())

        ## ~~~~ WITH UNSTABLE SATURATION ~~~~ ##
        error3 = [0.1, 0.14]
        self.arguments["actual_com"][:2] = error3
        self.arguments["actual_cop"][:2] = error3
        self.arguments["actual_com_vel"] = np.zeros(3)
        self.arguments["actual_com_acc"] = np.zeros(3)
        desired = stab_loop(self.stab, self.arguments, 1)

        self.assertTrue((desired["com"][:2] == error3).all())
        self.assertTrue((desired["cop"][:2] == error3).all())

        n = desired["n"]
        self.assertTrue((np.abs(n) == 0).all())

        desired_1000 = stab_loop(self.stab, self.arguments, 1000)

        self.assertTrue(
            (desired_1000["com"][:2] + desired_1000["dcom"][:2] / w == error3).all()
        )
        self.assertTrue((desired_1000["cop"][:2] == error3).all())

    def test_pcc_stabilization_with_integral_term(self):
        self.settings.cop_control_type = "p_cc"
        self.settings.cop_p_cc_gain = 3.0  # the gain used is [k, k/w]
        self.settings.integral_gain = np.array([0.0001, 0.0001])

        self.stab.configure(self.settings)
        w2 = self.settings.g / self.settings.height
        w = np.sqrt(w2)

        ## ~~~~ WITHOUT SATURATION ~~~~ ##
        error = [0.02, 0.02]
        self.arguments["actual_com"][:2] = error
        self.arguments["actual_cop"][:2] = error
        desired = stab_loop(self.stab, self.arguments, 1)

        self.assertTrue((desired["com"][:2] < error).all())
        self.assertTrue(
            (desired["cop"][:2] > self.settings.cop_p_cc_gain * np.array(error)).all()
        )

        n = desired["n"]
        self.assertTrue((np.abs(n) == 0).all())

        desired_1000 = stab_loop(self.stab, self.arguments, 1000)

        self.assertTrue(
            (
                np.abs(desired_1000["com"][:2] + desired_1000["dcom"][:2] / w) < 1e-3
            ).all()
        )
        self.assertTrue((np.abs(desired_1000["cop"][:2]) < 1e-3).all())

    def test_jccc_stabilization(self):
        self.settings.cop_control_type = "j_ccc"
        self.settings.cop_x_gains = np.array([-4719.67, -2040.139, -194.824])
        self.settings.cop_y_gains = np.array([-4719.67, -2040.139, -194.824])

        self.stab.configure(self.settings)
        w2 = self.settings.g / self.settings.height
        w = np.sqrt(w2)

        ## ~~~~ WITHOUT SATURATION ~~~~ ##
        error = [0.02, 0.02]
        self.arguments["actual_com"][:2] = error
        self.arguments["actual_cop"][:2] = error
        desired = stab_loop(self.stab, self.arguments, 1)

        self.assertTrue((desired["com"][:2] < error).all())
        self.assertTrue((desired["cop"][:2] > desired["com"][:2]).all())

        n = desired["n"]
        self.assertTrue((np.abs(n) == 0).all())

        desired_1000 = stab_loop(self.stab, self.arguments, 1000)

        self.assertTrue(
            (desired_1000["com"][:2] + desired_1000["dcom"][:2] / w < 1e-6).all()
        )
        self.assertTrue((desired_1000["cop"][:2] < 1e-6).all())

        ## ~~~~ WITH STABLE SATURATION ~~~~ ##
        error2 = [0.07, 0.12]
        self.arguments["actual_com"][:2] = error2
        self.arguments["actual_cop"][:2] = error2
        self.arguments["actual_com_vel"] = np.zeros(3)
        self.arguments["actual_com_acc"] = np.zeros(3)
        desired = stab_loop(self.stab, self.arguments, 1)

        self.assertTrue((desired["com"][:2] < error2).all())
        self.assertTrue((desired["cop"][:2] > desired["com"][:2]).all())

        n = desired["n"]
        self.assertTrue((np.abs(n) < 1e-15).all())

        desired_1000 = stab_loop(self.stab, self.arguments, 1000)

        self.assertTrue(
            (desired_1000["com"][:2] + desired_1000["dcom"][:2] / w < 1e-5).all()
        )
        self.assertTrue((desired_1000["cop"][:2] < 1e-5).all())

        # ~~~~ WITH UNSTABLE SATURATION ~~~~ ##
        error3 = [0.1, 0.14]
        self.arguments["actual_com"][:2] = error3
        self.arguments["actual_cop"][:2] = error3
        self.arguments["actual_com_vel"] = np.zeros(3)
        self.arguments["actual_com_acc"] = np.zeros(3)
        desired = stab_loop(self.stab, self.arguments, 1)

        self.assertTrue((desired["com"][:2] == error3).all())
        self.assertTrue((desired["cop"][:2] == error3).all())

        n = desired["n"]
        self.assertTrue((n == 0).all())

        desired_1000 = stab_loop(self.stab, self.arguments, 1000)

        self.assertTrue(
            (desired_1000["com"][:2] + desired_1000["dcom"][:2] / w == error3).all()
        )
        self.assertTrue((desired_1000["cop"][:2] == error3).all())

    def test_jccc_stabilization_with_integral_term(self):
        self.settings.cop_control_type = "j_ccc"
        self.settings.cop_x_gains = np.array([-4719.67, -2040.139, -194.824])
        self.settings.cop_y_gains = np.array([-4719.67, -2040.139, -194.824])
        self.settings.integral_gain = np.array([-0.1, -0.1])

        self.stab.configure(self.settings)
        w2 = self.settings.g / self.settings.height
        w = np.sqrt(w2)

        ## ~~~~ WITHOUT SATURATION ~~~~ ##
        error = [0.02, 0.02]
        self.arguments["actual_com"][:2] = error
        self.arguments["actual_cop"][:2] = error
        desired = stab_loop(self.stab, self.arguments, 1)

        self.assertTrue((desired["com"][:2] < error).all())
        self.assertTrue((desired["cop"][:2] > desired["com"][:2]).all())

        n = desired["n"]
        self.assertTrue((np.abs(n) < 1e-15).all())

        desired_1000 = stab_loop(self.stab, self.arguments, 1000)

        self.assertTrue(
            (
                np.abs(desired_1000["com"][:2] + desired_1000["dcom"][:2] / w) < 1e-3
            ).all()
        )
        self.assertTrue((np.abs(desired_1000["cop"][:2]) < 1e-3).all())


def stab_loop(tracker, arguments, iterations, printing=False):
    settigs = tracker.get_settings()
    w2 = settigs.g / settigs.height

    for i in range(iterations):
        results = tracker.stabilize(arguments)
        print_loop_results(results, arguments, w2, printing)

        arguments["actual_com"] = results[0]
        arguments["actual_com_vel"] = results[1]
        arguments["actual_com_acc"] = results[2]
        arguments["actual_cop"] = results[6]

    desired = dict(
        com=results[0],
        dcom=results[1],
        ddcom=results[2],
        cop=results[6],
        n=np.hstack([results[6][:2] - results[0][:2] + results[2][:2] / w2, 0]),
    )

    return desired


def print_loop_results(results, arguments, w2, printing=False):
    if printing:
        print("----------------------------")
        print("DCM: ", results[3][:2])
        print("B:   ", results[0][:2] - results[2][:2] / w2)
        print("CoP: ", results[6][:2])
        print(
            "rCP: ",
            arguments["reference_com"][:2] - arguments["reference_com_acc"][:2] / w2,
        )
        print(
            "n:   ",
            np.hstack([results[6][:2] - results[0][:2] + results[2][:2] / w2, 0]),
        )


if __name__ == "__main__":

    unittest.main()
