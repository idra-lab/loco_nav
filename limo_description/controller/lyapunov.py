import numpy as np
import math
from base_controllers.utils.math_tools import unwrap_angle
from base_controllers.tracked_robot.utils import constants
from base_controllers.utils.math_tools import Math
from scipy.optimize import fsolve

def unwrap_angle(angle_meas, angle_old):
    angle_unwrapped = angle_meas
    while (angle_unwrapped < (angle_old - np.pi)):
            angle_unwrapped += 2 * np.pi
    while (angle_unwrapped > (angle_old + np.pi)):
        angle_unwrapped -= 2 * np.pi
    angle_old = angle_unwrapped
    return angle_unwrapped, angle_old

class LyapunovParams:
    def __init__(self, K_P, K_THETA, DT=0.001, ESTIMATE_ALPHA_WITH_ACTUAL_VALUES = False):
        self.K_P = K_P
        self.K_THETA = K_THETA
        self.DT = DT
class Robot:
    pass

class LyapunovController:
    def __init__(self, params: LyapunovParams): #, matlab_engine = None):

        self.K_P = params.K_P
        self.K_THETA = params.K_THETA
        self.log_e_x = []
        self.log_e_y = []
        self.log_e_theta = []

        self.theta_old = 0.
        self.v_old = 0.
        self.omega_old = 0.

        self.params = params
        self.math_utils = Math()

    def getErrors(self):
        return self.log_e_x, self.log_e_y,  self.log_e_theta

    def control_unicycle(self, actual_state, current_time, des_x, des_y, des_theta, v_d, omega_d, traj_finished):
        """
        ritorna i valori di linear e angular velocity
        """
        if traj_finished:
            # save errors for plotting
            self.log_e_x.append(0.0)
            self.log_e_y.append(0.0)
            self.log_e_theta.append(0.0)
            return 0.0, 0.0, 0., 0.

        # compute errors
        # 3d case
        if hasattr(actual_state, 'roll'):
            # first compute the errors in the worlf frame (equivalent to horizontal frame)
            hf_ex = actual_state.x - des_x
            hf_ey = actual_state.y - des_y
            hf_R_b = self.math_utils.eul2Rot(np.array([actual_state.roll, actual_state.pitch, 0.]))
            # map from hf to bf
            b_exy = hf_R_b.T.dot(np.array([hf_ex, hf_ey, 0]))
            # discard the z component
            ex = b_exy[0]
            ey = b_exy[1]
            # map theta  along base z axis
            w_z_b = self.math_utils.eul2Rot(np.array([actual_state.roll, actual_state.pitch, actual_state.theta]))[:, 2]
            theta, self.theta_old = unwrap_angle(actual_state.theta, self.theta_old)
            w_etheta = theta - des_theta
            w_beta = theta + des_theta
            etheta = w_z_b.dot(np.array([0., 0., w_etheta]))
            beta = w_z_b.dot(np.array([0., 0., w_beta]))
        else:  # 2d case
            ex = actual_state.x - des_x
            ey = actual_state.y - des_y
            theta, self.theta_old = unwrap_angle(actual_state.theta, self.theta_old)
            etheta = theta - des_theta
            beta = theta + des_theta

        #compute ausiliary variables
        psi = math.atan2(ey, ex)
        exy = math.sqrt(ex**2 + ey**2)

        dv = -self.K_P * exy * math.cos(psi - theta)
        # important ! the result is 15% different for the sinc function with python from matlab
        # domega = -self.K_THETA * etheta -v_d * np.sinc(0.5 * etheta) * exy * math.sin(psi - 0.5 * beta)
        domega = -v_d * exy * (1/np.cos(etheta/2)) * np.sin(psi - (beta/2)) - self.K_THETA * np.sin(etheta)

        v = v_d + dv
        omega = omega_d + domega

        # V = 1 / 2 * (ex ** 2 + ey ** 2+ etheta**2)
        # V_dot = -self.K_THETA * etheta**2  - self.K_P * exy * math.pow(math.cos(psi - theta),2)
        V = 1 / 2 * (ex ** 2 + ey ** 2) + (1 - np.cos(etheta))
        V_dot = -self.K_P * (exy ** 2) * (np.cos(theta - psi) ** 2) - self.K_THETA * (np.sin(etheta) ** 2)

        #domega = - self.K_THETA * etheta - 2/etheta * v_d * np.sin(0.5 * etheta)* np.sin(psi - 0.5 * beta)
        # save errors for plotting

        self.log_e_x.append(ex)
        self.log_e_y.append(ey)
        self.log_e_theta.append(etheta)

        # print("ERRORS -> x:%.2f, y:%.2f, theta:%.2f" % (ex, ey, etheta))
        # print("VELS -> v:%.2f, o:%.2f" % (v_ref + dv, o_ref + domega))
        return v, omega

