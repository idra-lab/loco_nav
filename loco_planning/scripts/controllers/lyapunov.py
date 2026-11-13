import numpy as np
import math
from utils.math_tools import unwrap_angle
from utils.math_tools import Math

class LyapunovParams:
    def __init__(self, K_P, K_THETA, DT=0.001, ESTIMATE_ALPHA_WITH_ACTUAL_VALUES = False, lyapunov_function_type="lecture"):
        self.K_P = K_P
        self.K_THETA = K_THETA

        self.DT = DT
        self.lyapunov_function_type=lyapunov_function_type
class Robot:
    pass

class LyapunovController:
    def __init__(self, params: LyapunovParams): #, matlab_engine = None):
        #EXE 4
        self.K_Y = 20. #20.
        self.K_P = params.K_P
        self.K_THETA = params.K_THETA
        self.log_e_x = []
        self.log_e_y = []
        self.log_e_theta = []
        self.lyapunov_function_type = params.lyapunov_function_type

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

        if self.lyapunov_function_type=="class":
            # this is the implementation seen in the class:
            # V = 1 / 2 * (ex^2 + ey^2+ etheta^2)
            # V_dot = -K_THETA * etheta^2 - K_P * exy^2 * cos(psi - theta)^2
            # np.sinc is the NORMALIZED sinc: np.sinc(x) = sin(pi*x)/(pi*x). We need sin(etheta/2)/(etheta/2),
            # so the argument must be divided by pi (this was the source of the ~15% mismatch with matlab which does sin(x)/x)
            domega = - self.K_Y * v_d * np.sinc(0.5 * etheta / np.pi) * exy * math.sin(psi - 0.5 * beta) -self.K_THETA * etheta
            # equivalent but without sinc it is undefined at etheta=0!
            # domega =  -2/etheta * v_d * exy* np.sin(0.5 * etheta)* np.sin(psi - 0.5 * beta) - self.K_THETA * etheta
            V = 1 / 2 * (ex ** 2 + ey ** 2 + etheta ** 2)
            V_dot = -self.K_THETA * etheta ** 2 - self.K_P * exy ** 2 * math.pow(math.cos(psi - theta), 2)
        else:
            # this is the implementation implemented in the paper:
            # V = 1 / 2  * (ex^2 + ey^2) + (1 -  cos(etheta))
            # V_dot = -K_P * exy^2 * cos(theta - psi)^2 - K_THETA * sin(etheta)^2
            domega = - self.K_Y * v_d * exy * (1/np.cos(etheta/2)) * np.sin(psi - (beta/2)) - self.K_THETA * np.sin(etheta)
            V = 1 / 2 * (ex ** 2 + ey ** 2) + (1 - np.cos(etheta))
            V_dot = -self.K_P * (exy ** 2) * (np.cos(theta - psi) ** 2) - self.K_THETA * (np.sin(etheta) ** 2)

        v = v_d + dv
        omega = omega_d + domega


        # save errors for plotting
        self.log_e_x.append(ex)
        self.log_e_y.append(ey)
        self.log_e_theta.append(etheta)

        # print("ERRORS -> x:%.2f, y:%.2f, theta:%.2f" % (ex, ey, etheta))
        # print("VELS -> v:%.2f, o:%.2f" % (v_ref + dv, o_ref + domega))
        return v, omega

