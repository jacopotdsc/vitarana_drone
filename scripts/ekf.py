import numpy as np
import time
from math import sin, cos
import sympy as sp

class KalmanFilter:
    def __init__(self, d, Q, H_gps, H_imu, R_gps, R_imu, P):
        
        ###### ----------- defining symbolic variables ---------- ######
        x, y, z = sp.symbols('x y z')
        v_x, v_y, v_z = sp.symbols('v_x v_y v_z')
        theta, phi, psi = sp.symbols('theta phi psi')
        u_x, u_y, u_z = sp.symbols('u_x u_y u_z')
        u_theta, u_phi, u_psi = sp.symbols('u_theta u_phi, u_psi')

        self.state = sp.Matrix([x, y, z, v_x, v_y, v_z, theta, phi, psi])
        self.control = sp.Matrix([u_x, u_y, u_z, u_theta, u_phi, u_psi])

        ###### ----------- computing roll-pitch-yaw rotation ---------- ######
        roll = sp.Matrix([
            [1, 0, 0],
            [0, sp.cos(theta), -sp.sin(theta)],
            [0, sp.sin(theta), sp.cos(theta)]
        ])

        pitch = sp.Matrix([
            [sp.cos(phi), 0, sp.sin(phi)],
            [0, 1, 0],
            [-sp.sin(phi), 0, sp.cos(phi)]
        ])

        yaw = sp.Matrix([
            [sp.cos(psi), -sp.sin(psi), 0],
            [sp.sin(psi), sp.cos(psi), 0],
            [0, 0, 1]
        ])

        rpy = yaw @ pitch @ roll
        
        ###### ----------- composing transition function ---------- ######
        current_position = sp.Matrix([x, y, z])
        current_velocity = sp.Matrix([v_x, v_y, v_z])
        current_attitude = sp.Matrix([theta, phi, psi])
        control_linear = sp.Matrix([u_x, u_y, u_z])
        control_angular = sp.Matrix([u_theta, u_phi, u_psi])

        next_position = current_position + rpy @ control_linear
        next_velocity = current_velocity + control_linear
        next_attitude = current_attitude + control_angular

        next_state = sp.Matrix.vstack(next_position, next_velocity, next_attitude)

        ###### ----------- jacobian for EKF ---------- ######
        self.A = next_state.jacobian(self.state)  
        self.B = next_state.jacobian(self.control)  
        
        ###### ----------- state variables ---------- ######
        self.d = d # drift term

        self.Q = Q # process noise covariance
        self.P = P # estimate error covariance

        self.x = None

        self.H_gps = H_gps # observation matrix for gps
        self.H_imu = H_imu # observation matrix for imu

        self.R_gps = R_gps # measurement noise covariance of gps
        self.R_imu = R_imu # measurement noise covariance of imu

        self.sensor_type = {'imu': 'imu', 
                            'gps': 'gps'}

    def compute_A_B(self, current_state, current_control):

        substitutions = {self.state[i]: current_state[i][0] for i in range(len(self.state))}
        substitutions.update({self.control[i]: current_control[i][0] for i in range(len(self.control))})

        A_numeric = np.array(self.A.subs(substitutions).evalf(), dtype=float)
        B_numeric = np.array(self.B.subs(substitutions).evalf(), dtype=float)

        return A_numeric, B_numeric

    def predict(self, current_state, current_control):

        A, B = self.compute_A_B(current_state, current_control)
        x = current_state
        u = current_control

        self.x = A @ x + B @ u + self.d
        self.P = A @ self.P @ A.T + self.Q
        return self.x, self.P

    def update(self, z, sensor_type):

        if sensor_type == self.sensor_type['imu']:
            H = self.H_imu
            R = self.R_imu
        elif sensor_type == self.sensor_type['gps']:
            H = self.H_gps
            R = self.R_gps
        else:
            print("--- No sensor recognized ---")

        # kalman gain
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # update the state estimate
        y = z - H @ self.x
        self.x = self.x + K @ y
        
        # update the estimate covariance
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ H) @ self.P
        
        return self.x, self.P
    

if __name__ == '__main__':

    dt = 0.1  # Passo temporale
    state_dim = 9

    # Stato iniziale: [x, y, z, vx, vy, vz, roll, pitch, yaw]
    x_init = np.zeros((state_dim, 1))

    # Rumore di processo
    Q = np.eye(state_dim) * 0.01

    # Matrici di osservazione per GPS e IMU
    H_gps = np.zeros((3, state_dim))
    H_gps[:3, :3] = np.eye(3)  # Osserva solo [x, y, z]

    H_imu = np.zeros((3, state_dim))
    H_imu[:3, 6:] = np.eye(3)  # Osserva solo [roll, pitch, yaw]

    # Covarianze del rumore di misura (GPS meno preciso dell'IMU)
    R_gps = np.eye(3) * 0.1
    R_imu = np.eye(3) * 0.05

    # Matrice di covarianza dell'errore iniziale
    P = np.eye(state_dim) * 0.1

    # Creazione dell'istanza EKF
    ekf = KalmanFilter(
        d=np.zeros((state_dim, 1)),  # Drift term nullo
        Q=Q, 
        H_imu=H_imu,
        H_gps=H_gps,
        R_gps=R_gps, 
        R_imu=R_imu, 
        P=P
    )

    gps_measurements = [
        np.array([[1.0], [2.0], [3.0]]),  
        np.array([[1.1], [2.1], [3.2]]),
        np.array([[1.2], [2.2], [3.3]])
    ]

    imu_measurements = [
        np.array([[0.01], [0.02], [0.01]]),  
        np.array([[0.02], [0.03], [0.02]]),
        np.array([[0.03], [0.04], [0.03]])
    ]

    current_state = x_init.copy()
    current_control = np.zeros((6, 1))

    for i in range(len(gps_measurements)):
        print("\n🔹 Iterazione:", i + 1)

        # Simulazione del controllo attuale (esempio: spinta verticale)
        current_control = np.array([[0.0], [0.0], [-9.81], [0.01], [0.02], [0.03]])

        # PREDIZIONE
        ekf.predict(current_state, current_control)
        print("Predizione dello stato:", ekf.x)

        # AGGIORNAMENTO GPS
        if gps_measurements[i] is not None:
            print("Posizione prima aggiornamento GPS:\n", ekf.x.flatten())
            ekf.update(gps_measurements[i], 'gps')
            print("Posizione dopo aggiornamento GPS:\n", ekf.x.flatten())

        # AGGIORNAMENTO IMU
        if imu_measurements[i] is not None:
            print("Orientazione prima aggiornamento IMU:\n", ekf.x.flatten())
            ekf.update(imu_measurements[i], 'imu')
            print("Orientazione dopo aggiornamento IMU:\n", ekf.x.flatten())

        time.sleep(dt)
    

