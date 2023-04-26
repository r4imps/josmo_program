from kf import 
import numpy as np

# Create a Kalman filter object
kf = KalmanFilter(dim_x=2, dim_z=1)

# Define the state transition matrix
dt = 1.0  # time step
kf.F = np.array([[1, dt], [0, 1]])

# Define the measurement function
kf.H = np.array([[1, 0]])

# Define the process noise covariance matrix
q = 0.1  # process noise standard deviation
kf.Q = np.array([[0.25*dt**4*q, 0.5*dt**3*q], [0.5*dt**3*q, dt**2*q]])

# Define the measurement noise covariance matrix
r = 0.1  # measurement noise standard deviation
kf.R = np.array([[r]])

# Define the initial state estimate
x0 = np.array([0, 0])
kf.x = x0

# Define the initial covariance matrix
kf.P = np.array([[1, 0], [0, 1]])

# Generate some measurements
measurements = [1, 2, 3, 4, 5]

# Run the Kalman filter
filtered_states = []
for z in measurements:
    kf.predict()
    kf.update(z)
    filtered_states.append(kf.x)

# Print the filtered states
print(filtered_states)