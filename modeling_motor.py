import matplotlib.pyplot as plt
import numpy as np
import random

def draw_function(x,y, title, title_x, title_y, x1 = None, y1 = None, x2 = None, y2 = None):
    plt.plot(x, y, 'bo')
    if x1 != None :
        plt.plot(x1,y1,'ro')
        if x2 != None :
            plt.plot(x2,y2,'yo')
    plt.title(title)
    plt.xlabel(title_x)
    plt.ylabel(title_y)
    plt.pause(0.0001)

def encoder(vector, C):
        return C.dot(vector) + (np.random.normal(0,1,1))

def build_model(voltage_measurement, dt, j, stall_current, stall_torque, free_omega, free_current):
    # voltage_measurement is the voltage applied on the motor during the measurements of stall_torque, stall_current....
    R = float(voltage_measurement) / float(stall_current) # calculated from v = R*I + (omega/kv) when omega=0
    Kv = free_omega / (voltage_measurement-(R*free_current)) # calculated from v = R*I + (omega/kv)
    Kt = stall_torque / stall_current # ratio between I and torque
    A = np.array([(1, dt), (0, 1-(((Kt*dt)/(Kv*R))/j))])
    B = np.array([[0] , [(Kt*dt)/(R*j)]])
    C = np.array((1,0))
    return (A,B,C)

def update_vector(vector, voltage, dt, system_matrices):
    (A,B,C) = system_matrices
    vector = A.dot(vector) + B.dot(voltage)
    return vector

def update_vector_hat(vector_hat, vector, voltage, dt, system_matrices):
    l = np.array([[0], [10]]) #l1,l2
    error = encoder(vector, system_matrices[2]) - (system_matrices[2].dot(vector_hat))
    error = float(error)
    (A,B,C) = system_matrices
    vector_hat = A.dot(vector_hat) + B.dot(voltage) + l.dot(error)
    return vector_hat

def update_time(current_time, dt):
    return (current_time + dt)

def main():
    current_time = 0
    dt = 0.01 #change of time between interations
    vector = np.array([[0.0], [0.0]]) #[angle, angular velocity]
    vector.reshape((2,1))
    vector_hat = np.array([[0.0], [0.0]]) #[angle, angular velocity]
    vector_hat.reshape((2,1))
    while current_time < 5:
        encoder_value = encoder(vector, np.array((1,0)))
        vector = update_vector(vector, 120, dt, build_model(12,0.01, 0.00007753, 131, 2.41, 558.156, 2.7))
        vector_hat = update_vector_hat(vector_hat, vector, 12, dt, build_model(12,0.01, 0.00007753, 131, 2.41, 558.156, 2.7))
        draw_function(current_time, vector[0][0], 'x/s graph', 'time [s]', 'angle[radians]', current_time, vector_hat[0][0])
        encoder_value_prev = encoder_value
        current_time = update_time(current_time, dt)


if __name__ == '__main__':
    main()
