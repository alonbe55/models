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
    #plt.draw()
    plt.pause(0.0001)

def encoder(vector):
        #C = [1,0]
        #return (C[0]*vector[0]) + (C[1]*vector[1]) + (np.random.normal(0,1,1))
        C = np.array((1,0))
        return C.dot(vector)

def calculate_system_matrices(voltage, dt, j, stall_current, stall_torque, free_omega, free_current):
    R = 1
    R = float(R)
    R = float(voltage) / float(stall_current) # calculated from v = R*I + (omega/kv) when omega=0
    Kv = free_omega / (voltage-(R*free_current)) # calculated from v = R*I + (omega/kv)
    Kt = stall_torque / stall_current # ratio between I and torque
    A = np.array([(1, dt), (0, 1-(((Kt*dt)/(Kv*R))/j))])
    B = np.array([[0] , [(Kt*dt)/(R*j)]])
    B.reshape((2,1))
    C = np.array((1,0))
    #A = [[1, dt], [0, 1-(((kt*dt)/(kv*R))/j)]]
    #B = [0, (kt*dt)/(R*j)]
    #C = [1,0]
    return [A,B,C]

def update_vector(vector, voltage, dt):
    j = 0.00007753 # moment of inertia [kg*m*m]
    #R = 0.09 #resistance of cim motor
    #kv = 46.513 # ratio between voltage and angular velocity
    #I = (voltage/R) - (vector[1]/(kv*R)) # calculated from v = R*I + (omega/kv)
    #kt = 0.018 # ratio between I and torque
    #A = [[1, dt], [0, 1-(((kt*dt)/(kv*R))/j)]]
    #B = [0, (kt*dt)/(R*j)]
    #torque = I * kt #calculated from torque = I*kt
    #angular_acceleration = (torque / j) + np.random.normal(0,0.5,1) # calculated from j = torque / a
    #vector[1] = (A[1][1]*vector[1]) + (B[1]*voltage)
    #vector[0] = (A[0][0]*vector[0]) + (A[0][1]*vector[1]) + (B[0]*voltage)
    system_matrices = calculate_system_matrices(12,0.01, j, 131, 2.41, 558.156, 2.7)
    vector = system_matrices[0].dot(vector) + system_matrices[1].dot(voltage)
    return vector

def update_vector_hat(vector_hat, vector, voltage, dt):
    j = 0.00007753 # moment of inertia [kg*m*m]
    #R = 0.09 #resistance of cim motor
    #kv = 46.513 # ratio between voltage and angular velocity
    #I = (voltage/R) - (vector_hat[1]/(kv*R)) # calculated from v = R*I + (omega/kv)
    #kt = 0.018 # ratio between I and torque
    #torque = I * kt #calculated from torque = I*kt
    #angular_acceleration = torque / j # calculated from j = torque / a
    l = np.array([[0], [10]]) #l1,l2
    l.reshape((2,1))
    #l1 = 0 # correct value 0
    #l2 = 10 # correct value 10
    #error = encoder(vector) - vector_hat[0]
    system_matrices = calculate_system_matrices(12,0.01, j, 131, 2.41, 558.156, 2.7)
    error = encoder(vector) - (system_matrices[2].dot(vector_hat))
    error = float(error)
    vector_hat = system_matrices[0].dot(vector_hat) + system_matrices[1].dot(voltage) + l.dot(error)
    #vector_hat[0] = vector_hat[0] + (vector_hat[1]*dt) + (error*l1)
    #vector_hat[1] = vector_hat[1] + (angular_acceleration*dt) + (error*l2)
    return vector_hat

def update_time(current_time, dt):
    return (current_time + dt)

def main():
    current_time = 0
    dt = 0.01 #change of time between interations
    #vector = [0.0,0.0] #[angle, angular velocity]
    vector = np.array([[0.0], [0.0]]) #[angle, angular velocity]
    vector.reshape((2,1))
    #vector_hat = [0.0,0.0] #[angle hat, angular velocity hat]
    vector_hat = np.array([[0.0], [0.0]]) #[angle, angular velocity]
    vector_hat.reshape((2,1))
    #angular_acceleration = 0
    #omega_prev = 0
    #acceleration_prev = 0
    #encoder_value = 0
    #encoder_value_prev = 0
    while current_time < 5:
        encoder_value = encoder(vector)
        vector = update_vector(vector, 120, dt)
        vector_hat = update_vector_hat(vector_hat, vector, 12, dt)
        #draw_function(current_time, vector[1], 'omega/s graph', 'time [s]', 'angular velocity [radians/s]')
        #draw_function(current_time, vector[0], 'x/s graph', 'time [s]', 'angle[radians]')
        #draw_function(current_time, encoder_value, 'enc/s graph', 'time [s]', 'encoder[radians]')
        #draw_function(current_time, (encoder_value-encoder_value_prev)/dt, 'omega/s graph', 'time [s]', 'angular velocity[radians/s]', current_time, vector[1])
        #draw_function(current_time, vector[1], 'combined omega/s graph', 'time [s]', 'angular velocity [radians/s]', current_time, (encoder_value-encoder_value_prev)/dt, current_time, vector_hat[1])
        draw_function(current_time, vector[0][0], 'x/s graph', 'time [s]', 'angle[radians]', current_time, vector_hat[0][0])
#        draw_function(current_time, vector[0], 'x/s graph', 'time [s]', 'angle[radians]', current_time, vector_hat[0])
        encoder_value_prev = encoder_value
        current_time = update_time(current_time, dt)


if __name__ == '__main__':
    main()
