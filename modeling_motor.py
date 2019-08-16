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
        return vector[0] + (np.random.normal(0,1,1))

def update_vector(vector, voltage, dt):
    j = 0.00007753 # moment of inertia [kg*m*m]
    R = 0.09 #resistance of cim motor
    kv = 46.513 # ratio between voltage and angular velocity
    I = (voltage/R) - (vector[1]/(kv*R)) # calculated from v = R*I + (omega/kv)
    kt = 0.018 # ratio between I and torque
    torque = I * kt #calculated from torque = I*kt
    angular_acceleration = (torque / j) + np.random.normal(0,0.5,1) # calculated from j = torque / a
    vector[1] = vector[1] + (angular_acceleration*dt)
    vector[0] = vector[0] + (vector[1]*dt)
    #vector[1] = vector[1] + (((acceleration + acceleration_prev)*dt)/2)
    #vector[0] = vector[0] + (((vector[1] + omega_prev)*dt)/2)
    #omega_prev = vector[1]
    #acceleration_prev = vector[2]
    return vector

def update_vector_hat(vector_hat, vector, voltage, dt):
    j = 0.00007753 # moment of inertia [kg*m*m]
    R = 0.09 #resistance of cim motor
    kv = 46.513 # ratio between voltage and angular velocity
    I = (voltage/R) - (vector[1]/(kv*R)) # calculated from v = R*I + (omega/kv)
    kt = 0.018 # ratio between I and torque
    torque = I * kt #calculated from torque = I*kt
    angular_acceleration = torque / j # calculated from j = torque / a
    l1 = 0.5
    l2 = 0.5
    error = vector[0] - encoder(vector[0])
    vector_hat[1] = vector_hat[1] + (angular_acceleration*dt) + (error*l1)
    vector_hat[0] = vector_hat[0] + (vector_hat[1]*dt) + (error*l2)
    return vector_hat

def update_time(current_time, dt):
    return (current_time + dt)

def main():
    current_time = 0
    dt = 0.01 #change of time between interations
    vector = [0.0,0.0] #[angle, angular velocity]
    vector_hat = [0.0,0.0] #[angle hat, angular velocity hat]
    angular_acceleration = 0
    omega_prev = 0
    acceleration_prev = 0
    encoder_value = 0
    encoder_value_prev = 0
    while current_time < 5:
        encoder_value = encoder(vector)
        vector = update_vector(vector, 12, dt)
        vector_hat = update_vector_hat(vector_hat, vector, 12, dt)
        #draw_function(current_time, vector[1], 'omega/s graph', 'time [s]', 'angular velocity [radians/s]')
        #draw_function(current_time, vector[0], 'x/s graph', 'time [s]', 'angle[radians]')
        #draw_function(current_time, encoder_value, 'enc/s graph', 'time [s]', 'encoder[radians]')
        #draw_function(current_time, (encoder_value-encoder_value_prev)/dt, 'omega/s graph', 'time [s]', 'angular velocity[radians/s]', current_time, vector[1])
        draw_function(current_time, vector[1], 'combined omega/s graph', 'time [s]', 'angular velocity [radians/s]', current_time, (encoder_value-encoder_value_prev)/dt, current_time, vector_hat[1])
        encoder_value_prev = encoder_value
        current_time = update_time(current_time, dt)


if __name__ == '__main__':
    main()
