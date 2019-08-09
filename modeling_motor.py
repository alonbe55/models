import matplotlib.pyplot as plt
import random

def draw_function(x,y, title, title_x, title_y):
    plt.plot(x, y, 'bo')
    plt.title(title)
    plt.xlabel(title_x)
    plt.ylabel(title_y)
    plt.draw()
    plt.pause(0.0001)

def encoder(vector, dt):
        return vector[0] + (random.random()*0.01)

def update_vector(vector, voltage, dt):
    j = 0.00007753 # moment of inertia [kg*m*m]
    R = 0.09 #resistance of cim motor
    kv = 46.513 # ratio between voltage and angular velocity
    I = (voltage/R) + (vector[1]/(kv*R)) # calculated from v = R*I + (omega/kv)
    kt = 0.018 # ratio between I and torque
    torque = ((kt/R)*voltage) - ((kt/(kv*R))*vector[1]) #calculated from torque = I*kt
    vector[2] = torque / j # calculated from j = torque / a
    vector[1] = vector[1] + (vector[2]*dt)
    vector[0] = vector[0] + (vector[1]*dt)
    print(vector[1])
    #vector[1] = vector[1] + (((acceleration + acceleration_prev)*dt)/2)
    #vector[0] = vector[0] + (((vector[1] + omega_prev)*dt)/2)
    #omega_prev = vector[1]
    #acceleration_prev = vector[2]
    return vector

def update_time(current_time, dt):
    return (current_time + dt)

def main():
    current_time = 0
    dt = 0.01 #change of time between interations
    vector = [0.0,0.0, 0.0] #[angle, angular velocity, angular acceleration]
    omega_prev = 0
    acceleration_prev = 0
    while current_time < 5:
        vector = update_vector(vector, 12, dt)
        #draw_function(current_time, vector[1], 'v/s graph', 'time [s]', 'angular velocity [m/s]')
        #draw_function(current_time, vector[0], 'x/s graph', 'time [s]', 'angle[radians]')
        draw_function(current_time, encoder(vector, dt), 'enc/s graph', 'time [s]', 'encoder[radians]')
        #draw_function(current_time, vector[2], 'a/s graph', 'time [s]', 'acceleration [radians/(s*s)]')
        current_time = update_time(current_time, dt)


if __name__ == '__main__':
    main()
