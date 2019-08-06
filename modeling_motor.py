import matplotlib.pyplot as plt

def draw_function(x,y): # TODO: add head lines
    plt.plot(x, y, 'bo')
    plt.title('v/s graph')
    plt.xlabel('time [s]')
    plt.ylabel('angular velocity [m/s]')
    plt.draw()
    plt.pause(0.0001)

def update_vector(vector, voltage, dt):
    j = 0.00007753 # moment of inertia [kg*m*m]
    R = 0.09 #resistance of cim motor
    kv = 46.513 # ratio between voltage and angular velocity
    I = (voltage/0.09) + (vector[1]/4.18617) # calculated from v = R*I + (omega/kv)
    kt = 0.018 # ratio between I and torque
    torque = (0.2*voltage) - (0.0043*vector[1]) #calculated from torque = I*kt
    a = ((0.2*voltage) - (0.0043*vector[1])) / j # calculated from j = torque / a
    vector[1] = vector[1] + (a*dt)
    return vector

def update_time(current_time, dt):
    return (current_time + dt)

def main():
    current_time = 0
    dt = 0.01 #change of time between interation
    vector = [0.0,0.0] #[angle, angular velocity]
    while current_time < 5:
        vector = update_vector(vector, 12, dt)
        draw_function(current_time, vector[1])
        current_time = update_time(current_time, dt)


if __name__ == '__main__':
    main()
