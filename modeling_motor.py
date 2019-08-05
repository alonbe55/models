import matplotlib.pyplot as plt

dt = 0.01
vector = [0.0,0.0,0.0] #[angle, angular velocity,current_time]
w_prev = 0.0

def iteration(vector, voltage=12):
    torque = (0.2*voltage) - (0.0043*vector[1])
    a = ((0.2*voltage) - (0.0043*vector[1])) / 0.00007753
    vector[1] = vector[1] + (a*dt)
    #vector[1] = (46.513*voltage) - (232.565*torque)
    #vector[0] = vector[0] + (((vector[1]+w_prev)*dt)/2)
    #a = (vector[1]-w_prev)/dt
    plt.plot(vector[2], vector[1], 'bo')
    plt.draw()
    #plt.scatter(current_time,vector[1],s=100)
    plt.pause(0.0001)
    print(vector[1])
    #plt.clf()
    vector[2] = vector[2] + dt
    w_prev = vector[1]
    return vector

while True:
    vector = iteration(vector)
