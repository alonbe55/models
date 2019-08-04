import matplotlib.pyplot as plt

current_time = 0.0
dt = 0.01
vector = [0.0,0.0] #[angle, angular velocity]
w_prev = 0.0

def iteration(voltage=12, torque=0):
    global current_time, dt, vector, w_prev
    vector[1] = (46.513*voltage) - (232.565*torque)
    #vector[0] = vector[0] + (((vector[1]+w_prev)*dt)/2)
    #a = (vector[1]-w_prev)/dt
    plt.plot(current_time, vector[1], 'bo')
    plt.draw()
    #plt.scatter(current_time,vector[1],s=100)
    plt.pause(0.0001)
    print(vector[1])
    #plt.clf()
    current_time = current_time + dt
    w_prev = vector[1]

while True:
    iteration()
