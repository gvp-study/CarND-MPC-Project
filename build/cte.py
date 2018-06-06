import matplotlib
import matplotlib.pyplot as plt

cte = []
epsi = []
steer = []
v = []
n = 0
with open('./cte.txt', "r") as file:
    for line in file:
#        if((len(line.split()) > 2) & (n < 3200)):
        if((len(line.split()) > 4)):
           cte.append(float(line.split()[1]))
           epsi.append(float(line.split()[2]))
           steer.append(float(line.split()[3]))
           v.append(float(line.split()[4]))
           n = n + 1
cte = cte[1:];        
epsi = epsi[1:];        
epsi = [i/25.0 for i in epsi]
steer = steer[1:];        
v = v[1:];        
v = [i/100.0 for i in v]

fig = plt.figure(1)
plt.plot(cte, label='CTE')
plt.plot(epsi, label='Error Psi')
plt.plot(steer, label='Steering')
plt.plot(v, label='Velocity')
plt.ylabel('Value')
plt.xlabel('Time')
plt.title('CTE, EPSI, Steering and Velocity vs Time')
plt.grid()
plt.legend(loc='upper left')
#plt.plot(cte)

fig.show()
