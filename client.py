# Python client that feeds instructions to C++ robot controller

import subprocess
from matplotlib import pyplot as plt
from matplotlib import animation

def RepresentsFloat(s):
    try: 
        float(s)
        return True
    except ValueError:
        return False
proc = subprocess.Popen("./scl_orientation_control", stdin=subprocess.PIPE, stdout=subprocess.PIPE)

plt.ion()
ax = plt.axes(xlim=(0.0,.5),ylim=(.5,1))
line, = ax.plot([],[],lw=2)

def init():
    line.set_data([],[])


y_data = []
z_data = []



CANVAS_X = .589
def animate(i):
    while True:
      li = proc.stdout.readline()
      if li != '':
        #the real code does filtering here
        l = li.rstrip()
        l = l.split(" ")

        nums = [float(s) for s in l if RepresentsFloat(s)]
        if len(nums) < 3:
        	continue

        x = nums[1]
        print x
        if abs(x - CANVAS_X) > .02:
            continue
        y = nums[2]
        z = nums[3]
        y_data.append(y)
        z_data.append(z)
        line.set_data(y_data,z_data)
        print (y,z)

        return line,
      

#anim = animation.FuncAnimation(fig, animate, init_func=init,
#                              frames=200, interval=20, blit=True)

#anim.save('./basic_animation.mp4', fps=30)
while True:
    for i in range(100):
        animate(i)
        plt.draw()
    y_data = []
    z_data = []