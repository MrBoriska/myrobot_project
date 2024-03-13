import select
import time
import os

ob = open("/sys/class/gpio/gpio72/value")

pollerObject = select.poll()
pollerObject.register(ob, select.POLLPRI|select.POLLERR)

counter = 0
t0 = time.time()
while 1:
    events = pollerObject.poll(150)
    if not events:
        print ("Velocity:", '0')
    for fd, flag in events:
        
        if counter > 10:
            val = counter/(time.time() - t0)
            print("Velocity:", val)
            t0 = time.time()
            counter = 0
        
        clicked = 0
        os.lseek(fd, 0, 0)
        val = os.read(fd, 1)
        if val == b'0':
            counter += 1
        