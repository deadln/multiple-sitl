import sys
import time
import keyboard


timeout = float(sys.argv[1])
t = time.time()
while True:
    dt = time.time() - t
    if dt > timeout:
        keyboard.write('ekf2 stop')
        keyboard.press_and_release('enter')
        time.sleep(2)
        keyboard.write('ekf2 start')
        keyboard.press_and_release('enter')
        t = time.time()
        print("RESTART EKF2")
