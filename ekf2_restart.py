import pyatspi
import time
import keyboard


def name_apps():
    desktop = pyatspi.Registry.getDesktop(0)
    print('there are %s running applications' % (desktop.childCount))
    for app in desktop:
        print(app.name)


def show_windows():
    desktop = pyatspi.Registry.getDesktop(0)
    for app in desktop:
        print("APP", app)
        for window in app:
            print("WINDOW",window)
            for x in window:
                print(x)


t = time.time()
while True:
    dt = time.time() - t
    if dt > 30:
        keyboard.write('ekf2 stop')
        keyboard.press_and_release('enter')
        time.sleep(2)
        keyboard.write('ekf2 start')
        keyboard.press_and_release('enter')
        t = time.time()
