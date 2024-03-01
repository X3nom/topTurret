import threading
import time

def testThread():
    currTime()

def currTime():
    print(time.time())

threading.Thread(target=testThread).start()