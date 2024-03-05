import threading
import time
import queue

def testThread(inQ):
    inQ.get_nowait()
    
    currTime()

def currTime():
    print(time.time())

inQ = queue.Queue()
threading.Thread(target=testThread, args=[inQ]).start()