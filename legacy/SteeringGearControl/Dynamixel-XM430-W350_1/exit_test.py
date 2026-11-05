import atexit

def exit_handler():
    print ('My application is ending!')

atexit.register(exit_handler)
while True:
    pass