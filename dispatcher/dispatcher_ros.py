import rospy 
from dispatcher import *


def readkey(getchar_fn=None):
    fd=sys.stdin.fileno()

    old_settings=termios.tcgetattr(fd)
    try:

        tty.setraw(sys.stdin.fileno())
        ch=sys.stdin.read(1)
        
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)

    return ch

if __name__ == "__main__":
         

        (pipe_socket,pipe_ros)=multiprocessing.Pipe()

        sock_client = SockClient('192.168.3.149', 30006, pipe_socket)
        car_dispatcher=CarDispatcherROS(pipe_ros)   

        t1 = multiprocessing.Process(target=sock_client.run) 
        t2 = multiprocessing.Process(target=car_dispatcher.run)
        
        car_dispatcher.start()
        sock_client.start()
        while True:
            key=readkey()
            
            if key=='q' or key=='Q' or key=='\x03':
                os._exit(0)     
        sock_client.join()
        car_dispatcher.join()
        print('Terminated')
