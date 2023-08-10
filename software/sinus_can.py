'''
tflayols@laas.fr

This code sends positions commands to the motor controller, 
The state and commands data are published via UDP in json to be ploted in real time with plotjuggler (optional)
To plot the data, install and run plotjugler (UDP/JSON  9870)

Note to configure my CAN network interface:
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
candump can0 (optional - to see the trafic)
'''
import threading
import can
import time
from math import sin, pi
import struct
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
plotjuggler_address = ('localhost', 9870)  # Change as needed
q_latest = 0
q_dot_latest = 0
tau_latest = 0
lock = threading.Lock()

def receive_state_packet(bus):
    global q_latest, q_dot_latest, tau_latest
    while True:
        message = bus.recv(0) # Non-blocking read
        if message and message.arbitration_id == 0x456:  
            with lock:
                q_latest, q_dot_latest, tau_latest = struct.unpack('<lhh', message.data)
        
def pack_command_buffer(pos_gain, vel_gain, tau_ff, q_dot_d, q_d):
    assert 0 <= pos_gain <= 255
    assert 0 <= vel_gain <= 255
    assert 0 <= tau_ff <= 65535
    assert -511 <= q_dot_d <= 511
    assert -2097151 <= q_d <= 2097151
    q_dot_d += 511
    q_d += 2097151
    txCommandData = [0]*8
    txCommandData[0] = pos_gain
    txCommandData[1] = vel_gain
    txCommandData[2] = (q_dot_d >> 2) & 0xFF
    txCommandData[3] = ((q_dot_d & 0x3) << 6) | ((q_d >> 16) & 0x3F)
    txCommandData[4] = (q_d >> 8) & 0xFF
    txCommandData[5] = q_d & 0xFF
    txCommandData[6] = (tau_ff >> 8) & 0xFF
    txCommandData[7] = tau_ff & 0xFF
    return txCommandData

# Initialize the CAN bus
bus = can.interface.Bus(channel='can0', bustype='socketcan')
receive_thread = threading.Thread(target=receive_state_packet, args=(bus,))
receive_thread.daemon = True
receive_thread.start()

dt=0.001
T=10
f = 1

for i in range (int(T/dt)):
    t=i*dt
    q_d = int(100000*sin(2*pi*f*t));
    data = pack_command_buffer(1, 20, 0, 0, q_d)
    message = can.Message(arbitration_id=0x123, data=data, is_extended_id=False)
    bus.send(message)
    with lock:
        print (q_d, q_latest, q_dot_latest, tau_latest)
        msg = '{{"time": {}, "q_d": {}, "q": {}, "q_dot": {}, "tau": {}}}'.format(t, q_d, q_latest, q_dot_latest, tau_latest)
    sock.sendto(msg.encode('utf-8'), plotjuggler_address) #send to plotjuggler
    time.sleep(dt) #Not taking intoo account the loop execution time here..

bus.shutdown()
