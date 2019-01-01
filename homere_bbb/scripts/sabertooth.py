
import time, serial, struct


cmd_drive_forward_m1  = 0x00
cmd_drive_backward_m1 = 0x01
cmd_drive_forward_m2  = 0x04
cmd_drive_backward_m2 = 0x05

cmd_serial_timeout    = 0x0e
cmd_deadband          = 0x11

class SaberTooth:
    def __init__(self):
        self.device = '/dev/ttyS1'
        self.ser = serial.Serial(port=self.device, baudrate=9600)
        self.addr = 128
        self.set_serial_timeout(100)
        self.set_deadband(1)
        
    def set_serial_timeout(self, milisec):
        n = int(milisec/100)
        self.send_cmd(cmd_serial_timeout, n)

    def set_deadband(self, deadband):
        self.send_cmd(cmd_deadband, deadband)
        
        
    def send(self, m1, m2):
        ''' m1, m2: range -127:127 '''
        cmd, data = (cmd_drive_forward_m1, m1) if m1 >=0 else (cmd_drive_backward_m1, -m1)
        self.send_cmd(cmd, data)
        cmd, data = (cmd_drive_forward_m2, m2) if m2 >=0 else (cmd_drive_backward_m2, -m2)
        self.send_cmd(cmd, data)

    def send_cmd(self, cmd, data):
        checksum = self.addr+cmd+data & 0x7f
        s = struct.pack('cccc', chr(self.addr), chr(cmd), chr(data), chr(checksum))
        self.ser.write(s)
        
    def quit(self):
        self.send(0, 0)
        self.ser.close()





if __name__ == '__main__':
    mc = SaberTooth() 
    while True:
        mc.send(10, 10)
        time.sleep(0.05)
