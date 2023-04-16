#main.py
import time
import socket
import struct
import numpy as np
from cal.robotiq_gripper import RobotiqGripper
from cal.realsenseD435i import Camera
from cal.frametrans import tf
from cal.quest2 import VR
from ultralytics import YOLO


class UR_Robot:
    def __init__(self, tcp_host_ip="192.168.1.8", tcp_port=30003):
        # Init
        self.tcp_host_ip = tcp_host_ip
        self.tcp_port = tcp_port
        
        
        self.model=YOLO("/home/jiaming/test01/runs/detect/train/weights/best.pt")

        # UR5 robot configuration
        # Default joint/tool speed configuration
        self.joint_acc = 1.4  # Safe: 1.4   8
        self.joint_vel = 1.05  # Safe: 1.05  3

        # Joint tolerance for blocking calls
        self.joint_tolerance = 0.01

        # Default tool speed configuration
        self.tool_acc = 0.5  # Safe: 0.5
        self.tool_vel = 0.2  # Safe: 0.2

        # Tool pose tolerance for blocking calls
        self.tool_pose_tolerance = [0.002, 0.002, 0.002, 0.01, 0.01, 0.01]

        # robotiq85 gripper configuration
        self.gripper = RobotiqGripper()
        self.gripper.connect(self.tcp_host_ip, 63352)  # don't change the 63352 port
        self.gripper._reset()
        self.gripper.activate()
        time.sleep(1.5)
        
        # realsense configuration
        self.camera = Camera()
        
        self.vr=VR()

        self.home_joint_config = [-(0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                             (0 / 360.0) * 2 * np.pi, -(90 / 360.0) * 2 * np.pi,
                             -(0 / 360.0) * 2 * np.pi, 0.0]

    def move_j(self, joint_configuration,k_acc=1,k_vel=1,t=0,r=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        tcp_command = "movej([%f" % joint_configuration[0]  #"movej([]),a=,v=,\n"
        for joint_idx in range(1,6):
            tcp_command = tcp_command + (",%f" % joint_configuration[joint_idx])
        tcp_command = tcp_command + "],a=%f,v=%f,t=%f,r=%f)\n" % (k_acc*self.joint_acc, k_vel*self.joint_vel,t,r)
        self.tcp_socket.send(str.encode(tcp_command))
    
        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(1500)
        actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
        while not all([np.abs(actual_joint_positions[j] - joint_configuration[j]) < self.joint_tolerance for j in range(6)]):
            state_data = self.tcp_socket.recv(1500)
            actual_joint_positions = self.parse_tcp_state_data(state_data, 'joint_data')
            time.sleep(0.01)
        self.tcp_socket.close()
    # joint control


    #input:tool_configuration=[x y z r p y]
    def move_j_p(self, tool_configuration,k_acc=1,k_vel=1,t=0,r=0):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        print(f"movej_p([{tool_configuration}])")
        # command: movej([joint_configuration],a,v,t,r)\n
        tcp_command = "def process():\n"
        tcp_command +=" array = rpy2rotvec([%f,%f,%f])\n" %(tool_configuration[3],tool_configuration[4],tool_configuration[5])
        tcp_command += "movej(get_inverse_kin(p[%f,%f,%f,array[0],array[1],array[2]]),a=%f,v=%f,t=%f,r=%f)\n" % (tool_configuration[0],
            tool_configuration[1],tool_configuration[2],k_acc * self.joint_acc, k_vel * self.joint_vel,t,r ) # "movej([]),a=,v=,\n"
        tcp_command += "end\n"
        self.tcp_socket.send(str.encode(tcp_command))

        # Block until robot reaches home state
        state_data = self.tcp_socket.recv(1500)
        actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
        while not all([np.abs(actual_tool_positions[j] - tool_configuration[j]) < self.tool_pose_tolerance[j] for j in
                       range(3)]):
            state_data = self.tcp_socket.recv(1500)
            # print(f"tool_position_error{actual_tool_positions - tool_configuration}")
            actual_tool_positions = self.parse_tcp_state_data(state_data, 'cartesian_info')
            time.sleep(0.01)
        time.sleep(1.5)
        self.tcp_socket.close()
        
        
    def go_home(self):
        self.move_j(self.home_joint_config)
        
    def get_state(self):
        self.tcp_cket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))
        state_data = self.tcp_socket.recv(1500)
        self.tcp_socket.close()
        return state_data
    
    # get robot current joint angles and cartesian pose
    def parse_tcp_state_data(self, data, subpasckage):
        dic = {'MessageSize': 'i', 'Time': 'd', 'q target': '6d', 'qd target': '6d', 'qdd target': '6d',
               'I target': '6d',
               'M target': '6d', 'q actual': '6d', 'qd actual': '6d', 'I actual': '6d', 'I control': '6d',
               'Tool vector actual': '6d', 'TCP speed actual': '6d', 'TCP force': '6d', 'Tool vector target': '6d',
               'TCP speed target': '6d', 'Digital input bits': 'd', 'Motor temperatures': '6d', 'Controller Timer': 'd',
               'Test value': 'd', 'Robot Mode': 'd', 'Joint Modes': '6d', 'Safety Mode': 'd', 'empty1': '6d',
               'Tool Accelerometer values': '3d',
               'empty2': '6d', 'Speed scaling': 'd', 'Linear momentum norm': 'd', 'SoftwareOnly': 'd',
               'softwareOnly2': 'd',
               'V main': 'd',
               'V robot': 'd', 'I robot': 'd', 'V actual': '6d', 'Digital outputs': 'd', 'Program state': 'd',
               'Elbow position': 'd', 'Elbow velocity': '3d'}
        ii = range(len(dic))
        for key, i in zip(dic, ii):
            fmtsize = struct.calcsize(dic[key])
            data1, data = data[0:fmtsize], data[fmtsize:]
            fmt = "!" + dic[key]
            dic[key] = dic[key], struct.unpack(fmt, data1)

        if subpasckage == 'joint_data':  # get joint data
            q_actual_tuple = dic["q actual"]
            joint_data= np.array(q_actual_tuple[1])
            return joint_data
        elif subpasckage == 'cartesian_info':
            Tool_vector_actual = dic["Tool vector actual"]  # get x y z rx ry rz
            cartesian_info = np.array(Tool_vector_actual[1])
            return cartesian_info

    def get_current_tool_pos(self):
        return self.gripper.get_current_position()
      
    def log_gripper_info(self):
        print(f"Pos: {str(self.gripper.get_current_position())}")
    
    def close_gripper(self,speed=255,force=255):
        # position: int[0-255], speed: int[0-255], force: int[0-255]
        self.gripper.move_and_wait_for_pos(255, speed, force)
        print("gripper had closed!")
        time.sleep(1.2)
        self.log_gripper_info()
    
    def open_gripper(self,speed=255,force=255):
        # position: int[0-255], speed: int[0-255], force: int[0-255]
        self.gripper.move_and_wait_for_pos(0, speed, force)
        print("gripper had opened!")
        time.sleep(1.2)
        self.log_gripper_info()
    
    def get_camera_data(self):
        color_img, depth_img = self.camera.get_data()
        return color_img, depth_img
    
    def detect_cube(self):
        position=[]
        img=self.get_camera_data[1]
        results = self.model.predict(source=img, save=True, save_txt=True)
        boxes=results[0][0].boxes
        for i in range(len(boxes)):
            box = boxes[i]  # returns one box
            if box.conf>0.8:
                x=box.xywh[1]+box.xywh[3]*0.5
                y=box.xywh[2]+box.xywh[4]
                position.append([int(box.cls),x,y])
        return position
    
    #frame transfermation
    #[x,y]to[x,y,z,rx,ry,rz]
    def framestrans(self):
        position=self.detect_cube()
        for i in range(len(position)):
            if position[i][0]==0: 
                r=tf(position[i][1],position[i][2])
            elif position[i][0]==1: 
                y=tf(position[i][1],position[i][2])
            elif position[i][0]==2: 
                b=tf(position[i][1],position[i][2])
            elif position[i][0]==3: 
                g=tf(position[i][1],position[i][2])
                
        #send to VR
        self.vr.send(r,y,b,g)
        return r,y,b,g
    
    def start(self, open_size=0.85, k_acc=0.8, k_vel=0.8, speed=255, force=125):

        positions=self.framestrans()        
        r0=positions[0]
        y0=positions[1]
        b0=positions[2]
        g0=positions[3]
        grasp_home = [0, -0.3, -0.2, 0,np.pi, 0]
        box=[0, -0.5, -0.3, 0,np.pi, 0]
        r1=r0
        r1[3]=r1[3]+0.1
        y1=y0
        y1[3]=y1[3]+0.1
        b1=b0
        b1[3]=b1[3]+0.1
        g1=g0
        g1[3]=g1[3]+0.1
        
        self.move_j_p(grasp_home, k_acc, k_vel)
        self.open_gripper(speed, force)
        
        times=4
        while times>0:
            #signal
            #0-none 1-r 2-y 3-b 4-g
            signal=self.vr.rec()
            if signal==0:
                continue
            elif signal==1:
                p0=r0
                p1=r1
                print("r")
            elif signal==2:
                p0=y0
                p1=y1
                print("y")
            elif signal==3:
                p0=b0
                p1=b1
                print("b")
            elif signal==4:
                p0=g0
                p1=g1
                print("g")
            
            #move and grasp
            self.move_j_p(p1, k_acc, k_vel)
            self.move_j_p(p0, k_acc, k_vel)
            self.close_gripper(speed, force)
            self.move_j_p(p1, k_acc, k_vel)
            self.move_j_p(box, k_acc, k_vel)
            self.open_gripper(speed, force)
            self.move_j_p(grasp_home, k_acc, k_vel)
            print("done!")
            
            times=times-1
    
if __name__ =="__main__":
    ur_robot = UR_Robot()
    ur_robot.start()
