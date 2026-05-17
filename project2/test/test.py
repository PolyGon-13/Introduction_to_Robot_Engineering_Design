import serial
import time
import math
from threading import Thread
from queue import Queue

lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)
arduino_ser = serial.Serial("/dev/ttyS0", 9600, timeout=0.1)

data_queue = Queue(maxsize=10)
send_enable = False

def arduino_writer():
    while True:
        angle, distance = data_queue.get()
        msg = f"{angle:.2f},{distance:.2f}\n".encode()
        arduino_ser.write(msg)
        time.sleep(0.01)

thread = Thread(target=arduino_writer, daemon=True)
thread.start()

lidar_ser.write(bytes([0xA5, 0x40]))
time.sleep(1)
lidar_ser.write(bytes([0xA5, 0x20]))

wheel_R = 0.034
wheel_l = 0.179


def compute_wheel_phis(target_angle_deg: float):
    v = 0.73
    if -8 <= target_angle_deg <= 8:
        v = 1.2
        print("v is best!")
    if target_angle_deg != 0 :
        if target_angle_deg < 0 :
            target_angle_deg - 10
        else : 
            target_angle_deg + 10
        
    if  -35 < target_angle_deg < 35 :
        Kp_ang = 7.26
    
    else : Kp_ang = 6.91
        
    angle_err_rad = math.radians(target_angle_deg)
    w = -Kp_ang * angle_err_rad
    phi_l = (v / wheel_R) - ((w * wheel_l) / (2 * wheel_R))
    phi_r = (v / wheel_R) + ((w * wheel_l) / (2 * wheel_R))
    
    
    if phi_l < 4.1:
        if phi_l < 0: phi_l = 2.25
        else : phi_l = 4.1
        
    
    if phi_r < 4.1:
        if phi_r < 0: phi_r = 2.25
        else : phi_r = 4.1
        
    return phi_l, phi_r


distance_array = [0] * 181
def Follow_the_Gap_Method(distances, threshold):
    max_tmp = 0
    tmp = 0
    best_start = 0
    best_end = 0
    current_start = 0
    for angle in range(181):
        if distances[angle] >= threshold:
            if tmp == 0:
                current_start = angle
            tmp += 1
        else:
            if tmp > max_tmp:
                max_tmp = tmp
                best_start = current_start
                best_end = angle - 1
            tmp = 0
    if tmp > max_tmp:
        max_tmp = tmp
        best_start = current_start
        best_end = 180
        
    mid_angle = (best_start + best_end) / 2.0
    return mid_angle

while True:
    data = lidar_ser.read(5)
    
    if len(data) != 5:
        continue
        
    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    
    if s_inv_flag != (1 - s_flag):
        continue
        
    check_bit = data[1] & 0x01
    
    if check_bit != 1:
        continue
        
    quality = data[0] >> 2
    
    angle_q6 = ((data[1] >> 1) | (data[2] << 7))
    raw_angle = angle_q6 / 64.0
    
    angle = raw_angle + 90
    
    if angle >= 360:
        angle -= 360
        
    distance_q2 = (data[3] | (data[4] << 8))
    distance = distance_q2 / 4.0
    
    if distance < 50:
        continue
        
    angle_index = int(angle)
    
    if 0 <= angle_index <= 180:
        distance_array[angle_index] = distance
        
    desired_angle = Follow_the_Gap_Method(distance_array, threshold=700)

    target_angle = desired_angle - 90
    
    phi_l, phi_r = compute_wheel_phis(target_angle)
    
    print(f"[DETECT] Angle: {target_angle:.2f}, phi_l: {phi_l:.2f}mm, phi_r: {phi_r:.2f} mm")
    
    try:
        data_queue.put_nowait((phi_l, phi_r))
    except:
        pass
