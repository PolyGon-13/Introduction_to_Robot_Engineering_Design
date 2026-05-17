import serial
import time
import math
from threading import Thread
from queue import Queue

lidar_ser = serial.Serial("/dev/ttyUSB0", 460800, timeout=0.1)
arduino_ser = serial.Serial("/dev/ttyS0", 115200, timeout=0.1)

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
            target_angle_deg -= 10
        else : target_angle_deg += 10
        
    if  -35 < target_angle_deg < 35 :
        Kp_ang = 7.26
    
    else : Kp_ang = 6.91
        
    angle_err_rad = math.radians(target_angle_deg)
    w = -Kp_ang * angle_err_rad
    phi_l = (v / wheel_R) - ((w * wheel_l) / (2 * wheel_R))
    phi_r = (v / wheel_R) + ((w * wheel_l) / (2 * wheel_R))
    
    
    if phi_l < 10:
        if phi_l < 0: phi_l = 10
        else : phi_l = 10
        
    
    if phi_r < 10:
        if phi_r < 0: phi_r = 10
        else : phi_r = 10
        
    return phi_l, phi_r


distance_array = [0] * 181


def average_valid_distance(distances, start_angle, end_angle, max_wall_distance=2000):
    valid_distances = []

    for angle in range(start_angle, end_angle + 1):
        distance = distances[angle]
        if 50 <= distance <= max_wall_distance:
            valid_distances.append(distance)

    if len(valid_distances) < 3:
        return None

    return sum(valid_distances) / len(valid_distances)


def corridor_center_correction(distances):
    right_wall = average_valid_distance(distances, 25, 75)
    left_wall = average_valid_distance(distances, 105, 155)

    if right_wall is None or left_wall is None:
        return 0.0, right_wall, left_wall

    wall_error = right_wall - left_wall
    correction = wall_error * 0.025

    return max(-18.0, min(18.0, correction)), right_wall, left_wall


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
        
    desired_angle = Follow_the_Gap_Method(distance_array, threshold=400)

    target_angle = desired_angle - 90
    center_correction, right_wall, left_wall = corridor_center_correction(distance_array)
    target_angle += center_correction
    target_angle = max(-70.0, min(70.0, target_angle))
    
    phi_l, phi_r = compute_wheel_phis(target_angle)
    
    print(f"[DETECT] Angle: {target_angle:.2f}, correction: {center_correction:.2f}, right: {right_wall}, left: {left_wall}, phi_l: {phi_l:.2f}mm, phi_r: {phi_r:.2f} mm")
    
    try:
        data_queue.put_nowait((phi_l, phi_r))
    except:
        pass
