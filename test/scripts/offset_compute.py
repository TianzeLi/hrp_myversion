#! /usr/bin/python3

from math import sqrt


# Check points
cp = [[3.8, 25.3],  
      [19.6, 22.6], 
      [27.0, 67.9], 
      [12.5, 73.4],  
      [-12.7, 60.5],  
      [-23.1, -0.8],  
      [-0.5, -4.2],  
      [-0.3, -1.6]]
            
encoder = [[5.7, 26.1],	
           [19.1, 24.1],
           [25.4, 69.1],
           [13.1, 75.6],
           [-14.9, 63.3],
           [8.7, 8.2],
           [12.2, 25.9],
           [6.4, 26.7]]

IMU_left = [[7.4, 26.0],
            [20.4, 22.6],
            [30.4, 68.0],
            [18.1, 74.4],
            [-10.6,	63.0],
            [-18.0,	-4.7],
            [-0.3, -7.7],
            [1.7, -2.7]]

IMU_right = [[5.4,	26.0],
             [19.1,	26.0],
             [28.1,	70.0],
             [16.4,	77.4],
             [-13.3, 68.3],
             [-28.0, 0.6],
             [-10.9, -6.4],
             [-6.9,	-2.1]]

IMU_w_encoder = [[7.4, 26.0],
                 [20.8,	23.0],
                 [31.1,	67.3],
                 [19.1,	73.7],
                 [-9.6,	63.7],
                 [-19.3, -3.4],
                 [-1.9,	-7.7],
                 [0.1,	-2.4]]

def compute_offset(ref, array):
    offset_list = []
    for i, p in enumerate(ref):
        offset = sqrt((p[0] - array[i][0])**2 + (p[1] - array[i][1])**2)
        offset_list.append(offset)
    return offset_list

for est in [encoder, IMU_left, IMU_right, IMU_w_encoder]:
    offset_table = compute_offset(cp, est)
    print("\n")
    print(offset_table)

