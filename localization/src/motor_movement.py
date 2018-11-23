import numpy as np

motor_angles_deg = [45, 135, 225, 315]
motor_diameter = 0.11

def to_rad(a):
    return a * 0.01745329

motor_angles = [to_rad(a) for a in motor_angles_deg]
motor_coff = motor_diameter * np.pi * 62.5 / 18.75 / 64


coup = np.array([
    [ -np.sin(motor_angles[0]), -np.cos(motor_angles[0]), 1 ], 
    [ -np.sin(motor_angles[1]), -np.cos(motor_angles[1]), 1 ], 
    [ -np.sin(motor_angles[2]), -np.cos(motor_angles[2]), 1 ], 
    [ -np.sin(motor_angles[3]), -np.cos(motor_angles[3]), 1 ], 
    ])

coup *= 1/motor_coff
inverse_coup = np.linalg.pinv(coup)

def clamp(v, min_, max_):
    return max(min_, min(max_, v))

def to_eucl(motor_s):
    return inverse_coup * motor_s

def to_motor(eucl_s):
    return coup * eucl_s

def slippage(motor_s):
    return motor_s[0::2] - motor_s[1::2]

def test_splippage(motor_s):
    return np.all(slippage(motor_s) < 0.0001)

def correct_slippage(motor_s):
    return motor_s - np.sum(motor_s[0::2] - motor_s[1::2])/4 * np.matrix("1 -1 1 -1").T
