#!/usr/bin/env python3
"""
Interactive HSV tuning tool for pole detection
"""
import cv2
import numpy as np
import yaml

def nothing(x):
    pass

def main():
    cap = cv2.VideoCapture(0)
    
    # Create windows
    cv2.namedWindow('Original')
    cv2.namedWindow('Red Mask')
    cv2.namedWindow('White Mask')
    cv2.namedWindow('Controls')
    
    # Create trackbars for red
    cv2.createTrackbar('R_H_Low1', 'Controls', 0, 180, nothing)
    cv2.createTrackbar('R_S_Low', 'Controls', 100, 255, nothing)
    cv2.createTrackbar('R_V_Low', 'Controls', 100, 255, nothing)
    cv2.createTrackbar('R_H_High1', 'Controls', 10, 180, nothing)
    cv2.createTrackbar('R_S_High', 'Controls', 255, 255, nothing)
    cv2.createTrackbar('R_V_High', 'Controls', 255, 255, nothing)
    
    cv2.createTrackbar('R_H_Low2', 'Controls', 160, 180, nothing)
    cv2.createTrackbar('R_H_High2', 'Controls', 180, 180, nothing)
    
    # Create trackbars for white
    cv2.createTrackbar('W_S_High', 'Controls', 30, 255, nothing)
    cv2.createTrackbar('W_V_Low', 'Controls', 200, 255, nothing)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get trackbar values
        r_h_low1 = cv2.getTrackbarPos('R_H_Low1', 'Controls')
        r_s_low = cv2.getTrackbarPos('R_S_Low', 'Controls')
        r_v_low = cv2.getTrackbarPos('R_V_Low', 'Controls')
        r_h_high1 = cv2.getTrackbarPos('R_H_High1', 'Controls')
        r_s_high = cv2.getTrackbarPos('R_S_High', 'Controls')
        r_v_high = cv2.getTrackbarPos('R_V_High', 'Controls')
        
        r_h_low2 = cv2.getTrackbarPos('R_H_Low2', 'Controls')
        r_h_high2 = cv2.getTrackbarPos('R_H_High2', 'Controls')
        
        w_s_high = cv2.getTrackbarPos('W_S_High', 'Controls')
        w_v_low = cv2.getTrackbarPos('W_V_Low', 'Controls')
        
        # Create masks
        red_mask1 = cv2.inRange(hsv, 
                               np.array([r_h_low1, r_s_low, r_v_low]),
                               np.array([r_h_high1, r_s_high, r_v_high]))
        red_mask2 = cv2.inRange(hsv,
                               np.array([r_h_low2, r_s_low, r_v_low]),
                               np.array([r_h_high2, r_s_high, r_v_high]))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        white_mask = cv2.inRange(hsv,
                                np.array([0, 0, w_v_low]),
                                np.array([180, w_s_high, 255]))
        
        cv2.imshow('Original', frame)
        cv2.imshow('Red Mask', red_mask)
        cv2.imshow('White Mask', white_mask)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Save parameters
            params = {
                'hsv_params': {
                    'red_lower1': [r_h_low1, r_s_low, r_v_low],
                    'red_upper1': [r_h_high1, r_s_high, r_v_high],
                    'red_lower2': [r_h_low2, r_s_low, r_v_low],
                    'red_upper2': [r_h_high2, r_s_high, r_v_high],
                    'white_lower': [0, 0, w_v_low],
                    'white_upper': [180, w_s_high, 255]
                }
            }
            with open('../config/hsv_params.yaml', 'w') as f:
                yaml.dump(params, f)
            print("Parameters saved!")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()