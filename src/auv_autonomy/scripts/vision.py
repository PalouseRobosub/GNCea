"""
Computer vision utilities for pole detection
"""
import cv2
import numpy as np

class PoleDetector:
    def __init__(self, hsv_params=None):
        """Initialize with HSV parameters"""
        if hsv_params is None:
            # Default parameters
            self.red_lower1 = np.array([0, 100, 100])
            self.red_upper1 = np.array([10, 255, 255])
            self.red_lower2 = np.array([160, 100, 100])
            self.red_upper2 = np.array([180, 255, 255])
            self.white_lower = np.array([0, 0, 200])
            self.white_upper = np.array([180, 30, 255])
        else:
            self._load_params(hsv_params)
    
    def detect_poles(self, frame):
        """Detect red and white poles in frame"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Red detection
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        # White detection
        white_mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
        
        return red_mask, white_mask
    
    def find_pole_centers(self, mask, min_area=500):
        """Find centroids of poles in binary mask"""
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                        cv2.CHAIN_APPROX_SIMPLE)
        
        poles = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    poles.append((cx, cy, area))
        
        return poles
    
    def calculate_gate_center(self, red_poles, white_poles, frame_shape):
        """Calculate center point between red and white poles"""
        height, width = frame_shape[:2]
        
        if not red_poles or not white_poles:
            return None
        
        red_pole = max(red_poles, key=lambda p: p[2])
        white_pole = max(white_poles, key=lambda p: p[2])
        
        gate_x = (red_pole[0] + white_pole[0]) // 2
        gate_y = (red_pole[1] + white_pole[1]) // 2
        
        pole_distance = np.sqrt((red_pole[0] - white_pole[0])**2 + 
                               (red_pole[1] - white_pole[1])**2)
        
        return gate_x, gate_y, pole_distance
    
    def _load_params(self, params):
        """Load HSV parameters from dict"""
        self.red_lower1 = np.array(params['red_lower1'])
        self.red_upper1 = np.arra