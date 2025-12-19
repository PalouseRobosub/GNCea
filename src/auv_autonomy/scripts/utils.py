"""
Utility functions for gate navigation
"""
import cv2
import yaml

def load_hsv_config(config_path):
    """Load HSV parameters from YAML file"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config['hsv_params']

def draw_debug_overlay(frame, red_poles, white_poles, gate_info):
    """Draw visualization overlay on frame"""
    debug_frame = frame.copy()
    
    # Draw red poles
    for pole in red_poles:
        cv2.circle(debug_frame, (pole[0], pole[1]), 10, (0, 0, 255), -1)
    
    # Draw white poles
    for pole in white_poles:
        cv2.circle(debug_frame, (pole[0], pole[1]), 10, (255, 255, 255), -1)
    
    # Draw gate center
    if gate_info is not None:
        gate_x, gate_y, pole_distance = gate_info
        cv2.circle(debug_frame, (gate_x, gate_y), 15, (0, 255, 0), 2)
        cv2.line(debug_frame, 
                (frame.shape[1]//2, 0), 
                (frame.shape[1]//2, frame.shape[0]), 
                (255, 0, 0), 2)
        
        # Display distance
        cv2.putText(debug_frame, f"Dist: {pole_distance:.1f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0, 255, 0), 2)
    
    return debug_frame

class MovingAverageFilter:
    """Simple moving average filter for smoothing"""
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.values = []
    
    def update(self, value):
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)
    
    def reset(self):
        self.values = []