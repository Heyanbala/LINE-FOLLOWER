import graphviz
from graphviz import Digraph

# Create a new directed graph
dot = Digraph('Line_Follower_Control_System', comment='PID Line Follower Control Flow')

# Set graph attributes for a clean look
dot.attr(rankdir='LR') # Left-to-Right orientation
dot.attr('node', shape='box', style='filled', fillcolor='#E0FFFF')

## --- Define the System Components (Nodes) ---

# 1. Inputs/Sensors
dot.node('IR_Sensors', '3x IR Line Sensors (L, M, R)', shape='cylinder', fillcolor='#CCFFCC')
dot.node('IMU', 'MPU6050 (Gyro Z, Accel)', shape='cylinder', fillcolor='#CCFFCC')

# 2. Control Logic/Error Calculation
dot.node('Line_Detect', 'Read Analog Values\n> lineThreshold (500)', shape='ellipse', fillcolor='#FFFFCC')
dot.node('Error_Calc', 'Simple Error Calculation\n(-1, 0, +1)', fillcolor='#FFFFCC')

# 3. PID Core
dot.node('PID_P', 'P-Term (Kp * Error)', fillcolor='#F0FFF0')
dot.node('PID_I', 'I-Term (Ki * Integral)', fillcolor='#F0FFF0')
dot.node('PID_D', 'D-Term (Kd * Derivative)', fillcolor='#F0FFF0')
dot.node('Gyro_D', 'Gyro-Term (Kgyro * RateOfTurn)', fillcolor='#F0FFF0')
dot.node('PID_Sum', 'PID Output Summation', shape='Mdiamond', fillcolor='#ADFFAD')

# 4. Motor Mapping and Constraints
dot.node('Map_Speed', 'Map PID Output to Speed Offset', fillcolor='#DDDDFF')
dot.node('Constrain', 'Constrain(0, 150)', fillcolor='#DDDDFF')

# 5. Actuators/Output
dot.node('Motors', 'H-Bridge Motor Driver\n(PWM Speed, Direction)', shape='trapezium', fillcolor='#FFCCCC')

# 6. Specialized Function (Loss Detection)
dot.node('Search', 'Line Lost: Search & Recovery\n(Uses lastKnownDirection)', shape='house', fillcolor='#F08080')


## --- Define the Flow (Edges) ---

# Sensor to Logic Flow
dot.edge('IR_Sensors', 'Line_Detect', label='Raw Values')
dot.edge('Line_Detect', 'Error_Calc', label='Boolean On/Off')
dot.edge('IMU', 'Gyro_D', label='Rate of Turn (Gyro Z)')

# Error to PID Core
dot.edge('Error_Calc', 'PID_P', label='Error')
dot.edge('Error_Calc', 'PID_I', label='Error')
dot.edge('Error_Calc', 'PID_D', label='Error')

# Line Loss Path
dot.edge('Line_Detect', 'Search', label='All Sensors Off', style='dashed', color='red')
dot.edge('Search', 'Motors', label='Fixed Turn Speed', style='dashed', color='red')

# PID Core Calculation and Summation
dot.edge('PID_P', 'PID_Sum')
dot.edge('PID_I', 'PID_Sum')
dot.edge('PID_D', 'PID_Sum')
dot.edge('Gyro_D', 'PID_Sum', label='Stabilization')

# PID Output to Motors
dot.edge('PID_Sum', 'Map_Speed', label='Correction Signal')
dot.edge('Map_Speed', 'Constrain', label='Speed Offset')
dot.edge('Constrain', 'Motors', label='Left/Right Speed (0-150)')

# Feedback Loop
dot.edge('Motors', 'IR_Sensors', label='Robot Movement/Adjustment', style='bold', color='blue')

# Render the graph to a file
try:
    # Render and save the graph as an image (e.g., PNG)
    # The 'view=True' will attempt to open the resulting image file
    dot.render('line_follower_control_flow', view=True, format='png')
    print("Successfully generated line_follower_control_flow.png")
except graphviz.backend.ExecutableNotFound:
    print("\n--- ERROR ---")
    print("Graphviz rendering executable not found.")
    print("Please ensure the native Graphviz software is installed on your system.")
    print("The DOT source file (line_follower_control_flow.gv) has been created.")

#