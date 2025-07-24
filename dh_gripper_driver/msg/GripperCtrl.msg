# DH AG95 Gripper Control Message
# Based on DH official gripper control interface

bool initialize        # Initialize gripper
float32 position       # Target position (0.0-100.0mm for AG95)
float32 force          # Gripping force (0.0-100.0N)
float32 speed          # Movement speed (0.0-100.0%) 