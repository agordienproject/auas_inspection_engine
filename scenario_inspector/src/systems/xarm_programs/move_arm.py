# Sample xArm Program
# This is a simple example program for xArm robot
# 
# Instructions:
# - Move to home position
# - Move to inspection position
# - Return to home

# Program starts here
move_to_home()
wait(1000)  # Wait 1 second
move_to_position(300, 0, 400, 0, 0, 0)
wait(2000)  # Wait 2 seconds
move_to_home()
