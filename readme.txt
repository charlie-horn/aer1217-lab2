# ----- How to choose trajectory mode --------

In desired_position.py:
 - Line 44-45 calls the linear trajectory
 - Line 46-47 calls the circular trajectory
 - To alter publishing frequency, change line 41
 - To change the interim position check threshold, change line 39
 - To not include interim position checks of the circular trajectory and only check the destination points:
     - Comment lines 108-111 
     - Uncomment lines 114-126
     
# ----- How to change controller values --------

In position_controller.py:
 - Gain values are located in line 161-168
 - Lines 81 and 82 control the desired x and y velocities
 
