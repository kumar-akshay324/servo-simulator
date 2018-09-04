###Servo Simulator ###

##A prototype for position-controlled servo-mechanism implemented on different types of DC motor models using a PID controller##    

_servo_simulator.pdf_ - Documentation for the prototype
_servo_simulator.py_ - Contains the PID controller and simulates the servo joint motion. 
_motor.py_ - Motor models for separately excited DC motor and armature controlled DC motor

**Please read  the documentation for detailed analysis and discussion**

<iframe width="560" height="315" src="https://www.youtube.com/embed/6f2kRp-R3vs" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>

##Usage## 

Command to Run in Terminal:

```python3 servo_simulator.py [target angular position] -t [type of motor]  -s [initial angular position]```

_target angular position_			: float value between 0 - 90 degrees -   Compulsory Input

_type of motor_	:	  String value - Optional Input
	                    
Separately Excited Motor Model		:	`'sep'`
Armature Controlled Motor Model		:	`'armc'`	(Default)

_initial angular position_			:	float value between 0- 90 degrees - Optional Input

##Author##

* Akshay Kumar - akumar5@wpi.edu