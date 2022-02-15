# MIP control
### MIP, short for mobile inverted pendulum, is a mobile device with a robot platform and two wheels. Due to its unstable dynamic property, a good controller is needed.
![MIP figure](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/MIP_figure.png)
![MIP Simplfied figure](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/Simplified_MIP_figure.png)

### To control MIP to keep the robots in upright position, two approaches are used.
### 1.
###### In MATLAB file, MIPparameters.m defines linearized system's parameters, and MIP_control.slx is a SimuLink file that simulates lineared system. 
###### Controller in MATLAB is combination of a feedback controller and an observer. Both feedback controller and observer are designed by placing poles, and all poles are decided by trial and error. Eventhough MIP can be controlled to upright position by this method, several properties are far from good performances. 
###### Ex: Maximum motor voltage (±7.4V) allows only 0.65rad(~=37.3degree) of initial angle of robot platform.
![MAT Voltage performance](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/MATLAB_voltage.png)
###### Performance of theta
![MAT theta performance](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/MATLAB_theta.png)
### 2.
###### In Python file, to optimize performance, I applied LQG controller to find K and F. In the following figure, Maximum voltage allows larger initial angle to 68 degrees, and the LQG controller can still pull the robot platform back to upright position.
###### Performance of voltage when initial amgle is 68 degrees
![PYT Voltage performance](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/Python_voltage.png)
###### Performance of theta
![PYT theta performance](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/Python_theta.png)
##### Properties of K and F
![K and F](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/K_F_value.png)