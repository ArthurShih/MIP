# MIP control
###### MIP, short for mobile inverted pendulum, is a mobile device with a robot platform and two wheels.
![MIP figure](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/MIP_figure.png)
![MIP Simplfied figure](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/Simplified_MIP_figure.png)

### To control MIP to equilibrium, two approaches are used.
### 1.
###### In MATLAB file, MIPparameters.m defines linearized system's parameters, and MIP_control.slx is SimuLink file that simulate lineared system. 
###### Controller in MATLAB is a combination of feedback controller and an observer. Both feedback controller and observer are designed by placing poles, and all poles are decided by trial and error. Eventhough MIP can be controlled to equilibrium by this method, several properties are far from good performances. 
###### Ex: Maximum motor voltage (±7.4V) allows only 0.65rad(~=37.3degree) of initial theta(angle of robot platform).
![Voltage performance](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/V_response.png)
###### Performance of theta
![theta performance](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/theta.png)
### 2.
###### In Python file, to optimize performance, I applied LQG controller to fin K and F. In the following figure, Maximum voltage allows larger initial theta to 68 degrees.
###### Performance of voltage when initial theta = 68 degrees
![Voltage performance](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/Voltage.png)
###### Performance of theta
![theta performance](https://github.com/ArthurShih/MIP_control/blob/master/MIP-figure/angle.png)
