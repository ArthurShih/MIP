# TODO: MIP(Mobile Inverted Pendulum) LQR control
import numpy as np 
import control
from scipy.linalg import eig
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Model Parameters
r = 34*1e-3 # Radius of wheels: 34(mm)
l = 36*1e-3 # Body center of mass to wheel axis: 36(mm)
mw = 27*1e-3 # Mass of a wheel: 27(g)
mb = 263*1e-3 # Mass of the body: 263(g)
Gr = 35.57 # Gearbox ratio
Im = 3.6*1e-8 # Motor armature inertia: 3.6*10**-8(kg*m*m)
Ib = 4*1e-4 # Body inertia 4*10**-4(kg*m*m)
Iw = mw*(r**2)/2 + (Gr**2)*Im # Interia of singel wheel plus gearbox
Vmax = 7.4 # Nominal battery voltage: 7.4(V)
sbar = 0.003 # Motor stall torque: 0.003(Nm) at Vmax
wf = 1760 # Motor free run speed: 1760(rad/s) at Vmax
k = sbar/wf # Motor constant
g = 9.8 # Acceleration due to gravity: 9.8(m/s/s)

a = 2*Iw + (mb + 2*mw)*r*r 
b = mb*r*l 
c = Ib + mb*l*l 
d = mb*g*l 
e = 2*Gr*sbar/Vmax
j = 2*(Gr**2)*k
Para = a*c-b**2 

# Reduced System xdot = A*x + Bu*u + Bw*w  ,    y = Cy*x + v => y= Cy*x + Dyw*w
A = np.array([[-(a+b)*j/Para , (a+b)*j/Para , a*d/Para],
			  [ (b+c)*j/Para ,-(b+c)*j/Para ,-b*d/Para],
			  [        1     ,       0      ,     0   ]])
Bu = np.array([[-(a+b)*e/Para],
			   [ (b+c)*e/Para],
			   [       0     ]])
Bw = np.array([[-(a+b)*e/Para,0,0],
			   [ (b+c)*e/Para,0,0],
			   [       0     ,0,0]])
Cy = np.array([[1,0,0],
			   [0,1,0]])
Dyw = np.array([[0,1,0],
				[0,0,1]])

# Setting parameter
W1 = 4 # invariance of w1
W2 = 0.01 # invariance of w2
W3 = 0.1 # invariance of w3
W = np.array([[W1,0,0],
			  [0,W2,0],
			  [0,0,W3]])

# Find K
Qk = np.eye(3)
Rk = 1 
[K,X,E] = control.lqr(A, Bu, Qk, Rk)
K = -K
print("K =\n",K)
eigvalue, eigvector = eig(A+Bu.dot(K))
print("\nEigenvalue of A+Bu*K =\n",eigvalue)

# Find F
Qf = Bw.dot(W).dot(np.transpose(Bw))
Rf = Dyw.dot(W).dot(np.transpose(Dyw))
Af = np.transpose(A)
Cf = np.transpose(Cy)
[F,Y,E] = control.lqr(Af,Cf,Qf,Rf)
F = -np.transpose(F)
print("\nF =\n",F)
eigvalue, eigvector = eig(A+F.dot(Cy))
print("\nEigenvalue of A+F*Cy =\n",eigvalue)

# Simulation parameters
t_start = 0 
t_end = 10
t_step = 0.001 
t = np.arange(t_start, t_end, t_step)
theta0 = 65*np.pi/180
x0 = [0,0,theta0]
xhat0 = [0,0,0]
f0 = np.array(x0+xhat0)

# Closed loop system
def sys(f,t):
	x = f[0:3]
	xhat = f[3:6]
	xdot = A.dot(x) + Bu.dot(K).dot(xhat)
	xhatdot = -F.dot(Cy).dot(x) + (A+Bu.dot(K)+F.dot(Cy)).dot(xhat)
	return np.concatenate((xdot,xhatdot))

f = odeint(sys, f0, t)

x = f[:,0:3]
xhat = f[:,3:6]
thetadata = x[:,2]*180/np.pi 
plt.figure(1)
plt.plot(t,thetadata)
plt.grid()
plt.figure(2)
V = np.transpose(K.dot(np.transpose(xhat)))
if (max(V)> -min(V)):
	V_max = max(V)
else:
	V_max = -min(V)
plt.plot(t,V)
plt.title(("Vmax ="+str(V_max)))
plt.grid()
plt.show()
