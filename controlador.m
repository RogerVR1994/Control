%2.151 final project
%COM and controllability, observability, LQR design
%Date 2012/12/13
%Revised 2012/12/16
%Parameters
m=600.3*10^(-3); %body part mass [kg]
m2=0*10^(-3); %shaft part mass [kg], CAN CHANGABLE!!!!
m1=m-m2; %head part mass [kg], change by mass of shaft
mw=9.2*10^(-3); %wheel(*2) mass [kg]
L1=1*10^(-3); %height of head [m]
L2=92*10^(-3); %height of shaft [m]
I=m1*(L1/2+L2)^2+m2*L2*L2/12; %inertia of body part [kg*m^2]
Iw=389.6*10^(-9)*2; %inertia of wheel [kg*m^2]
Br=0.01; %rolling damping ratio [N*m/(rad/s)]
Bm=0.01; %bearing damping ratio [N*m/(rad/s)]
L=L2/2+(L1+L2)*m1/(2*m) %position of COM [m]
R=90*10^(-3); %radius of wheel [m]
g=9.8; %gravity [m/s^2]

%Weighting matrices
E=[Iw+(mw+m)*R*R m*R*L; m*R*L m*L*L+I]; %for d^2/dt^2 (phi and theta)
F=[Br+Bm -Bm; -Bm Bm]; %for d/dt (phi and theta)
G=[0; -m*g*L]; %for theta
H=[1; -1]; %for input torque

%state-space representation of the system
%state variable: phi, theta, d(phi)/dt, d(theta)/dt
A=[0 0 1 0; 0 0 0 1; [0; 0] -inv(E)*G -inv(E)*F] %system matrix
B=[0; 0; inv(E)*H] %input matrix
C=[R 0 0 0; 0 1 0 0] %output matrix
D=0
sys1=ss(A,B,C,D)
G1=tf(sys1) %transfer function of sys1
G1zp=zpk(sys1) %Gain/pole/zero representation of sys1

%controllability and observability check for sys1
Cont=[B A*B A*A*B A*A*A*B];
rank(Cont)
Obs=[C; C*A; C*A*A; C*A*A*A];
rank(Obs)

%LQR controller design
xweight=eye(4); %weighting matrix Q
uweight=1; %weighting matrix R
K=-lqr(A,B,xweight,uweight) %gain matrix
sys1_lqr=ss(A+B*K,B,C,D); %close-loop system
initial(sys1_lqr, [0; 0.17; 0; 0]) %free response