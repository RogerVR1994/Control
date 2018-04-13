%Description
% This script loads all initialization parameters required to run
% the Simulink model, modelSim.mdl. The model is run, a dynamic
% simulation is shown and the results are plotted.
clear all;clc;
global R L

% Struggles Mass Properties
m=.45; % Mr. Struggles mass [kg]
mw=.007; % Mr. Struggles wheel mass [kg]
R=.015; % Wheel Radius [m]
L=.083; % CG Height [m]
bh=0.001; % Ground Damping
bf=0.001; % Bearing Damping
g=9.81; % Gravitational Constant [kg/m^2]
% Controller Gains
Kp=500; % Proportional Gain
Kv=250; % Derivative Gain
Ki=1; % Integral Gain
% Disturbance Force
L_f=.1; % Disturbance Force Location
F=0; % Disturbance Force
th_f=pi/4; % Disturbance Force angle

% Motor Parameters
t_sat=0.2; % Motor Torque Limit [Nm]

% Noise Parameters
angle_var=0.2071^2; % Variance in Acceleration Angle Error
rate_var=1E-5; % Variance in Gyro Rate Error
gyro_drift=(0.00016*pi/180); % Gyro Drift Rate [rad/s^2]
error_var=(0.18*pi/180)^2; % Gyro Angular Error[rad/s]
noise_var=(.1*pi/180)^2; % Variance in Gyro Rate Error

% Kalman Filter Covariance Parameters
Q_theta=error_var; % Angle Process Noise
Q_bias=.003; % Rate Process Noise
R_theta=angle_var; % Angle Measurement Noise
kalman_switch=0;

% Initialization
init=[10*pi/180;0;0]; % Initial Conditions
theta_init=init(1); % Kalman Filter Initialization
x_des=[0;0;0]; % Desired steady-state

% Miscellaneous
enc_res=1/(512*(2*pi)); % Encoder Resolution [counts/rev]
clock=0.005; % Clock Period [s]
t_final=5; % Simulation Stop Time


% Load model, set block properties
load_system('modelSim');
% Set Sample Rate for Discrete Blocks
set_param('modelSim/Process Noise/Random Number', 'SampleTime',
num2str('clock'));
set_param('modelSim/Noise Injection/Noise1', 'SampleTime', num2str('clock')); 
Y. DING, J. GAFFORD AND M. KUNIO
set_param('modelSim/Noise Injection/Noise2', 'SampleTime', num2str('clock'));
set_param('modelSim/Noise Injection/Error1', 'SampleTime', num2str('clock'));
set_param('modelSim/Noise Injection/Discrete-Time Integrator', 'SampleTime',
num2str('clock'));
set_param('modelSim/Noise Injection/Quantizer', 'SampleTime',
num2str('clock'));
% Simulation Output Parameters
simOut = sim('modelSim', 'StopTime',
num2str(t_final),'SimulationMode','rapid','AbsTol','1e-5',...
 'SaveState','on','StateSaveName','xoutNew',...
 'SaveOutput','on','OutputSaveName','youtNew');

% Extract Results
theta=simOut.get('theta');theta=squeeze(theta);
gamma=simOut.get('gamma');gamma=squeeze(gamma);
z=simOut.get('z');z=squeeze(z);
tau=simOut.get('tau');tau=squeeze(tau);
z_obs=simOut.get('z_obs');z_obs=squeeze(z_obs);
z_filt=simOut.get('z_filt');z_filt=squeeze(z_filt);
P_out=simOut.get('P_out');P_out=squeeze(P_out);

t=simOut.get('t_out'); % Continuous Time Vector
t_d=0:clock:t_final; % Discrete Time Vector

% Plot Data
PlotSim(t,z(1,:)',z(2,:)',tau); % Dynamic Simulation

% Plot Actual, Observed, and Kalman Response
figure(1);
plot(t,z_obs(:,1).*180/pi,'b');hold on;
plot(t,z_filt(:,1).*180/pi,'r','LineWidth',2);hold on;
plot(t,z(1,:).*180/pi,'k','LineWidth',2);grid on;hold off;
legend('Observed Angle','Kalman Estimate','Actual Angle');
xlabel('Time [s]');ylabel('Angle [deg]');

% Plot Observed and Kalman Error
figure(2)
plot(t,(z_obs(:,1)-z(1,:)').*180/pi);hold on;grid on;
plot(t,(z_filt(:,1)-z(1,:)').*180/pi,'r');hold off;
legend('Error in Observed Angle','Error in Kalman Estimate');
xlabel('Time [s]');ylabel('Error [deg]');

% Display Root-Mean-Squared Deviation
rms=norm(z(1,:)'-z_filt(:,1))/sqrt(length(z(1,:)));
disp(strcat('RMSD: ',num2str(rms),' rad'));

% Plot covariance norm
figure(3)
for j=1:length(t)
 plot(t(j),norm(P_out(:,:,j)),'o');hold on;
 grid on;
end
hold off;
xlabel('Time [s]');
ylabel('Error Covariance Norm');