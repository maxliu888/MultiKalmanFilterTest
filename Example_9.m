%  ==========Example 9=============

% Five  Kalman filters:
% X_(n+1,n) = F * X_(n,n) + G * u_n + w_n   -------------the state extrapolate equation
% P_(n,n)   = F * P_(n,n) * F' + Q   -------the Covariance  Extrapolation
% Kn        = P_(n,n) * H'* (H * P_(n,n) * H' + Rn)^(-1)     ------kalman gain
% X_(n,n)   = X_(n,n-1) + Kn * (Zn - H * X_(n,n-1))  ------the state update equation 
% P_(n,n) =(I - Kn * H) * P_(n,n) * (I - Kn * H)' +  Kn * Rn * Kn' ------Covariance  update

%define the system
clear;
N = 35;   % number of time steps
t = (1:N);
dt = 1;   % time interval 
sigma_acc = 0.2; %The random acceleration standard deviation
sigma_x =3; %measurement error standard deviation (x-axis)
sigma_y = 3; %measurement error standard deviation (y-axis)

F =[1 dt 0.5*dt^2  0  0    0;
    0 1  dt        0  0    0;
    0 0  1         0  0    0;
    0 0  0         1  dt  0.5*dt^2;     
    0 0  0         0  1    dt;
    0 0  0         0  0     1; ];  %the state transition matrix

% G , u_n  is null;  % we don't have a control variable u since we don't have control input,and control matrix

H = [1 0 0 0 0 0;
     0 0 0 1 0 0]; %observation matrix 
 
Q = [(dt^4)/4  (dt^3)/2  (dt^2)/2   0         0         0;
     (dt^3)/2  (dt^2)     dt        0         0         0;
     (dt^2)/2   dt        1         0         0         0;
      0         0         0        (dt^4)/4  (dt^3)/2  (dt^2)/2; 
      0         0         0        (dt^3)/2  (dt^2)     dt;
      0         0         0        (dt^2)/2   dt        1;       ] * sigma_acc;   % process noise matrix 

  R = [(sigma_x)^2    0;  
        0            (sigma_y)^2 ]; % measurement uncertainty 
  
 I = eye(6);  %identity matrix
 

%define the initial position,velocity,acceleration
position_Ini_x = 0;
velocity_Ini_x = 0;
acceleration_Ini_x = 0;
position_Ini_y = 0;
velocity_Ini_y = 0;
acceleration_Ini_y = 0;
 
%set of 35 noisy measurements(Row 1 is the data for the x-axis,Row 2 is the data for the y-axis): 
measurement_value = zeros(2,N);
measurement_value(1,:) = [-393.66	-375.93	-351.04	-328.96	-299.35	-273.36	-245.89	-222.58	-198.03	-174.17	-146.32	-123.72	-103.47	-78.23	-52.63	-23.34	25.96	49.72	76.94	95.38	119.83	144.01	161.84	180.56	201.42	222.62	239.4	252.51	266.26	271.75	277.4	294.12	301.23	291.8	299.89] ;
measurement_value(2,:) = [300.4	301.78	295.1	305.19	301.06	302.05	300	303.57	296.33	297.65	297.41	299.61	299.6	302.39	295.04	300.09	294.72	298.61	294.64	284.88	272.82	264.93	251.46	241.27	222.98	203.73	184.1	166.12	138.71	119.71	100.41	79.76	50.62	32.99	2.14];



%ideal value 
ideal_x = [-400,-300,-200,-100,0];
ideal_y = [300,300,300,300,300];
r = 300;
xc = 0;
yc = 0;
ideal_value = zeros(2,5);

ideal_value(1,:)= ideal_x;
ideal_value(2,:)= ideal_y;
theta = linspace(0,(pi/2));
round_x = r*cos(theta) + xc;
round_y = r*sin(theta) + yc;


figure('Name','ideal Vehicle position','NumberTitle','off');
plot(ideal_value(1,:),ideal_value(2,:),'g',round_x,round_y,'g');
grid on;
axis equal

%=====================Start ineration=============================


%----------[intializatio]---------
% we don't know the vehicle loation; we will set position,velocity,acceleration to 0
 X_estimate = zeros(6,N);
 X_estimate(:,1) = [position_Ini_x; velocity_Ini_x; acceleration_Ini_x; position_Ini_y; velocity_Ini_y; acceleration_Ini_y;];
 
%Since our initial state vector is a guess, we will set a very high estimate uncertaionty.
%The high etimate uncertainty results in a hige Kalman gain, giving a high weigh to measurement 
P_estimate = 500*eye(6);

%initial parameter
Z_measurement =  zeros(2,N);
Kalman_gain =  zeros(6,2);
X_predict =  zeros(6,1);
P_predict =  zeros(6,6);
position_x_y = zeros(2,N);

%--------[ineration 1 to N]---------
for i = 1 : N
    %predict 
    X_predict = F *  X_estimate(:,i);
    P_predict = F * P_estimate * F' + Q;
    
    %measure 
    Z_measurement(:,i) = measurement_value(:,i);
    
    %update
    Kalman_gain =  P_predict * H'* (H *  P_predict * H' + R)^(-1);
    X_estimate(:,i)  =  X_predict + Kalman_gain * (Z_measurement(:,i) - H * X_predict);  
    P_estimate  = (I - Kalman_gain * H) * P_predict * (I - Kalman_gain * H)' +  Kalman_gain * R * Kalman_gain';
     
end


position_x_y(1,:) = X_estimate(1,:);
position_x_y(2,:) = X_estimate(4,:);


figure('Name','Vehicle position','NumberTitle','off');
plot(position_x_y(1,:), position_x_y(2,:), 'r', Z_measurement(1,:), Z_measurement(2,:), 'b');
hold on ;
plot(ideal_value(1,:),ideal_value(2,:),'g',round_x,round_y,'g');
grid on;

legend('estimate','measure','ideal');
xlabel('x(m)');
ylabel('y(m)');
set(gca,'xtick',-400:100:300);  
set(gca,'ytick',0:50:300);  
title('Vehicle position');


