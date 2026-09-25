clear;clc;close all;

% The first logical thing to start with is the accelerometer attitude
% estimation and comparing it with the actual angles.
data = readtable("flight_data.csv");
calSens = readtable("calibratedSensorsData.xlsx");
acc_x_cal = calSens.Acc_X_cal;
acc_y_cal = calSens.Acc_Y_cal;
acc_z_cal = calSens.Acc_Z_cal;
time_uniform = calSens.Time_uniform;
N = length(time_uniform)
g = 9.80665; % or use norm of your accel vector instead of a fixed g

roll_acc  = atan2(acc_y_cal, -acc_z_cal);              % phi
pitch_acc = atan2(acc_x_cal, sqrt(acc_y_cal.^2 + acc_z_cal.^2)); % theta

figure;
plot(time_uniform,roll_acc);
hold on 
plot(data.time,data.roll_deg*pi/180);
figure;
plot(time_uniform,pitch_acc);
hold on 
plot(data.time,data.pitch_deg*pi/180);

% Angles estimation using gyro only 
gyro_p_cal = calSens.Gyro_P_cal;
gyro_q_cal = calSens.Gyro_Q_cal;
gyro_r_cal = calSens.Gyro_R_cal;
pitch0_gyro = data.pitch_deg(1)*pi/180;
roll0_gyro = data.roll_deg(1)*pi/180;
eulerRates0 = [gyro_p_cal(1)*1+gyro_q_cal(1)*sin(roll0_gyro)*tan(pitch0_gyro)+gyro_r_cal(1)*cos(roll0_gyro)*tan(pitch0_gyro),...
    gyro_q_cal(1)*cos(roll0_gyro)-gyro_r_cal(1)*sin(roll0_gyro), ...
    gyro_q_cal(1)*sin(roll0_gyro)/cos(pitch0_gyro)+gyro_r_cal(1)*cos(roll0_gyro)/cos(pitch0_gyro)]
eulerRates = zeros(N,3);
eulerRates(1,:) = eulerRates0;
roll_gyro = zeros(N,1);roll_gyro(1) = roll0_gyro;
pitch_gyro = zeros(N,1); pitch_gyro(1) = pitch0_gyro;
dT = 0.01;
for i=2:N
    roll_gyro(i)= eulerRates(i-1,1)*dT + roll_gyro(i-1);
    pitch_gyro(i)= eulerRates(i-1,2)*dT + pitch_gyro(i-1);
    eulerRates(i,:) = [gyro_p_cal(i)*1+gyro_q_cal(i)*sin(roll_gyro(i))*tan(pitch_gyro(i))+gyro_r_cal(i)*cos(roll_gyro(i))*tan(pitch_gyro(i)),...
    gyro_q_cal(i)*cos(roll_gyro(i))-gyro_r_cal(i)*sin(roll_gyro(i)), ...
    gyro_q_cal(i)*sin(roll_gyro(i))/cos(pitch_gyro(i))+gyro_r_cal(i)*cos(roll_gyro(i))/cos(pitch_gyro(i))];
end
figure;
plot(time_uniform,roll_gyro);
hold on 
plot(data.time,data.roll_deg*pi/180);

figure;
plot(time_uniform,pitch_gyro);
hold on 
plot(data.time,data.pitch_deg*pi/180)

%% Complementary Filter Gyro + Acceleromter
gyro_p_cal = calSens.Gyro_P_cal;
gyro_q_cal = calSens.Gyro_Q_cal;
gyro_r_cal = calSens.Gyro_R_cal;
pitch0_comp = data.pitch_deg(1)*pi/180;
roll0_comp = data.roll_deg(1)*pi/180;
eulerRates0_comp = [gyro_p_cal(1)*1+gyro_q_cal(1)*sin(roll0_comp)*tan(pitch0_comp)+gyro_r_cal(1)*cos(roll0_comp)*tan(pitch0_comp),...
    gyro_q_cal(1)*cos(roll0_comp)-gyro_r_cal(1)*sin(roll0_comp), ...
    gyro_q_cal(1)*sin(roll0_comp)/cos(pitch0_comp)+gyro_r_cal(1)*cos(roll0_comp)/cos(pitch0_comp)]
eulerRates_comp = zeros(N,3);
eulerRates_comp(1,:) = eulerRates0_comp;
roll_comp = zeros(N,1);roll_comp(1) = roll0_comp;
pitch_comp = zeros(N,1); pitch_comp(1) = pitch0_comp;
dT = 0.01;
compConst = 0.99
for i=2:N
    roll_comp(i)= (eulerRates_comp(i-1,1)*dT + roll_comp(i-1))*compConst + roll_acc(i)*(1-compConst);
    pitch_comp(i)= (eulerRates_comp(i-1,2)*dT + pitch_comp(i-1))*compConst + pitch_acc(i)*(1-compConst);
    eulerRates_comp(i,:) = [gyro_p_cal(i)*1+gyro_q_cal(i)*sin(roll_comp(i))*tan(pitch_comp(i))+gyro_r_cal(i)*cos(roll_comp(i))*tan(pitch_comp(i)),...
    gyro_q_cal(i)*cos(roll_comp(i))-gyro_r_cal(i)*sin(roll_comp(i)), ...
    gyro_q_cal(i)*sin(roll_comp(i))/cos(pitch_comp(i))+gyro_r_cal(i)*cos(roll_comp(i))/cos(pitch_comp(i))];
end
figure;
plot(time_uniform,roll_comp,"Color","blue");
hold on 
plot(data.time,data.roll_deg*pi/180,"Color","red");

figure;
plot(time_uniform,pitch_comp,"Color","blue");
hold on 
plot(data.time,data.pitch_deg*pi/180,"Color","red")
%% Kalman Filter Gyro + Accelerometer (roll)
dT = 0.01;

% Tunable noise parameters
Q_angle = (0.005*pi/180)^2*dT;    % process noise: trust in gyro integration
Q_bias  = Q_angle*0;    % process noise: how fast bias drifts
R_meas  = (0.007^2/dT)/g^2;      % measurement noise: accel angle noise variance (try var(roll_acc) on a static segment)

% State: x = [angle; bias]
x_roll = [roll0_gyro; 0];      % initial bias guess = 0
P_roll = eye(2);                % initial covariance guess

roll_kf = zeros(N,1);
roll_kf(1) = x_roll(1);

A = [1 -dT; 0 1];
B = [dT; 0];
H = [1 0];
Q = [Q_angle 0; 0 Q_bias];

for i = 2:N
    % ---- Predict ----
    u = gyro_p_cal(i);                 % gyro rate as control input
    x_roll = A*x_roll + B*u;
    P_roll = A*P_roll*A' + Q;

    % ---- Update ----
    z = roll_acc(i);                   % accelerometer-derived angle as measurement
    y = z - H*x_roll;                  % innovation
    S = H*P_roll*H' + R_meas;          % innovation covariance
    K = P_roll*H'/S;                   % Kalman gain

    x_roll = x_roll + K*y;
    P_roll = (eye(2) - K*H)*P_roll;

    roll_kf(i) = x_roll(1);
end

figure;
plot(time_uniform, roll_kf, "Color","blue"); hold on;
plot(data.time, data.roll_deg*pi/180, "Color","red");
legend('Kalman roll estimate','True roll');
