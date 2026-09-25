clc;close;clear all;
rng(1);
Fs = 100;
dt = 1/Fs;
uniform_data = readtable("uniformDataFile.xlsx");
roll_uniform = uniform_data.Roll_uniform*pi/180;
pitch_uniform = uniform_data.Pitch_uniform*pi/180;
yaw_uniform = uniform_data.Yaw_uniform*pi/180;
omega_uniform = [uniform_data.Omega_p_uniform,uniform_data.Omega_q_uniform,uniform_data.Omega_r_uniform]*pi/180;
acc_uniform = [uniform_data.Accel_X_uniform,uniform_data.Accel_Y_uniform,uniform_data.Accel_Z_uniform];
alt_uniform = uniform_data.Alt_uniform;
time_uniform = uniform_data.Time_uniform;
%% Gyroscope Error Modeling
S_gyro = diag([0.002, 0.002, -0.001]);
M_gyro = [ 0,       0.0005, 0.001;
          -0.0005,  0,     -0.0008;
          -0.001,   0.0008, 0     ];
T_gyro = eye(3) + M_gyro + S_gyro;
b_gyro_static = [0.01; -0.015; 0.008];    % Static Gyro Bias (rad/s)

ARW = 0.005 * (pi/180);                   % Angle Random Walk (rad/s/sqrt(s))
N = length(time_uniform);
std_white_gyro = ARW / sqrt(dt);
white_noise_gyro = std_white_gyro * randn(N, 3);

gyro_meas = (T_gyro * omega_uniform')' + b_gyro_static' +  white_noise_gyro;

figure;
plot(time_uniform,omega_uniform(:,1),"Color","blue");
hold on;
plot(time_uniform,gyro_meas(:,1),"Color","red");
hold off;
%% Accelerometer
S_acc = diag([0.005, -0.003, 0.004]);     % Scale factor errors
M_acc = [  0,       0.001, -0.002;
          -0.001,   0,      0.0015;
           0.002,  -0.001,  0      ];    % Misalignment matrix
T_acc = eye(3) + M_acc + S_acc;
b_acc_static = [0.05; -0.03; 0.08];       % Static residual bias (m/s^2)

VRW = 0.007;                              % Velocity Random Walk (m/s/sqrt(s))

std_white_acc = VRW / sqrt(dt);
white_noise_acc = std_white_acc * randn(N, 3);

acc_meas = (T_acc * acc_uniform')' + b_acc_static' + white_noise_acc;

figure;
plot(time_uniform,acc_uniform(:,1),"Color","blue");
hold on;
plot(time_uniform,acc_meas(:,1),"Color","red");
hold off

%% Magnetometer Error Modeling
B_NED = [21.5; 1.2; 42.8];                % Magnetic field vector in uT

mag_uniform = zeros(N, 3);
for k = 1:N
    phi = roll_uniform(k); theta = pitch_uniform(k); psi = yaw_uniform(k);
    R_NED2Body = [
        cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta);
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta)
    ];
    mag_uniform(k,:) = (R_NED2Body * B_NED)';
end

A_soft_iron = [ 1.05, 0.03, 0.01;
                0.03, 0.96, 0.02;
                0.01, 0.02, 0.98 ];
V_hard_iron = [ 12.5; -8.3; 22.0 ];       % uT
std_mag_noise = 0.5;                       % Noise (uT)

mag_meas = (A_soft_iron * mag_uniform')' + V_hard_iron' + std_mag_noise * randn(N, 3);

figure;
plot(time_uniform,mag_uniform(:,1),"Color","blue");
hold on;
plot(time_uniform,mag_meas(:,1),"Color","red");
hold off

%% Barometer Error Modelling
P0 = 101325;      % Sea-level standard pressure (Pa)
T0 = 288.15;      % Sea-level standard temperature (K)
L  = 0.0065;      % Temperature lapse rate (K/m)
g  = 9.80665;     % Standard gravity (m/s^2)
M  = 0.0289644;   % Molar mass of dry air (kg/mol)
R  = 8.31446;     % Universal gas constant (J/(mol*K))

exponent = (g*M)/(R*L);   % ~5.2559

% True pressure from true altitude (troposphere ISA formula)
press_uniform = P0 * (1 - L.*alt_uniform./T0).^exponent;

% Sensor error model
b_baro_static  = 15;   % Static bias (Pa)  -> roughly translates to a few meters of altitude bias
std_baro_noise = 5;    % White noise std (Pa), typical of a MEMS barometer

baro_meas = press_uniform + b_baro_static + std_baro_noise*randn(N,1);

figure;
plot(time_uniform, press_uniform, "Color","blue"); hold on;
plot(time_uniform, baro_meas, "Color","red"); hold off;
legend('True pressure','Measured pressure');
xlabel('Time (s)'); ylabel('Pressure (Pa)');

simulatedSensorsData = table( ...
    time_uniform, ...
    acc_meas(:,1),acc_meas(:,2),acc_meas(:,3),gyro_meas(:,1),gyro_meas(:,2), ...
    gyro_meas(:,3),mag_meas(:,1),mag_meas(:,2),mag_meas(:,3), ...
    baro_meas, ...
    'VariableNames', { ...
    'Time_uniform', 'Acc_X','Acc_Y','Acc_Z',...
    'Gyro_P', 'Gyro_Q', 'Gyro_R', ...
    'Mag_X', 'Mag_Y', 'Mag_Z', ...
    'Baro'});

simulatedSensorsFile = 'simulatedSensorsData.xlsx';
writetable(simulatedSensorsData, simulatedSensorsFile);


