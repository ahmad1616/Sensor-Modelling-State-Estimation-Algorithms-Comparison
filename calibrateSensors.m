clear;clc;close all;

simSens = readtable("simulatedSensorsData.xlsx");
data = readtable("flight_data.csv");
uniformData = readtable("uniformDataFile.xlsx")

time_uniform = simSens.Time_uniform;

acc_meas = [simSens.Acc_X,simSens.Acc_Y,simSens.Acc_Z];
gyro_meas = [simSens.Gyro_P,simSens.Gyro_Q,simSens.Gyro_R];
mag_meas = [simSens.Mag_X,simSens.Mag_Y,simSens.Mag_Z];
baro_meas = simSens.Baro;

%% Accelerometer Calibration 
S_acc = diag([0.005, -0.003, 0.004]);     % Scale factor errors
M_acc = [  0,       0.001, -0.002;
          -0.001,   0,      0.0015;
           0.002,  -0.001,  0      ];    % Misalignment matrix
T_acc = eye(3) + M_acc + S_acc;
b_acc_static = [0.05; -0.03; 0.08];       % Static residual bias (m/s^2)

acc_cal = (inv(T_acc) * (acc_meas' - b_acc_static))'

figure;
plot(time_uniform,acc_cal(:,1),"Color","blue");
hold on;
plot(data.time,data.ax_body_fps2*0.3048,"Color","red")
hold off;
%% Gyroscope Calibration
S_gyro = diag([0.002, 0.002, -0.001]);
M_gyro = [ 0,       0.0005, 0.001;
          -0.0005,  0,     -0.0008;
          -0.001,   0.0008, 0     ];
T_gyro = eye(3) + M_gyro + S_gyro;
b_gyro_static = [0.01; -0.015; 0.008];    % Static Gyro Bias (rad/s)

gyro_cal = (inv(T_gyro) * (gyro_meas' - b_gyro_static))'


figure;
plot(time_uniform,gyro_cal(:,1),"Color","blue");
hold on;
plot(data.time,data.p_degps*pi/180,"Color","red")
hold off;

%% Magnetometer Calbration 
A_soft_iron = [ 1.05, 0.03, 0.01;
                0.03, 0.96, 0.02;
                0.01, 0.02, 0.98 ];
V_hard_iron = [ 12.5; -8.3; 22.0 ];       % uT
mag_cal = (inv(A_soft_iron) * (mag_meas' - V_hard_iron))'

figure;
plot(time_uniform,mag_cal(:,1),"Color","blue");
hold on;
plot(time_uniform,mag_meas(:,1),"Color","red")
hold off;
calibratedSensorsData = table( ...
    time_uniform, ...
    acc_cal(:,1),acc_cal(:,2),acc_cal(:,3),gyro_cal(:,1),gyro_cal(:,2), ...
    gyro_cal(:,3),mag_cal(:,1),mag_cal(:,2),mag_cal(:,3), ...
    'VariableNames', { ...
    'Time_uniform', 'Acc_X_cal','Acc_Y_cal','Acc_Z_cal',...
    'Gyro_P_cal', 'Gyro_Q_cal', 'Gyro_R_cal', ...
    'Mag_X_cal', 'Mag_Y_cal', 'Mag_Z_cal'});

calibratedSensorsFile = 'calibratedSensorsData.xlsx';
writetable(calibratedSensorsData, calibratedSensorsFile);




