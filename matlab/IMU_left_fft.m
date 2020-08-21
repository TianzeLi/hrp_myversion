% Compute the discrete Frouier transform, using Fast Fourier transform,
% Data are extracted from rosbag into a csv file.
% ROS command:
% rostopic echo -b <bagname>.bag -p <topic> > <csvname>.csv


IMU_simpleloop_test = readtable('simple_imu_left_bagdata.csv', 'PreserveVariableNames', true);
% T0 = 389;           % Bag time length
IMU_static = readtable('imu_left_static.csv', 'PreserveVariableNames', true);
% T0 = 20;
IMU_smooth_trans = readtable('imu_left_smooth_tran.csv', 'PreserveVariableNames', true);
% T0 = 47;
IMU_smooth_rotate = readtable('imu_left_smooth_rotate.csv', 'PreserveVariableNames', true);
% T0 = 57;
IMU_simulated_navi = readtable('imu_left_rough.csv', 'PreserveVariableNames', true);
% T0 = 40;
IMU_xshake = readtable('imu_left_xshake.csv', 'PreserveVariableNames', true);
T0 = 15;


Fs = 252;             % Sampling frequency, average for IMU                    
T = 1/Fs;             % Sampling period
L = T0/T;             % Length of signal
t = (0:L-1)*T;        % Time vector


% Choose what data to process
IMU_data = IMU_smooth_rotate;
Acc = [IMU_data{:,30}, IMU_data{:,31}, IMU_data{:,32}];

% Low pass filter part
% acc_out_k+1 = (1 - alpha)*acc_in_k+1 + alpha*acc_out_k
alpha = 0.05;
for i=1:3
    for j=1:L
        Acc_lowpass(j,i) = (1 - alpha)*Acc(:,i) + alpha 


Y = fft(Acc);

% Compute the two-sided spectrum P2. 
% Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L. 
P2 = abs(Y/L);
P1 = P2(1:L/2+1,:);
P1(2:end-1,:) = 2*P1(2:end-1,:);

figure;
for i=1:3
    subplot(3,1,i)
    f = Fs*(0:(L/2))/L;
    plot(f,P1(:,i));
    title(['Row ',num2str(i),' in the Frequency Domain'])
end
