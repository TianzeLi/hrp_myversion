% Compute the discrete Frouier transform, using Fast Fourier transform,
% Data are extracted from rosbag into a csv file.
% ROS command to convert .bag file to .csv or .txt:
% rostopic echo -b <bagname>.bag -p <topic> > <csvname>.csv


mag_simpleloop_test = readtable('mag_left_simple_with_vision.csv', 'PreserveVariableNames', true);
% T0 = ??;           % Bag time length

% Choose what data to process
mag_data = mag_simpleloop_test;
Mag = [mag_data{:,5}, mag_data{:,6}, mag_data{:,7}];


Fs = 252;                   % Sampling frequency, average for IMU                    
T = 1/Fs;                   % Sampling period
[L,~] = size(mag_data);     % Length of signal
t = (0:L-1)*T;              % Time vector


% Online low-pass filter part
% acc_out_k+1 = (1 - alpha)*acc_in_k+1 + alpha*acc_out_k
% alpha = 0.2;
% Acc_lowpass = zeros(L,3);
% for i=1:3
%     for j=1:L
%         Acc_lowpass(j,i) = (1 - alpha)*Acc(j,i) + alpha*Acc_lowpass(j,i);
%     end
% end

% Offline low-pass filter
fc = 3;
Wn = (2/Fs)*fc;
b = fir1(20,Wn,'low',kaiser(21,3));
Mag_lowpass = filter(b,1,Mag);

% FFT 
Y = fft(Mag);
Y_lowpass = fft(Mag_lowpass);

% To elimate frequency around f-1 < k*16.82 < f + 1 
d1 = designfilt('bandstopiir','FilterOrder',2, ...
               'HalfPowerFrequency1',15.8,'HalfPowerFrequency2',17.8, ...
               'DesignMethod','butter','SampleRate',Fs);
d2 = designfilt('bandstopiir','FilterOrder',2, ...
               'HalfPowerFrequency1',32.6,'HalfPowerFrequency2',34.6, ...
               'DesignMethod','butter','SampleRate',Fs);
d3 = designfilt('bandstopiir','FilterOrder',2, ...
               'HalfPowerFrequency1',49.6,'HalfPowerFrequency2',51.6, ...
               'DesignMethod','butter','SampleRate',Fs);
buttLoop1 = filtfilt(d1,Mag_lowpass);
buttLoop2 = filtfilt(d2,buttLoop1);
buttLoop = filtfilt(d3,buttLoop2);
Y_bandkill = fft(buttLoop);

figure;
for i=1:3
    subplot(3,2,2*i-1)
    plot(t,Mag(:,i));
    grid on;
    title(['Mag ',num2str(i),' in the Time Domain'])
    subplot(3,2,2*i)
    plot(t,buttLoop(:,i))
    grid on;
    title(['Mag Filtered ',num2str(i),' in the Time Domain'])
end

           
% Compute the two-sided spectrum P2. 
% Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L. 
P2 = abs(Y/L);
P1 = P2((floor(1:L/2+1)),:);
P1(2:end-1,:) = 2*P1(2:end-1,:);

% Do the same for low-pass
P2_pass = abs(Y_bandkill/L);
P1_pass = P2_pass(floor(1:L/2+1),:);
P1_pass(2:end-1,:) = 2*P1_pass(2:end-1,:);


figure;
for i=1:3
    subplot(3,2,2*i-1)
    f = Fs*(0:(L/2))/L;
    plot(f,P1(:,i));
    grid on;
    title(['Mag ',num2str(i),' in the Freq-Domain'])
    subplot(3,2,2*i)
    plot(f,P1_pass(:,i));
    grid on;
    title(['Mag low-passed ',num2str(i),' in the Freq-Domain'])
end
