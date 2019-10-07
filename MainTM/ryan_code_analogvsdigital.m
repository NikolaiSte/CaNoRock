%-----Comparative plotting---------
%% Import data

load('CaNoRock_XVIII_Launch.mat')

A4 = Data1_A2_Acc_X__Raw;       %A2_Acc_X
A5 = Data1_A3_Acc_Y__Raw;       %A3_Acc_Y

FrameCounter = Data1_Count__Raw; 

time_offset = 174.8;                   %Check liftoff time to get correct time.
time = Data1_time_Count__Raw-time_offset;
time_digital = Data1_time_D1_GPSIMU__Raw;

FrameLock = Data1_FLOCK;
time_lock = Data1_time_FLOCK-time_offset;

%x axis analog
Analog4 = A4*5/255;         %Converts data to volts
AccXGain = 0.020;           %Sensitivity of the Accelerometer in V/g 
AccXOffset = 2.5;      %Calibrated at zero-g (nominal 2.5) %Save to DeweSoft
Analog4 = -(Analog4-AccXOffset)/AccXGain;    %Converts to G
Analog4_smooth = smooth(Analog4,1000);


% figure;
% subplot(2,1,1),plot(time,Analog4);
% grid on
% subplot(2,1,2),plot(time,Analog4_smooth);
% ylabel('[g]');              %Label for the Y-axis
% xlabel('Time [s]');         %Label for the X-axis (normally time)
% grid on
% title('Acceleration X');	%Plot title
% 

% y axis analog
Analog5 = A5*5/255;         %Converts data to volts
AccYGain = 0.040;           %Sensitivity of the Accelerometer in V/g 
AccYOffset = 2.6;     % Calibrated at zero-g (nominal 2.5) %Save to DeweSoft
Analog5 = (-(Analog5-AccYOffset)/AccYGain);    %Converts to G
Analog5_smooth = smooth(Analog5,1000);

% figure;             
% subplot(2,1,1),plot(time,Analog5);
% grid on
% subplot(2,1,2),plot(time,Analog5_smooth);
% ylabel('[g]');              %Label for the Y-axis
% xlabel('Time [s]');         %Label for the X-axis (normally time)
% grid on
% title('Acceleration Y');	%Plot title

%%

%figure (1) ;
%hold all
%plot(timeLsm, LsmAxData);
%plot(timeIcm, IcmAxData);
% axis([-5 90 -30 30]);
% title('Accelerometer x-axis verus Time')
% legend('LSM','ICM','Analog');
% xlabel('Time (s)')
% ylabel('Acceleration (g)')

% Compare analog to ICM accel in x
% figure;
% subplot(3,1,1),plot(time, Analog4_smooth);
% title('Analog Acceleration in principle axis ')
% axis([-5 20 -50 50]);
% ylabel('accelerationn [g]');              %Label for the Y-axis
% xlabel('Time [s]');         %Label for the X-axis (normally time)
% grid on
% 
% subplot(3,1,2),plot(timeLsm,smooth(LsmAxData,5));
% ylabel('acceleration [g]');              %Label for the Y-axis
% xlabel('Time [s]');         %Label for the X-axis (normally time)
% grid on
% title('LSM Acceleration in principle axis');	%Plot title
% axis([-5 20 -50 50]);
% 
% subplot(3,1,3),plot(timeLsm,smooth(IcmAxData,5));
% ylabel('acceleration [g]');              %Label for the Y-axis
% xlabel('Time [s]');         %Label for the X-axis (normally time)
% grid on
% title('ICM Acceleration in principle axis');	%Plot title
% axis([-5 20 -50 50]);

% Compare analog to ICM accel in y
% figure;
% subplot(3,1,1),plot(time, Analog5_smooth);
% title('Analog Acceleration in y')
% axis([-5 90 -50 50]);
% ylabel('[g]');              %Label for the Y-axis
% xlabel('Time [s]');         %Label for the X-axis (normally time)
% grid on
% 
% subplot(3,1,2),plot(timeIcm,smooth(IcmAyData,5));
% ylabel('[g]');              %Label for the Y-axis
% xlabel('Time [s]');         %Label for the X-axis (normally time)
% grid on
% title('ICM Acceleration in y');	%Plot title
% axis([-5 90 -50 50]);
% 
% subplot(3,1,3),plot(timeLsm,smooth(LsmAyData,5));
% ylabel('[g]');              %Label for the Y-axis
% xlabel('Time [s]');         %Label for the X-axis (normally time)
% grid on
% title('LSM Acceleration in y');	%Plot title
% axis([-5 90 -50 50]);


%%
%angular velocity from acceleration
hold all
r = 0.03;
ac = Analog5_smooth;
ac1 = sqrt(ac.^2);
omega_analog = sqrt(9.81*ac1/r)*180/pi;

subplot(3,1,1),plot(time,omega_analog);
ylabel('Angular velocity [dps]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Angular velocity from Analog Acceleration along Principal axis');	%Plot title'
axis([-5 20 0 10000]);

subplot(3,1,2),plot(timeLsm,smooth(LsmGxData,5));
ylabel('Angular velocity [dps]');           %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Angular velocity LSM along Principal axis');	%Plot title
xlim([-5 20]);
axis([-5 20 0 10000]);

subplot(3,1,3),plot(timeLsm,smooth(IcmGxData,5));
ylabel('Angular velocity [dps]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Angular velocity ICM along Principal axis');	%Plot title
axis([-5 20 0 10000]);