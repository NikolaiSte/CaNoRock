%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% This script takes sensor data from DeweSoft in .MAT-format              %
% from a student rocket launched from Andøya Space Center, and converts   %
% it into the actual units of the sensors.                                %
%                                                                         %
% NAROM AS                                                                %
% Last modified: 31.10.2017                                               %   
%                                                                          %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% -------------- import data from datafile ------
tic;
close all;

if (exist('time') == 0)
load('Z:\Courses\2019\CaNoRock XVIII\Rocket Data\MainTM\CaNoRock_XVIII_Launch.mat'); % If you don't want to open a new file every time use: load('"filename".mat');
end


%% Define the different channels in use and sort the data 
%Check if name of variabels are correct.
 A0 = Data1_A0_Int_T__Raw;          %A0_Int_T
 A1 = Data1_A1_Ext_T__Raw;        %A1_Ext_T
 A4 = Data1_A2_Acc_X__Raw;       %A2_Acc_X
 A5 = Data1_A3_Acc_Y__Raw;       %A3_Acc_Y
 A3 = Data1_A4_B_Field__Raw;           %A4_B_Field
 A6 = Data1_A5_Light__Raw;           %A5_Light
 A2 = Data1_A6_PTU__Raw;          %A6_PTU
 A7 = Data1_A7_Volt__Raw;           %A7_Volt
 D0 = Data1_D0_TempArray__Raw;      %D0_TempArray
 D1 = Data1_D1_GPSIMU__Raw;         %D1_GPS\IMU
 FrameCounter = Data1_Count__Raw; 

 time_offset = 174.8;                   %Check liftoff time to get correct time.
 time = Data1_time_Count__Raw-time_offset;
 time_digital = Data1_time_D1_GPSIMU__Raw;

FrameLock = Data1_FLOCK;
time_lock = Data1_time_FLOCK-time_offset;  
 Do_plots = 1; % Change to 0 if you don't want plots

%% FrameCounter 

FrameCounter_Lock = 0;
FrameCounter_Loss = 0;
for n=1:1:(length(FrameCounter)-1)
    if ((FrameCounter(n)+1) == (FrameCounter(n+1))||(FrameCounter(n+1))==0) 
        FrameCounter_Lock = FrameCounter_Lock+1;
    else
        FrameCounter_Loss = FrameCounter_Loss+1;
    end
end

if (Do_plots)
figure;             
plot(time,FrameCounter);
ylabel('Frame count');  %Label for the Y-axis
xlabel('Time [s]');     %Label for the X-axis (normally time)
grid on
title('Frame Counter [0 - 65534]');	%Plot title
numFC_L= num2str(FrameCounter_Lock);
txt=['Lock = ' numFC_L];
annotation('textbox',[0.9 .7 .0 .2],'String',txt,'EdgeColor','none');
numFC_Loss= num2str(FrameCounter_Loss);
txt=['Loss = ' numFC_Loss];
annotation('textbox',[0.9 .5 .0 .2],'String',txt,'EdgeColor','none');
end 

%% PCB Temperature sensor 
Analog0 = A0*5/255;         %Converts data to volts

PcbTempGain=11;%4.9;        %Save this to math in dewesoft

Analog0 = Analog0/(0.01*PcbTempGain);	%Converts to Celsius
Analog0_smooth = smooth(Analog0,1000);  %Removes some of the noise that will occur

if (Do_plots)
figure;             
subplot(2,1,1),plot(time,Analog0);         %Plotting unfiltred data
subplot(2,1,2), plot(time,Analog0_smooth); %Plotting smoothed data
%plot(time,Analog0);
ylabel('[ ]');                  %Label for the Y-axis
xlabel('Time [s]');             %Label for the X-axis (normally time)
grid on
title('Internal Temperature');  %Plot title
end

%% External Temperature sensor 
Analog1 = A1*5/255;         %Converts data to volts

ExtTempGain=4.9;%11;    %Save this to math in dewesoft            %

Analog1 = Analog1/(0.01*ExtTempGain);	%Converts to Celsius
Analog1_smooth = smooth(Analog1,1000);  %Takes average of 1000 samples

if (Do_plots)
figure;             
subplot(2,1,1),plot(time,Analog1);
subplot(2,1,2), plot(time,Analog1_smooth);
%plot(time,Analog1);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('External Temperature');	%Plot title
end

%% Air Pressure Sensor
Analog2 = A2*5/255;         %Converts data to volts

m = 0.475;          
n = 0.0045;
Analog2 = (222.22*Analog2+105.15);
Analog2_smooth = smooth(Analog2,1000);	%Converts to mBar

if (Do_plots)
figure;             
subplot(2,1,1),plot(time,Analog2);
subplot(2,1,2), plot(time,Analog2_smooth);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Pressure');          %Plot title
end

%% Magnetic field sensor Y 
Analog3 = A3*5/255;         %Converts data to volts
Analog3_smooth = smooth(Analog3,5);

if(Do_plots)
figure;             
subplot(2,1,1),plot(time,Analog3);
subplot(2,1,2),plot(time,Analog3_smooth);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Magnetic');          %Plot title
end

%% Accelerometer X-axis 
Analog4 = A4*5/255;         %Converts data to volts

AccXGain = 0.020;           %Sensitivity of the Accelerometer in V/g 
AccXOffset = 2.5;      %Calibrated at zero-g (nominal 2.5) %Save to DeweSoft

Analog4 = (Analog4-AccXOffset)/AccXGain;    %Converts to G
Analog4_smooth = smooth(Analog4,1000);

if(Do_plots)
figure;             
subplot(2,1,1),plot(time,Analog4);
subplot(2,1,2),plot(time,Analog4_smooth);
ylabel('[g]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Acceleration X');	%Plot title
end

%% Accelerometer Y-axis 
Analog5 = A5*5/255;         %Converts data to volts

AccYGain = 0.040;           %Sensitivity of the Accelerometer in V/g 
AccYOffset = 2.5095;     % Calibrated at zero-g (nominal 2.5) %Save to DeweSoft

Analog5 = (-(Analog5-AccYOffset)/AccYGain);    %Converts to G
Analog5_smooth = smooth(Analog5,1000);

if(Do_plots)
figure;             
subplot(2,1,1),plot(time,Analog5);
subplot(2,1,2),plot(time,Analog5_smooth);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Acceleration Y');	%Plot title
end

%% Light Sensor (Phototransistor) 
Analog6 = A6*11.16-30.25 ;        %Converts data to volts

if(Do_plots)
figure;             
plot(time,Analog6);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Light');             %Plot title
end

%% Battery output
Analog7 = A7*5/255*4.3;     %Converts data to volts (4.3 is the multiplier for the voltage divider)

if(Do_plots)
figure;             
plot(time,Analog7);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Battery');           %Plot title
end

%% Digital outputs
if(Do_plots)
figure;             
subplot(2,1,1),plot(time_digital,D0);
ylabel('D0[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
title('Temperature Array data');	%Plot title
subplot(2,1,2),plot(time_digital(1:end-1),D1);
ylabel('D1[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
title('GPS & IMU data');	%Plot title
grid on
end


%% GPS
latitude = Data1_Latitude__Raw*1E-7;
longitude = Data1_Longtitude__Raw*1E-7;
GPS_altitude = Data1_Altitude__Raw*0.01;
GPS_velocity = Data1_Velocity__Raw*0.01;
numberOfSatellites = Data1_Number_of_Satellites;

%%Ask christoffer for the GPS\IMU-script


%% IMU

IMU_Ax = Data1_Ax__Raw*0.000732;  %%Number constant derived from???
IMU_Ay = Data1_Ay__Raw*0.000732;
IMU_Az = Data1_Az__Raw*0.000732;
IMU_Gx = Data1_Gx__Raw*0.07;
IMU_Gy = Data1_Gy__Raw*0.07;
IMU_Gz = Data1_Gz__Raw*0.07;
IMU_Mx = Data1_Mx__Raw*0.00014*100;
IMU_My = Data1_My__Raw*0.00014*100;
IMU_Mz = Data1_Mz__Raw*0.00014*100;

%%Ask christoffer for the GPS\IMU-script



%% Temperatur array

A1_ = 3.354016E-3;
B1_ =  2.569850E-4;
C1_ =  2.620131E-6;
D1_ =  6.383091E-8;

R1 = 10000;

Temparray = double([Data1_A0__Raw Data1_A1__Raw Data1_A2__Raw Data1_A3__Raw Data1_A4__Raw Data1_A5__Raw Data1_A6__Raw Data1_A7__Raw Data1_A8__Raw Data1_A9__Raw]);
Timearray = [Data1_time_A0__Raw Data1_time_A1__Raw Data1_time_A2__Raw Data1_time_A3__Raw Data1_time_A4__Raw Data1_time_A5__Raw Data1_time_A6__Raw Data1_time_A7__Raw Data1_time_A8__Raw Data1_time_A9__Raw];
for k=1:length(Temparray)
Positionarray(k,:) = 0:5:45; %%A0 is startingposition and distance between each sensor have to be measured approx values
end
% R_NTC = R1*(4095.0-TA_frame)./TA_frame;
R_NTC = R1*Temparray./(4095.0-Temparray);
log_NTC = log(R_NTC/10000);
actual_tmp_frame = real(1./(A1_ + B1_*log_NTC + C1_*log_NTC.^2 + D1_*log_NTC.^3)-273.15);

if Do_plots
    figure;
    plot(Data1_time_A0__Raw, smooth(actual_tmp_frame(:,1),20),Data1_time_A0__Raw, actual_tmp_frame(:,1));
    
    hold on;
    for i=1:10
        plot(Timearray(:,i),  actual_tmp_frame(:,i));
    end
    hold off;
 %%   
    figure;
    s = surf(Timearray,Positionarray,actual_tmp_frame, 'EdgeColor','none');
    xlabel('Time [s]');
    ylabel('Position [cm]');
    zlabel('Temperature [C]');
    
end

%% Current
Current = Data1_A11__Raw/2;     %Converts data to volts (4.3 is the multiplier for the voltage divider)

if(Do_plots)
figure;             
plot(Data1_time_A11__Raw-time_offset,Current);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Current');           %Plot title
end

%% Height plot (Derived from pressure sensor) Case assignment for Telemetry
R=8.3145;                   %Specific gas constant
a=-0.0065;                  %Temperature gradient [K/m]
%T0=273.15+9.8;             %Starting temperature at ground (in kelvin) from Tower
%P0=1037.1;                  %Ground pressure in mBar from the Tower
T0= 273.15+Analog0_smooth(1);     %Starting temperature at ground (in kelvin) from the rocket.
P0= Analog2_smooth(1);            %Ground pressure in mBar from the rocket
M = 0.029;                  %Molecular mass of air.
g = 9.80665;

beta = -T0/a;
gamma = -(R*a)/(g*M);
%height = beta*(1-(Analog2/P0).^gamma); %Use of unfiltered pressure data
height = beta*(1-(Analog2_smooth/P0).^gamma); 

if(Do_plots)
figure;             
plot(time,height);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Height [m]');        %Plot title
end

%% Spinn speed (Derived from sentripetal acceleration) Case assignment for TM-Readout
radius = 31E-3;             %Accelerometer distanse to center of the rocket
L=500;
Fs = 1000;
spinn_acceleration = (sqrt(abs(Analog5_smooth)*g./radius)/(2*pi)-0.5); %Angular speed
% Y= (fft(Analog3_smooth)); %spinn_magneticfield =(fft(Analog3_smooth));
% P2=abs(Y/L);
% P1 = P2(1:L/2+1);
% P1(2:end-1)=2*P1(2:end-1);
% f=Fs*(0:(L/2))/L;

if(Do_plots)
figure;
plot(time,spinn_acceleration);
%subplot(2,1,1), plot(f,P1);
%subplot(2,1,2),plot(time,spinn_acceleration);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Spinn');             %Plot title
end

%% Velocity Case assignment for Rocket
offset = -0;                 %1.292; If the velocity is not constant 0 at the beginning use this offset to correct it.
velocity = -(cumtrapz(time,((g*(-Analog4_smooth+offset)))));

if(Do_plots)
figure;             
plot(time,velocity);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Velocity');          %Plot title
end


%% Framelock Caseassignment for Telemetry
if (Do_plots)

figure;             
plot(time_lock,FrameLock);
ylabel('[ ]');              %Label for the Y-axis
xlabel('Time [s]');         %Label for the X-axis (normally time)
grid on
title('Framelock');          %Plot title
end


% FrameCounter_Lock = 0;
% FrameCounter_Loss = 0;
% for n=1:1:(length(FrameCounter)-1)
%     if ((FrameCounter(n)+1) == (FrameCounter(n+1))||(FrameCounter(n+1))==0) 
%         FrameCounter_Lock = FrameCounter_Lock+1;
%     else
%         FrameCounter_Loss = FrameCounter_Loss+1;
%     end
% end

% Number of Framelock loss
%FL_nr = length(Analog0)-length(FrameLock);
ind_last = find(FrameLock == 2,1,'last');
FrameLock = FrameLock(1:ind_last);
time_lock = time_lock(1:ind_last);
ind0 = FrameLock == 0;
ind1 = FrameLock == 1;
ind2 = FrameLock == 2;
Number_Framelock = length (FrameLock(ind2));
Number_SearchFramelock = length (FrameLock(ind1));
Number_LossFramelock = length (FrameLock(ind0));




%% Cleaning Workspace (Removing unused variables)
% clear('dataset','rawdata');
% clear('Time','hours','minutes','seconds','offset_time','i','raw_text');
% clear ('PcbTempGain','ExtTempGain','AccXGain','AccXOffset','AccYGain','AccYOffset')
% clear ('m','n','radius');
% clear ('A0','A1','A2','A3','A4','A5','A6','A7');


%% Notes.. 

toc
