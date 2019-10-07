% The rocket data must already have been imported. The user must give
% GPSIMU the digital raw data
close all;
INV_accel_conv = 1.334050546448087;

INV_gyro_conv = 1.74216;

do_plot = 1;
D1 = Data1_D1_GPSIMU__Raw;

GPSIMU = uint16(D1); %dataset(:,11);
time_GPSIMU = Data1_time_D1_GPSIMU;%time; %1:length(GPSIMU);
time_offset = 174.8;
time_GPSIMU = time_GPSIMU-time_offset;

bytes_pr_frame = 5*(2+18);

sync0 = 160;
sync1 = 161;

inds_sync = find(and(GPSIMU(1:end-1) == 160, GPSIMU(2:end) == 161));
tot_inds = NaN(2*length(D1),1);

counter = 1;
for i=1:length(inds_sync)
    tot_inds(counter:counter+bytes_pr_frame-1) = [inds_sync(i) (1:bytes_pr_frame-1)+inds_sync(i)];
    counter = counter+bytes_pr_frame;
end

tot_inds = tot_inds(~isnan(tot_inds));

tot_inds = tot_inds(1:end-2*bytes_pr_frame);
tot_inds = tot_inds(~isnan(tot_inds));

time_GPSIMU = time_GPSIMU(tot_inds);
GPSIMU = GPSIMU(tot_inds);

time_GPSIMU = time_GPSIMU(1:bytes_pr_frame/5:(length(time_GPSIMU)-bytes_pr_frame));
GPSIMU_frame = reshape(GPSIMU(1:(length(time_GPSIMU)*bytes_pr_frame/5)),bytes_pr_frame/5,length(time_GPSIMU))';

time_GPS = time_GPSIMU(1:5:end);
GPS_raw = GPSIMU_frame(1:5:end,:);

inds_imu = find(GPSIMU_frame(:,2) ~= 161);
time_IMU = time_GPSIMU(inds_imu);
IMU_raw = GPSIMU_frame(inds_imu,:);


%Calculate IMU data
IMU_Ax_uint16 = uint16(IMU_raw(:,3)*256+IMU_raw(:,4));
IMU_Ax = double(typecast(IMU_Ax_uint16,'int16'))*0.000732;
IMU_Ay_uint16 = uint16(IMU_raw(:,5)*256+IMU_raw(:,6));
IMU_Ay = double(typecast(IMU_Ay_uint16,'int16'))*0.000732;
IMU_Az_uint16 = uint16(IMU_raw(:,7)*256+IMU_raw(:,8));
IMU_Az = double(typecast(IMU_Az_uint16,'int16'))*0.000732;
IMU_Atot = sqrt(IMU_Ax.^2 + IMU_Ay.^2 + IMU_Az.^2);

IMU_Gx_uint16 = uint16(IMU_raw(:,9)*256+IMU_raw(:,10));
IMU_Gx = double(typecast(IMU_Gx_uint16,'int16'))*0.07;
IMU_Gy_uint16 = uint16(IMU_raw(:,11)*256+IMU_raw(:,12));
IMU_Gy = double(typecast(IMU_Gy_uint16,'int16'))*0.07;
IMU_Gz_uint16 = uint16(IMU_raw(:,13)*256+IMU_raw(:,14));
IMU_Gz = double(typecast(IMU_Gz_uint16,'int16'))*0.07;
IMU_Gtot = sqrt(IMU_Gx.^2 + IMU_Gy.^2 + IMU_Gz.^2);


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%this code is for accelerometer data splicing 
LsmAxData = IMU_Ax(1:4:end,:);
IcmAxData = -1*IMU_Ax(3:4:end,:)*INV_accel_conv;

LsmAyData = IMU_Ay(1:4:end,:);
IcmAyData = IMU_Ay(3:4:end,:)*INV_accel_conv;

LsmAzData = IMU_Az(1:4:end,:);
IcmAzData = IMU_Az(3:4:end,:)*INV_accel_conv;

IcmATot = sqrt(IcmAxData.^2 + IcmAyData.^2 + IcmAzData.^2);

timeLsm = time_IMU(1:4:end,:);
timeIcm = time_IMU(3:4:end,:);

figure (1) ;
hold all
plot(timeLsm, smooth(LsmAxData,5));
plot(timeIcm, smooth(IcmAxData,5));
xlim([-5 84]);
title('Acceleration in Principle Axis versus Time')
legend('LSM','ICM');
xlabel("Time [s]");
ylabel("Acceleration [g]");

figure (2) ;
hold all
plot(timeLsm, smooth(LsmAyData,5));
plot(timeIcm, smooth(IcmAyData,5));
xlim([-5 84]);
title('Accelerometer Y Data verus Time')
legend('LSM','ICM');
xlabel("Time [s]");
ylabel("Acceleration in perpendicular plane [g]");

figure (3) ;
hold all
plot(timeLsm, smooth(LsmAzData, 5));
plot(timeIcm, smooth(IcmAzData, 5));
xlim([-5 84]);
title('Accelerometer Z Data verus Time')
legend('LSM','ICM');
xlabel("Time [s]");
ylabel("Acceleration in perpendicular plane [g]");

figure(4);
hold all
plot(timeIcm, smooth(IcmATot,5));
title('Total accelerometer Data verus Time')
xlabel("Time [s]");
ylabel("Acceleration in perpendicular plane [g]");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This code is for gyroscope splicing

LsmGxData = (-1)*IMU_Gx(1:4:end,:);
IcmGxData = IMU_Gx(3:4:end,:)*INV_gyro_conv;
 
LsmGyData = IMU_Gy(1:4:end,:);
IcmGyData = IMU_Gy(3:4:end,:)*INV_gyro_conv;

LsmGzData = IMU_Gz(1:4:end,:);
IcmGzData = IMU_Gz(3:4:end,:)*INV_gyro_conv;

IcmGTot = sqrt(IcmGxData.^2 + IcmGyData.^2 + IcmGzData.^2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot the gyroscope data
figure ;
hold all
plot(timeLsm, LsmGxData);
plot(timeIcm, IcmGxData);
legend('LSM','ICM');
xlim([-5 84]);
xlabel('Time (s)')
ylabel('Angular Velocity (dps)')
ylim([-4200 100]);
%axis([-5 90 -50 4200]);
title('Angular Velocity along Principle Axis');

figure ;
hold all
plot(timeLsm, LsmGyData);
plot(timeIcm, IcmGyData);
legend('LSM','ICM');
xlim([-5 84]);
%axis([-5 90 -50 2000]);
title('Gyroscope Y Data verus Time [dps]');

figure ;
hold all
plot(timeLsm, LsmGzData);
plot(timeIcm, IcmGzData);
legend('LSM','ICM');
xlim([-5 84]);
%axis([-5 90 -50 3200]);
title('Gyroscope Z Data verus Time [dps]');
xlabel('time [s]');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%now compute magnetometer data
IMU_Mx_uint16 = uint16(IMU_raw(:,15)*256+IMU_raw(:,16));
IMU_Mx = double(typecast(IMU_Mx_uint16,'int16'))*0.014;
IMU_My_uint16 = uint16(IMU_raw(:,17)*256+IMU_raw(:,18));
IMU_My = double(typecast(IMU_My_uint16,'int16'))*0.014;
IMU_Mz_uint16 = uint16(IMU_raw(:,19)*256+IMU_raw(:,20));
IMU_Mz = double(typecast(IMU_Mz_uint16,'int16'))*0.014;
IMU_Mtot = sqrt(IMU_Mx.^2 + IMU_My.^2 + IMU_Mz.^2);


 figure;
 hold all
 plot(time_IMU, IMU_Mx);
 xlim([-5 84]);
 legend('LSM');
 title('Magnetometer X Data verus Time m[G]');
 
 figure;
 hold all
 plot(time_IMU, IMU_My);
 xlim([-5 84]);
 legend('LSM');
 title('Magnetometer Y Data verus Time m[G]');
 
 figure;
 hold all
 xlim([-5 84]);
 plot(time_IMU, IMU_Mz);
 legend('LSM');
 title('Magnetometer Z Data verus Time m[G]');