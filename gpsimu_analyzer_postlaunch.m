% The rocket data must already have been imported. The user must give
% GPSIMU the digital raw data
% filename = 'gps_test1.dat';
% load(uigetfile('Canorock_secondlaunch_NAROM_TM'));
do_plot = 1;
D1 = Data1_D0_GPSIMU__Raw;

GPSIMU = uint16(D1); %dataset(:,11);
time_GPSIMU = Data1_time_A0_TempPCB;%time; %1:length(GPSIMU);
time_offset = 175.0457;
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


% tmp_sum_prev=0;
% ind = 0;
% 
% for i=1:bytes_pr_frame
%     tmp_sum = sum(GPSIMU(i:bytes_pr_frame:end) == sync0) ...
%         + sum(GPSIMU((i+1):bytes_pr_frame:end) == sync1);
%     if (tmp_sum > tmp_sum_prev)
%         tmp_sum_prev = tmp_sum;
%         ind = i;
%     end
% end

% time = time(ind:bytes_pr_frame/5:(length(time)-bytes_pr_frame));


% time_GPSIMU = time_GPSIMU(ind:bytes_pr_frame/5:(length(time)-bytes_pr_frame));
time_GPSIMU = time_GPSIMU(1:bytes_pr_frame/5:(length(time_GPSIMU)-bytes_pr_frame));
% GPSIMU = GPSIMU(ind:end);
GPSIMU_frame = reshape(GPSIMU(1:(length(time_GPSIMU)*bytes_pr_frame/5)),bytes_pr_frame/5,length(time_GPSIMU))';

time_GPS = time_GPSIMU(1:5:end);
GPS_raw = GPSIMU_frame(1:5:end,:);

inds_imu = find(GPSIMU_frame(:,2) ~= 161);
time_IMU = time_GPSIMU(inds_imu);
IMU_raw = GPSIMU_frame(inds_imu,:);

% Calculate actual GPS data
GPS_numsats = GPS_raw(:,3);
GPS_numsats(GPS_numsats > 63) = GPS_numsats(GPS_numsats > 63)-64;

GPS_latitude_uint8 = uint8(GPS_raw(:,4:7));
GPS_latitude = NaN(size(GPS_latitude_uint8,1),1);
for i=1:size(GPS_latitude_uint8,1)
    GPS_latitude(i) = typecast(GPS_latitude_uint8(i,[4 3 2 1]), 'int32');
end
GPS_latitude = double(GPS_latitude)*1E-7;

GPS_longitude_uint8 = uint8(GPS_raw(:,8:11));
GPS_longitude = NaN(size(GPS_longitude_uint8,1),1);
for i=1:size(GPS_longitude_uint8,1)
    GPS_longitude(i) = typecast(GPS_longitude_uint8(i,[4 3 2 1]), 'int32');
end
GPS_longitude = double(GPS_longitude)*1E-7;

GPS_altitude = double(bitand(GPS_raw(:,12),127))*256*256+double(GPS_raw(:,13))*256+double(GPS_raw(:,14));
inds = GPS_altitude > 2^22;
GPS_altitude(inds) = GPS_altitude(inds)-2^23;
GPS_altitude = GPS_altitude*0.01;

GPS_velocity = GPS_raw(:,15)*256*256 + GPS_raw(:,16)*256 + GPS_raw(:,17);
GPS_velocity = GPS_velocity*0.01;

% Calculate IMU data
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

IMU_Mx_uint16 = uint16(IMU_raw(:,15)*256+IMU_raw(:,16));
IMU_Mx = double(typecast(IMU_Mx_uint16,'int16'))*0.00014*100;
IMU_My_uint16 = uint16(IMU_raw(:,17)*256+IMU_raw(:,18));
IMU_My = double(typecast(IMU_My_uint16,'int16'))*0.00014*100;
IMU_Mz_uint16 = uint16(IMU_raw(:,19)*256+IMU_raw(:,20));
IMU_Mz = double(typecast(IMU_Mz_uint16,'int16'))*0.00014*100;
IMU_Mtot = sqrt(IMU_Mx.^2 + IMU_My.^2 + IMU_Mz.^2);

clear('inds_imu', 'GPS_latitude_uint8', 'GPS_longitude_uint8', ...
    'IMU_Ax_uint16', 'IMU_Ay_uint16', 'IMU_Az_uint16', 'IMU_Gx_uint16', ...
    'IMU_Gy_uint16', 'IMU_Gz_uint16', 'IMU_Mx_uint16', 'IMU_My_uint16', ...
    'IMU_Mz_uint16', 'tmp_sum', 'tmp_sum_prev', 'sync0', 'sync1', 'i'); %'', '', ''

if 1
    for i=1:length(GPS_altitude)
        fprintf('%f,%f,%f\n',GPS_longitude(i), GPS_latitude(i), GPS_altitude(i));
    end
end

if do_plot
    figure;
    plot(time_GPS, GPS_latitude);
    xlabel('time, s')
    ylabel('Latitude')
    title('Time versus Latitude')

    figure;
    plot(time_GPS, GPS_longitude);
    xlabel('time, s')
    ylabel('Longitude')
    title('Time versus Longitude')

    figure;
    plot(time_GPS, GPS_altitude);
    xlabel('time, s');
    ylabel('Altitude');
    title('Time versus Altitude');
    
    figure;
    plot(time_IMU, IMU_Atot);
    xlabel('Time, s');
    ylabel('Altitude');
    title('Time versus total accelerometer');
end