function [meas, meas_, timeline, Ts, Fs, HW_cutoff, SW_cutoff, info] = load_measures(selector)
%LOAD_MEASURES - Loads the experimental measurements set to characterize
% You can add your new data-set in this function. Just be sure to enter
% them in a new 'case' option in the code and to add the data set to the
% 'load_me_IMU.mat' file. Also add a line in the main.m experiment list
% with a brief description of your dataset.
% (e.g. % SET 0  : exp 0 - ACCELEROMETERS  [ 42 Hz, 30 Hz, 200 Hz ]') 
%
% Syntax:  [meas, meas_, timeline, Ts, Fs, HW_cutoff, SW_cutoff, info] = load_measures(selector)
%
% Inputs:
%    selector - Index of the experiment to load.
%
% Outputs:
%    meas - experiment data-set - [nxm] matrix.
%    meas_ - average removed data-set - [nxm] matrix.
%    timeline - a time base for the measurements - [nx1] vector. 
%    Ts - sampling time - scalar.
%    Fs - sampling frequency - scalar.
%    HW_cutoff - cut-off frequency of the hardware low-pass filter - scalar.
%    SW_cutoff - cut-off frequency of the software low-pass filter - scalar.
%    info - information about the data-set - struct.
%
% Other m-files required: vline.m, hline.m (download these from 
% http://www.mathworks.com/matlabcentral/fileexchange/1039-hline-and-vline) 
% MAT-files required: load_me_IMU.mat

% Author: Antonio Toma
% email: antonio.toma@outlook.com
% November 2104; Last revision: 14-Nov-2014

%------------- BEGIN CODE --------------

load load_me_IMU.mat;

switch selector
    
    case 0
        %% SET 0 - ACCELEROMETERS
        % 42 Hz hardware low pass filter
        % 30 Hz software low pass filter
        % 200 Hz mavlink framerate
        
        info.type = 'acc';
        
        meas = [fieldlinear_accelerationx0(2:end,:), ...
            fieldlinear_accelerationy0(2:end,:), ...
            fieldlinear_accelerationz0(2:end,:)];
        
        meas_ = [fieldlinear_accelerationx0(2:end,:) - mean(fieldlinear_accelerationx0(2:end,:)), ...
            fieldlinear_accelerationy0(2:end,:) - mean(fieldlinear_accelerationy0(2:end,:)), ...
            fieldlinear_accelerationz0(2:end,:) - mean(fieldlinear_accelerationz0(2:end,:))];
        
        timeline = (timestamp0(2:end) - timestamp0(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = 30;
        HW_cutoff = 42;
        
    case 1
        %% SET 0 - GYROSCOPES
        % 42 Hz hardware low pass filter
        % 30 Hz software low pass filter
        % 200 Hz mavlink framerate
        
        info.type = 'gyr';
        
        meas = [fieldangular_velocityx0(2:end,:), ...
            fieldangular_velocityy0(2:end,:), ...
            fieldangular_velocityz0(2:end,:)];
        
        meas_ = [fieldangular_velocityx0(2:end,:) - mean(fieldangular_velocityx0(2:end,:)), ...
            fieldangular_velocityy0(2:end,:) - mean(fieldangular_velocityy0(2:end,:)), ...
            fieldangular_velocityz0(2:end,:) - mean(fieldangular_velocityz0(2:end,:))];
        
        timeline = (timestamp0(2:end) - timestamp0(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = 30;
        HW_cutoff = 42;
        
    case 2
        %% SET 1 - SHAKING IMU - ACCELEROMETERS
        % 42 Hz hardware low pass filter
        % 30 Hz software low pass filter
        % 200 Hz mavlink framerate
        
        info.type = 'acc';
        
        meas = [fieldlinear_accelerationx1(2:end,:), ...
            fieldlinear_accelerationy1(2:end,:), ...
            fieldlinear_accelerationz1(2:end,:)];
        
        meas_ = [fieldlinear_accelerationx1(2:end,:) - mean(fieldlinear_accelerationx1(2:end,:)), ...
            fieldlinear_accelerationy1(2:end,:) - mean(fieldlinear_accelerationy1(2:end,:)), ...
            fieldlinear_accelerationz1(2:end,:) - mean(fieldlinear_accelerationz1(2:end,:))];
        
        timeline = (timestamp1(2:end) - timestamp1(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = 30;
        HW_cutoff = 42;
        
    case 3
        %% SET 1 - SHAKING IMU - GYROSCOPES
        % 42 Hz hardware low pass filter
        % 30 Hz software low pass filter
        % 200 Hz mavlink framerate
        
        info.type = 'gyr';
        
        meas = [fieldangular_velocityx1(2:end,:), ...
            fieldangular_velocityy1(2:end,:), ...
            fieldangular_velocityz1(2:end,:)];
        
        meas_ = [fieldangular_velocityx1(2:end,:) - mean(fieldangular_velocityx1(2:end,:)), ...
            fieldangular_velocityy1(2:end,:) - mean(fieldangular_velocityy1(2:end,:)), ...
            fieldangular_velocityz1(2:end,:) - mean(fieldangular_velocityz1(2:end,:))];
        
        timeline = (timestamp1(2:end) - timestamp1(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = 30;
        HW_cutoff = 42;
        
    case 4
        %% SET 2 - ACCELEROMETERS
        % 10Hz hardware low pass filter
        % 30 Hz software low pass filter
        % 200 Hz mavlink framerate)
        
        info.type = 'acc';
        
        meas = [fieldlinear_accelerationx2(2:end,:), ...
            fieldlinear_accelerationy2(2:end,:), ...
            fieldlinear_accelerationz2(2:end,:)];
        
        meas_ = [fieldlinear_accelerationx2(2:end,:) - mean(fieldlinear_accelerationx2(2:end,:)), ...
            fieldlinear_accelerationy2(2:end,:) - mean(fieldlinear_accelerationy2(2:end,:)), ...
            fieldlinear_accelerationz2(2:end,:) - mean(fieldlinear_accelerationz2(2:end,:))];
        
        timeline = (timestamp2(2:end) - timestamp2(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = 30;
        HW_cutoff = 10;
        
    case 5
        %% SET 2 - GYROSCOPES
        % 10Hz hardware low pass filter
        % 30 Hz software low pass filter
        % 200 Hz mavlink framerate)
        
        info.type = 'gyr';
        
        meas = [fieldangular_velocityx2(2:end,:), ...
            fieldangular_velocityy2(2:end,:), ...
            fieldangular_velocityz2(2:end,:)];
        
        meas_ = [fieldangular_velocityx2(2:end,:) - mean(fieldangular_velocityx2(2:end,:)), ...
            fieldangular_velocityy2(2:end,:) - mean(fieldangular_velocityy2(2:end,:)), ...
            fieldangular_velocityz2(2:end,:) - mean(fieldangular_velocityz2(2:end,:))];
        
        timeline = (timestamp2(2:end) - timestamp2(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = 30;
        HW_cutoff = 10;
        
    case 6
        %% SET 3 - ACCELEROMETERS
        % 10Hz hardware low pass filter
        % raw (software unfiltered) sens measurements
        % 200 Hz mavlink framerate
        
        info.type = 'acc';
        
        meas = [fieldlinear_accelerationx3(2:end,:), ...
            fieldlinear_accelerationy3(2:end,:), ...
            fieldlinear_accelerationz3(2:end,:)];
        
        meas_ = [fieldlinear_accelerationx3(2:end,:) - mean(fieldlinear_accelerationx3(2:end,:)), ...
            fieldlinear_accelerationy3(2:end,:) - mean(fieldlinear_accelerationy3(2:end,:)), ...
            fieldlinear_accelerationz3(2:end,:) - mean(fieldlinear_accelerationz3(2:end,:))];
        
        timeline = (timestamp3(2:end) - timestamp3(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = Fs/2;
        HW_cutoff = 10;
        
    case 7
        %% SET 4 - ACCELEROMETERS
        % 10Hz hardware low pass filter
        % raw (software unfiltered) sens measurements
        % 500 Hz mavlink framerate
        
        info.type = 'acc';
        
        meas = [fieldlinear_accelerationx4(2:end,:), ...
            fieldlinear_accelerationy4(2:end,:), ...
            fieldlinear_accelerationz4(2:end,:)];
        
        meas_ = [fieldlinear_accelerationx4(2:end,:) - mean(fieldlinear_accelerationx4(2:end,:)), ...
            fieldlinear_accelerationy4(2:end,:) - mean(fieldlinear_accelerationy4(2:end,:)), ...
            fieldlinear_accelerationz4(2:end,:) - mean(fieldlinear_accelerationz4(2:end,:))];
        
        timeline = (timestamp4(2:end) - timestamp4(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = Fs/2;
        HW_cutoff = 10;
        
    case 8
        %% SET 5 - ACCELEROMETERS
        % 98Hz hardware low pass filter
        % 30 Hz software low pass filter (I forgot to change for acc, it's
        % ok for gyro, for accs see SET 6)
        % 200 Hz mavlink framerate
        
        info.type = 'acc';
        
        SW_cutoff = 30;
        HW_cutoff = 98;
        
        meas = [fieldlinear_accelerationx5(2:end,:), ...
            fieldlinear_accelerationy5(2:end,:), ...
            fieldlinear_accelerationz5(2:end,:)];
        
        meas_ = [fieldlinear_accelerationx5(2:end,:) - mean(fieldlinear_accelerationx5(2:end,:)), ...
            fieldlinear_accelerationy5(2:end,:) - mean(fieldlinear_accelerationy5(2:end,:)), ...
            fieldlinear_accelerationz5(2:end,:) - mean(fieldlinear_accelerationz5(2:end,:))];
        
        timeline = (timestamp5(2:end) - timestamp5(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
    case 9
        %% SET 5 - GYROSCOPES
        % 98Hz hardware low pass filter
        % raw (software unfiltered) sens measurements
        % 200 Hz mavlink framerate
        
        info.type = 'gyr';
        
        meas = [fieldangular_velocityx5(2:end,:), ...
            fieldangular_velocityy5(2:end,:), ...
            fieldangular_velocityz5(2:end,:)];
        
        meas_ = [fieldangular_velocityx5(2:end,:) - mean(fieldangular_velocityx5(2:end,:)), ...
            fieldangular_velocityy5(2:end,:) - mean(fieldangular_velocityy5(2:end,:)), ...
            fieldangular_velocityz5(2:end,:) - mean(fieldangular_velocityz5(2:end,:))];
        
        timeline = (timestamp5(2:end) - timestamp5(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = Fs/2;
        HW_cutoff = 98;
        
    case 10
        %% SET 6 - ACCELEROMETERS
        % 98Hz hardware low pass filter
        % raw (software unfiltered) sens measurements
        % 200 Hz mavlink framerate
        
        info.type = 'acc';
        
        meas = [fieldlinear_accelerationx6(2:end,:), ...
            fieldlinear_accelerationy6(2:end,:), ...
            fieldlinear_accelerationz6(2:end,:)];
        
        meas_ = [fieldlinear_accelerationx6(2:end,:) - mean(fieldlinear_accelerationx6(2:end,:)), ...
            fieldlinear_accelerationy6(2:end,:) - mean(fieldlinear_accelerationy6(2:end,:)), ...
            fieldlinear_accelerationz6(2:end,:) - mean(fieldlinear_accelerationz6(2:end,:))];
        
        timeline = (timestamp6(2:end) - timestamp6(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = Fs/2;
        HW_cutoff = 98;
        
    case 11
        %% SET 6 - GYROSCOPES
        % 98Hz hardware low pass filter
        % raw (software unfiltered) sens measurements
        % 200 Hz mavlink framerate
        
        info.type = 'gyr';
        
        meas = [fieldangular_velocityx6(2:end,:), ...
            fieldangular_velocityy6(2:end,:), ...
            fieldangular_velocityz6(2:end,:)];
        
        meas_ = [fieldangular_velocityx6(2:end,:) - mean(fieldangular_velocityx6(2:end,:)), ...
            fieldangular_velocityy6(2:end,:) - mean(fieldangular_velocityy6(2:end,:)), ...
            fieldangular_velocityz6(2:end,:) - mean(fieldangular_velocityz6(2:end,:))];
        
        timeline = (timestamp6(2:end) - timestamp6(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = Fs/2;
        HW_cutoff = 98;
        
    case 12
        %% SET 7 - MAGNETOMETER
        % unknown hardware low pass filter
        % unknown software low pass filter
        % 80 Hz mavlink framerate
        
        info.type = 'mag';
        
        meas = [fieldmagnetic_fieldx7(2:end), ...
            fieldmagnetic_fieldy7(2:end), ...
            fieldmagnetic_fieldz7(2:end)];
        
        meas_ = [fieldmagnetic_fieldx7(2:end) - mean(fieldmagnetic_fieldx7(2:end)), ...
            fieldmagnetic_fieldy7(2:end) - mean(fieldmagnetic_fieldy7(2:end)), ...
            fieldmagnetic_fieldz7(2:end) - mean(fieldmagnetic_fieldz7(2:end))];
        
        timeline = (timestamp7(2:end) - timestamp7(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = Fs/2;
        HW_cutoff = Fs/2;
        
    case 13
        %% SET 7 - GPS - ECEF
        % unknown hardware low pass filter
        % unknown software low pass filter
        % 5 Hz mavlink framerate
        
        info.type = 'gps_ecef';
        
        lat = deg2rad(fieldlatitude7(2:end));
        lon = deg2rad(fieldlongitude7(2:end));
        alt = deg2rad(fieldaltitude7(2:end));
        
        [meas(:,1), meas(:,2), meas(:,3)] = lla2ecef(lat,lon,alt);
        
        meas = [meas(:,1) - meas(1,1), meas(:,2) - meas(1,2), meas(:,3) - meas(1,3)];
        avg = mean(meas);
        
        meas_(:,1) = (meas(:,1) - avg(1));
        meas_(:,2) = (meas(:,2) - avg(2));
        meas_(:,3) = (meas(:,3) - avg(3));
        
        timeline = (timestamp7GPS(2:end) - timestamp7GPS(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = Fs/2;
        HW_cutoff = Fs/2;
        
    case 14
        %% SET 8 - GPS - ECEF
        % unknown hardware low pass filter
        % unknown software low pass filter
        % 5 Hz mavlink framerate
        
        info.type = 'gps_ecef';
        
        lat = deg2rad(fieldlatitude8(2:end));
        lon = deg2rad(fieldlongitude8(2:end));
        alt = (fieldaltitude8(2:end));
        
        [meas(:,1), meas(:,2), meas(:,3)] = lla2ecef(lat,lon,alt);
        
        meas = [meas(:,1) - meas(1,1), meas(:,2) - meas(1,2), meas(:,3) - meas(1,3)];
        avg = mean(meas);
        
        meas_(:,1) = (meas(:,1) - avg(1));
        meas_(:,2) = (meas(:,2) - avg(2));
        meas_(:,3) = (meas(:,3) - avg(3));
        
        timeline = (timestamp8(2:end) - timestamp8(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = Fs/2;
        HW_cutoff = Fs/2;
        
    case 15
        %% SET 8 - GPS - RADIANS
        % unknown hardware low pass filter
        % unknown software low pass filter
        % 5 Hz mavlink framerate
        
        info.type = 'gps_rad';
        
        lat = deg2rad(fieldlatitude8(2:end));
        lon = deg2rad(fieldlongitude8(2:end));
        alt = (fieldaltitude8(2:end));
        
        meas = [lat,lon,alt];
        
        meas = [meas(:,1) - meas(1,1), meas(:,2) - meas(1,2), meas(:,3) - meas(1,3)];
        avg = mean(meas);
        
        meas_(:,1) = (meas(:,1) - avg(1));
        meas_(:,2) = (meas(:,2) - avg(2));
        meas_(:,3) = (meas(:,3) - avg(3));
        
        timeline = (timestamp8(2:end) - timestamp8(2))/1000000000;
        Ts = timeline(end)/length(timeline);
        Fs = 1/Ts;
        
        SW_cutoff = Fs/2;
        HW_cutoff = Fs/2;
end
end

%------------- END OF CODE --------------
%Please send suggestions for improvement of the above template header 
%to Denis Gilbert at this email address: gilbertd@dfo-mpo.gc.ca.
%Your contribution towards improving this template will be acknowledged in
%the "Changes" section of the TEMPLATE_HEADER web page on the Matlab
%Central File Exchange