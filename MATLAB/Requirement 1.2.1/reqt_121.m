% MAE 491-01 Team 03 Requirement 1.1.2 Data MATLAB Parser

% Goal: parse angle and time data from Requirement 1.1.2 to verify
% whether the system is physically capable of rotating (by hand)
% between the angles of +/-90° with a tolerance of +/-5°

% housekeeping
clear
clc
close all
format compact 

filename = "data_1.csv"; % file name is set here for convenience

% threshold variables
nominalThresh = 180; % 90 degrees
threshBand = 5; % +/- 5 degrees for tolerance
maxThresh = nominalThresh+threshBand; % minimum value

% open csv data
log_data = readtable(filename,"VariableNamingRule","preserve");

% sort into arrays
time = table2array(log_data(:, 1)); % get first column from table
time = time/1000; % convert from source data in ms to seconds
angle = table2array(log_data(:, 2)); % get second column from table

% arrays for plotting threshold bands
nomThreshArray = nominalThresh * ones(1,length(time));
maxThreshArray = maxThresh * ones(1,length(time));

% compute max and min angles for analysis
maxAng = max(angle);
minAng = min(angle);

if (maxAng < maxThresh)
    if (minAng > -maxThresh)
        % print success & angles
        fprintf('The system meets the success criteria!\n')
        fprintf('Max angle = %.2f°\n',maxAng)
        fprintf('Min angle = %.2f°\n',minAng)
    else % didn't rotate past thresh. in -ve
        % print failure & angles
        fprintf('The system fails the success criteria.\n')
        fprintf('Rotated past -180°\n')
        fprintf('Max angle = %.2f°\n',maxAng)
        fprintf('Min angle = %.2f°\n',minAng)
    end
else % didn't rotate past +ve thresh.
    % print failures & angles
    fprintf('The system fails the success criteria.\n')
    fprintf('Rotated past +180°\n')
    fprintf('Max angle = %.2f°\n',maxAng)
    fprintf('Min angle = %.2f°\n',minAng)
end

% plot
hold on
plot(time, angle); % plot angle vs. time
% plot target values and tolerance bands
plot(time,nomThreshArray,'-.k')
plot(time,maxThreshArray,'-.r')
plot(time,-maxThreshArray,'-.k')
plot(time,-nomThreshArray,'-.r')
% add legend
legend('Angle','Nominal Threshold', 'Maximum Threshold','Location'...
    ,'bestoutside')
% title plot
title('Angle vs. Time for Requirement 1.2.1')
% label axes
xlabel('Time [s]')
ylabel('Angle [°]')













