% MAE 491-01 Team 03 Requirement 1.1.3 Data MATLAB Parser
% Written by Ian Holbrook

% Goal: parse angle, time, and pressure data for Requirement 1.1.3
% to verify that the pressure in one nozzle is at least 20 +/-4 psi
% in response to a 45° step input *after* the system has reached
% steady state

% steady state is defined as the input +/-5°, so a band of 40-50°

% housekeeping
clear
clc
close all
format compact 

filename = "data_1.csv"; % file name is set here for convenience

% threshold variables for settling
nominalThresh = 45; % 45 degrees
threshBand = 5; % +/- 5 degrees for tolerance
minThresh = nominalThresh-threshBand; % minimum value
maxThresh = nominalThresh+threshBand; % maximum value

% pressure thresholds
nominalPressure = 20;
pressureThresh = 4;
minPressure = nominalPressure - pressureThresh;

% open csv data
log_data = readtable(filename,"VariableNamingRule","preserve");

% sort into arrays
time = table2array(log_data(:, 1)); % get first column from table
time = time/1000; % convert from source data in ms to seconds
angle = table2array(log_data(:, 2)); % get second column from table
% get pressures
pressureTop = table2array(log_data(:,4));
pressureBot = table2array(log_data(:,5));


% arrays for plotting threshold bands
nomThreshArray = nominalThresh * ones(1,length(time));
minThreshArray = minThresh * ones(1,length(time));
maxThreshArray = maxThresh * ones(1,length(time));

% arrays for plotting pressure bands

nomPressureArray = nominalPressure*ones(1,length(time));
minPressureArray = minPressure*ones(1,length(time));

% parse angle data to find steady state

logicalArray = abs(angle)>minThresh & abs(angle)<maxThresh; 
    % contains logical variables with indices corresponding to each 
    % value in angle

val = 0; % logical variable

% work in reverse order.
% ASSUME system is settled at end of test. must add to test procedures
for i = length(angle):-1:1
    if val == 1
        logicalArray(i) = 0;
        continue
    end
    if logicalArray(i) == 0
        val = 1;
    end
end

% find settled pressures
steadyPressureTop = pressureTop(logicalArray);
steadyPressureBot = pressureBot(logicalArray);

% determine dominant nozzle and analyze to make sure always in excess
if sign(angle(end)) == 1
    % settled angle is positive so "bottom" nozzle is the dominant one
    if steadyPressureBot > minPressure
        fprintf('The pressure exceeds the required threshold after\n')
        fprintf('settling, so the test is passed.\n')
    else
        fprintf('The pressure does not always exceed the required\n')
        fprintf('threshold after settling, so the test is failed.\n')
    end
else
    % dominant nozzle is top
    if steadyPressureTop > minPressure
        fprintf('The pressure exceeds the required threshold after\n')
        fprintf('settling, so the test is passed.\n')
    else
        fprintf('The pressure does not always exceed the required\n')
        fprintf('threshold after settling, so the test is failed.\n')
    end
end


% plot
hold on
plot(time, angle); % plot angle vs. time
% plot target values and tolerance bands
if sign(angle(end)) == 1
    plot(time,nomThreshArray,'-.k')
    plot(time,minThreshArray,'-.r')
else
    plot(time,-nomThreshArray,'-.k')
    plot(time,-minThreshArray,'-.r')
end

% add legend
legend('Angle','Nominal Threshold', 'Minimum Threshold','Location'...
    ,'bestoutside')
% title plot
title('Angle vs. Time for Requirement 1.1.3')
% label axes
xlabel('Time [s]')
ylabel('Angle [°]')
hold off

% create pressure plot
figure
hold on
if sign(angle(end)) == 1
    plot(time(logicalArray),steadyPressureBot)
else
    plot(time(logicalArray),steadyPressureTop)
end
plot(time(logicalArray),nomPressureArray(logicalArray),'-.k')
plot(time(logicalArray),minPressureArray(logicalArray),'-.r')
legend('Pressure','Nominal Threshold', 'Minimum Threshold','Location'...
    ,'bestoutside')
title('Pressure vs. Time at Steady State for Requirement 1.1.3')
% label axes
xlabel('Time [s]')
ylabel('Pressure [psig]')














