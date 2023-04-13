% MAE 491-01 Team 03 Requirement 1.3.1 Data MATLAB Parser
% Written by Ian Holbrook

% Goal:

% steady state is defined as the input +/-5°, so a band of 40-50°

% housekeeping
clear
clc
close all
format compact 

%% 30 degree data

filename = "Req_1_3_2_angle_30_data.csv"; % file name is set here for convenience

% threshold variables for settling
nominalThresh = 30; % SET TARGET HERE
threshBand = 5; % +/- 5 degrees for tolerance
minThresh = nominalThresh-threshBand; % minimum value
maxThresh = nominalThresh+threshBand; % maximum value

% settling time requirement
nominalSettle = 2.5;
threshSettle = 0.5;
maxSettle = nominalSettle + threshSettle;

% open csv data
log_data = readtable(filename,"VariableNamingRule","preserve");

% sort into arrays
time = table2array(log_data(:, 1)); % get first column from table
time = time/1000; % convert from source data in ms to seconds
angle = table2array(log_data(:, 2)); % get second column from table


% arrays for plotting threshold bands
nomThreshArray = nominalThresh * ones(1,length(time));
minThreshArray = minThresh * ones(1,length(time));
maxThreshArray = maxThresh * ones(1,length(time));

% parse angle data to find steady state

logicalArrayBack = abs(angle)>minThresh & abs(angle)<maxThresh; 
    % contains logical variables with indices corresponding to each 
    % value in angle

val = 0; % logical variable

% work in reverse order.
% ASSUME system is settled at end of test. must add to test procedures
for i = length(angle):-1:1
    if val == 1
        logicalArrayBack(i) = 0;
        continue
    end
    if logicalArrayBack(i) == 0
        val = 1;
    end
end

errorArray = nominalThresh - angle;

riseIndex = find(angle >= 2,1);
timeStart = time(riseIndex);
settleIndex = find(logicalArrayBack == 1,1);
timeEnd = time(settleIndex);
settlingTime = timeEnd-timeStart;

fprintf('30° Test:\n')

if abs(errorArray(logicalArrayBack)) < threshBand & ~isnan(settlingTime)
    fprintf('After settling, the maximum error was %.2f degrees\n',max(errorArray(logicalArrayBack)))
    fprintf('This is less than the requirement, so the test is passed.\n')
elseif ~isnan(settlingTime)
    fprintf('After settling, the maximum error was %.2f degrees\n',max(errorArray(logicalArrayBack)))
    fprintf('This is greater than the requirement, so the test is failed.\n')
else
    fprintf('The system did not settle to the target angle.\n')
end

fprintf('---------------------------------------\n\n')

% plot
figure
subplot(3,1,1)
hold on
plot(time, angle); % plot angle vs. time
% plot target values and tolerance bands
if sign(angle(end)) == 1
    plot(time,nomThreshArray,'-.k')
    plot(time,minThreshArray,'-.r')
    plot(time,maxThreshArray,'-.r')
else
    plot(time,-nomThreshArray,'-.k')
    plot(time,-minThreshArray,'-.r')
    plot(time,-maxThreshArray,'-.r')
end

plot(timeStart,angle(riseIndex),'xb')
plot(timeEnd,angle(settleIndex),'xb')

% add legend
legend('Angle','Nominal Threshold', 'Settling Band','','Rise start/end','Location'...
    ,'bestoutside')
% title plot
title('Angle vs. Time for Requirement 1.3.2 - 30°')
% label axes
xlabel('Time [s]')
ylabel('Angle [°]')
hold off

%% 45 degree data
filename = "Req_1_3_2_angle_45_data.csv"; % file name is set here for convenience

% threshold variables for settling
nominalThresh = 45; % SET TARGET HERE
threshBand = 5; % +/- 5 degrees for tolerance
minThresh = nominalThresh-threshBand; % minimum value
maxThresh = nominalThresh+threshBand; % maximum value

% settling time requirement
nominalSettle = 2.5;
threshSettle = 0.5;
maxSettle = nominalSettle + threshSettle;

% open csv data
log_data = readtable(filename,"VariableNamingRule","preserve");

% sort into arrays
time = table2array(log_data(:, 1)); % get first column from table
time = time/1000; % convert from source data in ms to seconds
angle = table2array(log_data(:, 2)); % get second column from table


% arrays for plotting threshold bands
nomThreshArray = nominalThresh * ones(1,length(time));
minThreshArray = minThresh * ones(1,length(time));
maxThreshArray = maxThresh * ones(1,length(time));

% parse angle data to find steady state

logicalArrayBack = abs(angle)>minThresh & abs(angle)<maxThresh; 
    % contains logical variables with indices corresponding to each 
    % value in angle

val = 0; % logical variable

% work in reverse order.
% ASSUME system is settled at end of test. must add to test procedures
for i = length(angle):-1:1
    if val == 1
        logicalArrayBack(i) = 0;
        continue
    end
    if logicalArrayBack(i) == 0
        val = 1;
    end
end

errorArray = nominalThresh - angle;

riseIndex = find(angle >= 2,1);
timeStart = time(riseIndex);
settleIndex = find(logicalArrayBack == 1,1);
timeEnd = time(settleIndex);
settlingTime = timeEnd-timeStart;

fprintf('45° Test:\n')

if abs(errorArray(logicalArrayBack)) < threshBand & ~isnan(settlingTime)
    fprintf('After settling, the maximum error was %.2f degrees\n',max(errorArray(logicalArrayBack)))
    fprintf('This is less than the requirement, so the test is passed.\n')
elseif ~isnan(settlingTime)
    fprintf('After settling, the maximum error was %.2f degrees\n',max(errorArray(logicalArrayBack)))
    fprintf('This is greater than the requirement, so the test is failed.\n')
else
    fprintf('The system did not settle to the target angle.\n')
end

fprintf('---------------------------------------\n\n')

% plot
subplot(3,1,2)
hold on
plot(time, angle); % plot angle vs. time
% plot target values and tolerance bands
if sign(angle(end)) == 1
    plot(time,nomThreshArray,'-.k')
    plot(time,minThreshArray,'-.r')
    plot(time,maxThreshArray,'-.r')
else
    plot(time,-nomThreshArray,'-.k')
    plot(time,-minThreshArray,'-.r')
    plot(time,-maxThreshArray,'-.r')
end

plot(timeStart,angle(riseIndex),'xb')
plot(timeEnd,angle(settleIndex),'xb')



% add legend
legend('Angle','Nominal Threshold', 'Settling Band','','Rise start/end','Location'...
    ,'bestoutside')
% title plot
title('Angle vs. Time for Requirement 1.3.2 - 45°')
% label axes
xlabel('Time [s]')
ylabel('Angle [°]')
hold off


%% 60 degree data
filename = "Req_1_3_2_angle_60_data.csv"; % file name is set here for convenience

% threshold variables for settling
nominalThresh = 60; % SET TARGET HERE
threshBand = 5; % +/- 5 degrees for tolerance
minThresh = nominalThresh-threshBand; % minimum value
maxThresh = nominalThresh+threshBand; % maximum value

% settling time requirement
nominalSettle = 2.5;
threshSettle = 0.5;
maxSettle = nominalSettle + threshSettle;

% open csv data
log_data = readtable(filename,"VariableNamingRule","preserve");

% sort into arrays
time = table2array(log_data(:, 1)); % get first column from table
time = time/1000; % convert from source data in ms to seconds
angle = table2array(log_data(:, 2)); % get second column from table


% arrays for plotting threshold bands
nomThreshArray = nominalThresh * ones(1,length(time));
minThreshArray = minThresh * ones(1,length(time));
maxThreshArray = maxThresh * ones(1,length(time));

% parse angle data to find steady state

logicalArrayBack = abs(angle)>minThresh & abs(angle)<maxThresh; 
    % contains logical variables with indices corresponding to each 
    % value in angle

val = 0; % logical variable

% work in reverse order.
% ASSUME system is settled at end of test. must add to test procedures
for i = length(angle):-1:1
    if val == 1
        logicalArrayBack(i) = 0;
        continue
    end
    if logicalArrayBack(i) == 0
        val = 1;
    end
end

errorArray = nominalThresh - angle;

riseIndex = find(angle >= 2,1);
timeStart = time(riseIndex);
settleIndex = find(logicalArrayBack == 1,1);
timeEnd = time(settleIndex);
settlingTime = timeEnd-timeStart;

finalTime = time(end);
settledTime = finalTime-timeEnd;

fprintf('60° Test:\n')

if abs(errorArray(logicalArrayBack)) < threshBand & ~isnan(settlingTime)
    fprintf('After settling, the maximum error was %.2f degrees\n',max(errorArray(logicalArrayBack)))
    fprintf('This is less than the requirement, so the test is passed.\n')
elseif ~isnan(settlingTime)
    fprintf('After settling, the maximum error was %.2f degrees\n',max(errorArray(logicalArrayBack)))
    fprintf('This is greater than the requirement, so the test is failed.\n')
else
    fprintf('The system did not settle to the target angle.\n')
end

fprintf('---------------------------------------\n\n')

% plot
subplot(3,1,3)
hold on
plot(time, angle); % plot angle vs. time
% plot target values and tolerance bands
if sign(angle(end)) == 1
    plot(time,nomThreshArray,'-.k')
    plot(time,minThreshArray,'-.r')
    plot(time,maxThreshArray,'-.r')
else
    plot(time,-nomThreshArray,'-.k')
    plot(time,-minThreshArray,'-.r')
    plot(time,-maxThreshArray,'-.r')
end

plot(timeStart,angle(riseIndex),'xb')
plot(timeEnd,angle(settleIndex),'xb')

% add legend
legend('Angle','Nominal Threshold', 'Settling Band','','Rise start/end','Location'...
    ,'bestoutside')
% title plot
title('Angle vs. Time for Requirement 1.3.2 - 60°')
% label axes
xlabel('Time [s]')
ylabel('Angle [°]')
hold off









