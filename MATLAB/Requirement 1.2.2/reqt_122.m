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
nominalThresh = -4; % 4 psig
threshBand = 1; % +/- 1 psig tolerance
maxThresh = nominalThresh-threshBand; % maximum decreasing rate

% open csv data
log_data = readtable(filename,"VariableNamingRule","preserve");

% sort into arrays
time = table2array(log_data(:, 1)); % get first column from table
time = time/1000; % convert from source data in ms to seconds
% get pressures. assume columns 2 and 3
pressureTop = table2array(log_data(:,2));
pressureBot = table2array(log_data(:,3));

% ignore first 4 data points, aka first 20 seconds

time([1,2,3,4]) = [];
timeMin = time/60;

pressureTop([1,2,3,4]) = [];
pressureBot([1,2,3,4]) = [];

linFitTop = polyfit(timeMin,pressureTop,1);
linFitBot = polyfit(timeMin,pressureBot,1);

slopeTop = linFitTop(1);
slopeBot = linFitBot(1);

if slopeTop > maxThresh
    fprintf('Top pressure satisfies requirement.\n')
    fprintf('Top PT changed at a rate of %.2f psig/min\n',slopeTop)
    if slopeBot > maxThresh
        fprintf('Bottom pressure satisfied requirement.\n')
        fprintf('Bottom PT changed at a rate of %.2f psig/min\n',slopeBot)
        fprintf('Test is passed.\n')
    else
        fprintf('Bottom pressure failed requirement.\n')
        fprintf('Bottom PT changed at a rate of %.2f psig/min\n',slopeBot)
    end
else
    fprintf('Top pressure failed requirement.\n')
    fprintf('Top PT changed at a rate of %.2f psig/min\n',slopeTop)
end

figure
subplot(2,1,1)
hold on
plot(timeMin,pressureTop)

plot(timeMin,polyval(linFitTop,timeMin),'-.k')

plot(timeMin,polyval([maxThresh linFitTop(2)],timeMin),'-.r')

% add legend
legend('Pressure','Regression', 'Threshold','Location','bestoutside')
% title plot
title('Top pressure drop vs. time for Requirement 1.2.2')
% label axes
xlabel('Time [min]')
ylabel('Top Pressure [psig]')
hold off


subplot(2,1,2)
hold on
plot(timeMin, pressureBot)
plot(timeMin,polyval(linFitBot,timeMin),'-.k')
plot(timeMin,polyval([maxThresh linFitBot(2)],timeMin),'-.r')
% add legend
legend('Pressure','Regression', 'Threshold','Location','bestoutside')
% title plot
title('Bottom pressure drop vs. time for Requirement 1.2.2')
% label axes
xlabel('Time [min]')
ylabel('Top Pressure [psig]')










