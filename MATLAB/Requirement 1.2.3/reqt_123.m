% MAE 491-01 Team 03 Requirement 1.2.3 Data MATLAB Parser

% Goal: Parse time and pressure data

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
pressureTop = table2array(log_data(:,4));
pressureBot = table2array(log_data(:,5));

pressureTop = lowpass(pressureTop,0.5,'Steepness',0.95);
pressureBot = lowpass(pressureBot,0.5,'Steepness',0.95);

% ignore first 4 data points, aka first 20 seconds


figure
subplot(2,1,1)
hold on
plot(time,pressureTop)
% add legend
%legend('Pressure','Regression', 'Threshold','Location','bestoutside')
% title plot
title('Top pressure vs. time for Requirement 1.2.3')
% label axes
xlabel('Time [s]')
ylabel('Top Pressure [psig]')
hold off


subplot(2,1,2)
hold on
plot(time,pressureBot)
% add legend
%legend('Pressure','Regression', 'Threshold','Location','bestoutside')
% title plot
title('Bottom pressure vs. time for Requirement 1.2.3')
% label axes
xlabel('Time [s]')
ylabel('Top Pressure [psig]')










