% MAE 491-01 Team 03 Requirement 1.2.3 Data MATLAB Parser

% Goal: Parse time and pressure data

% housekeeping
clear
clc
close all
format compact 

filename = "Req_1_2_3_data.csv"; % file name is set here for convenience

% threshold variables
nominalThresh = -4; % 4 psig
threshBand = 1; % +/- 1 psig tolerance
maxThresh = nominalThresh-threshBand; % maximum decreasing rate

pressureThresh = 10;

angle = 45;

% open csv data
log_data = readtable(filename,"VariableNamingRule","preserve");

% sort into arrays
time = table2array(log_data(:, 1)); % get first column from table
time = time/1000; % convert from source data in ms to seconds
% get pressures. assume columns 2 and 3
pressureTop = table2array(log_data(:,3));
pressureBot = table2array(log_data(:,4));

pressureTop = lowpass(pressureTop,0.35,'Steepness',0.95);
pressureBot = lowpass(pressureBot,0.35,'Steepness',0.95);

% ignore first 4 data points, aka first 20 seconds




figure
if sign(angle) == 0

    lastBeforeClose = find(pressureTop > 22,1,"last");
    closeTime = time(lastBeforeClose);
    closePressure = pressureTop(lastBeforeClose);

    finalTime = closeTime + 10;
    finalTimeIndex = find(time > finalTime,1);
    finalPressure = pressureTop(finalTimeIndex);

    pressureChange = closePressure-finalPressure;

    if pressureChange > pressureThresh
        fprintf('The pressure dropped by %.2f psig over 10 seconds.\n',pressureChange)
        fprintf('This is greater than the requirement, so the test is passed.\n')
    else
        fprintf('The pressure dropped by %.2f psig over 10 seconds.\n',pressureChange)
        fprintf('This is less than the requirement, so the test is failed.\n')
    end


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
else 
    lastBeforeClose = find(pressureBot > 22,1,"last");
    closeTime = time(lastBeforeClose);
    closePressure = pressureBot(lastBeforeClose);

    finalTime = closeTime + 8;
    finalTimeIndex = find(time > finalTime,1);
    finalPressure = pressureBot(finalTimeIndex);

    pressureChange = closePressure-finalPressure;

    if pressureChange > pressureThresh
        fprintf('The pressure dropped by %.2f psig over 10 seconds.\n',pressureChange)
        fprintf('This is greater than the requirement, so the test is passed.\n')
    else
        fprintf('The pressure dropped by %.2f psig over 10 seconds.\n',pressureChange)
        fprintf('This is less than the requirement, so the test is failed.\n')
    end

    hold on
    plot(time,pressureBot)
    % add legend
    %legend('Pressure','Regression', 'Threshold','Location','bestoutside')
    % title plot
    title('Bottom pressure vs. time for Requirement 1.2.3')
    % label axes
    xlabel('Time [s]')
    ylabel('Top Pressure [psig]')
end









