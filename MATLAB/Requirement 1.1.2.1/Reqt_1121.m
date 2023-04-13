clear
clc
close all


array = cellfun(@str2num,inputdlg({'Enter Y Coordinate:','Enter Z Coordinate'},'Input Coordinates'));

y = array(1);
z = array(2);

mag = sqrt(y^2+z^2);

if mag < 0.375
    fprintf('The center of gravity lies %.2f inches radially from the axis of rotation.\n',mag)
    fprintf('This is less than the maximum, so the test is passed.\n')
else
    fprintf('The center of gravity lies %.2f inches radially from the axis of rotation.\n',mag)
    fprintf('This is greater than the maximum, so the test is failed.\n')
end