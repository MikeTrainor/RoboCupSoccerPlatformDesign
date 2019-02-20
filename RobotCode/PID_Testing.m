%% Embedded System PID Testing
% This function reads a selected COM port and plots the received data
% Author: Mike Trainor
% Date: February 19th, 2019
% Revision: 2

%% Get the Data
clear
clc

desvel = 150; %Desired Velocity
n = 5; %Length of sample

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

s1 = serial('COM5', 'BaudRate', 9600); %Specify the required COM port
fopen(s1);

% Collecting the data
for i = 1:n
    port_read{i} = fgets(s1);
    data{i}   = sscanf(port_read{i}, '%2x'); %Decode number stream every two numbers
end

% Correcting the size of the first cell of data samples 
if(length(data{1}) ~= length(data{2}))
    data{1}(end:length(data{2})) = data{1}(end); %Make the cell length the same as the others, not great as it just duplicates the last value
end

% Plotting the data
figure()
x = 1:length(data{1});
for i = 1:n
    hold on
    plot(x,data{i}, 'b'); %Plot the error
    x(:) = x(:) + length(data{1}); %Go to next cell
end

data_mean = cellfun(@mean,data); %Find the mean of the data
data_mean = mean(data_mean);
plot([0,length(data{1})*5],[desvel,desvel], 'r');
plot([0,length(data{1})*5],[data_mean,data_mean],'g');

title('Ramp Response Kp = 8, Ki = -2, Kd = 2')
xlabel('Samples')
ylabel('Velocity (RPM)')

% Invisible plots to record the colors
L(1) = plot(nan,nan,'b');
L(2) = plot(nan, nan, 'g');
L(3) = plot(nan, nan, 'r');
legend(L, {'Measured Velocity (RPM)', 'Measured Velocity Mean (RPM)', 'Desired Velocity (RPM)'})
hold off