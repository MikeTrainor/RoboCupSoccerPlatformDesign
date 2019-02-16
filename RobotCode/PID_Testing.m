%% Embedded System PID Testing


%% Get the Data
clear
clc

desvel = 150;

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

s1 = serial('COM5', 'BaudRate', 9600);
fopen(s1);

% Collecting the data
for i = 1:5
    data{i} = fgets(s1);
    e{i}   = sscanf(data{i}, '%2x');
end
e{1}(251:256) = e{1}(250); %Make the cell 256 long

% Plotting the error
figure()
x = 1:256;
for i = 1:5
    hold on
    plot(x,e{i}, 'b'); %Plot the error
    x(:) = x(:) + 256
end

data_mean = cellfun(@mean,e); %Find the mean of the data
data_mean = mean(data_mean);
plot([0,256*5],[desvel,desvel], 'r');
plot([0,256*5],[data_mean,data_mean],'g');

title('Ramp Response Kp = 8, Ki = -2, Kd = 2')
xlabel('Samples')
ylabel('Velocity (RPM)')

% Invisible plots to record the colors
L(1) = plot(nan,nan,'b');
L(2) = plot(nan, nan, 'g');
L(3) = plot(nan, nan, 'r');
legend(L, {'Measured Velocity (RPM)', 'Measured Velocity Mean (RPM)', 'Desired Velocity (RPM)'})
hold off



