close all
clear
clc

%% Load the Library
if ~libisloaded('MEGSV86x64') 
    loadlibrary('.\\MEGSV86x64.dll', '.\\MEGSV86x64.h');
end

%% COM Port Configuration
com = int32(5);  

%% Activate and configure device
[extendet] = calllib('MEGSV86x64', 'GSV86actExt', com);
freq = 100;
calllib('MEGSV86x64', 'GSV86setFrequency', com, freq);
calllib('MEGSV86x64', 'GSV86setZero', com, int32(0));
calllib('MEGSV86x64', 'GSV86clearDLLbuffer', com);

%% Setup UDP Sender
u = udpport("datagram", "IPV4");
targetIP = "127.0.0.1";
targetPort = 25000;

%% Allocate Buffer for GSV86readMultiple and for error Msg
count = int32(48000);
buffer = zeros(1, double(count));
out = libpointer('doublePtr', buffer);
valsread = libpointer('int32Ptr', 0);
errFlags = libpointer('int32Ptr', 0);
errMsg = blanks(256);
errTextPtr = libpointer('cstring', errMsg);

%% Initialize Data Storage
A = [];
T = [];

% History vectors
data1_history = [];
data2_history = [];
data3_history = [];
data4_history = [];
data5_history = [];
data6_history = [];
time_history = [];

%% Setup Real-Time Plot
figure;
ax = axes;
hold on;
grid on;
title('Real-Time Force and Torque');
xlabel('Time (s)');
ylabel('Force / Moment (Units)');

h1 = animatedline('Color', 'b', 'DisplayName', 'Fx');
h2 = animatedline('Color', 'r', 'DisplayName', 'Fy');
h3 = animatedline('Color', 'g', 'DisplayName', 'Fz');
h4 = animatedline('Color', 'c', 'DisplayName', 'Mx');
h5 = animatedline('Color', 'm', 'DisplayName', 'My');
h6 = animatedline('Color', 'k', 'DisplayName', 'Mz');
legend show;

tic;
AcquisitionTime = 10; 
desiredSamples = AcquisitionTime * freq;

%% Main Loop
while true
    pause(0.01)
    t = toc;  
    [err, ~] = calllib('MEGSV86x64', 'GSV86readMultiple', com, int32(0), out, count, valsread, errFlags);

    if err == 1 && valsread.Value > 0
        validData = out.Value(1:valsread.Value);
        reshaped = reshape(validData, 8, []).';

        timestamps = repmat(t, size(reshaped, 1), 1);
        A = [A; reshaped];
        T = [T; timestamps];

        latestSample = reshaped(end, :);

        data1 = single(latestSample(1));
        data2 = single(latestSample(2));
        data3 = single(latestSample(3));
        data4 = single(latestSample(4));
        data5 = single(latestSample(5));
        data6 = single(latestSample(6));

        data1_history = [data1_history; data1];
        data2_history = [data2_history; data2];
        data3_history = [data3_history; data3];
        data4_history = [data4_history; data4];
        data5_history = [data5_history; data5];
        data6_history = [data6_history; data6];
        time_history = [time_history; t];

        addpoints(h1, t, data1);
        addpoints(h2, t, data2);
        addpoints(h3, t, data3);
        addpoints(h4, t, data4);
        addpoints(h5, t, data5);
        addpoints(h6, t, data6);
        drawnow 

        packet = [data1, data2, data3, data4, data5, data6];
        bytes = typecast(packet, 'uint8');
        write(u, bytes, targetIP, targetPort);

    elseif err == 0
        disp("No new data available.");

    elseif err == -1
        calllib('MEGSV86x64', 'GSV86getLastErrorText', com, errTextPtr);
        disp([' DLL Error: ', errTextPtr.Value]);

    end
end

%% After stop
calllib('MEGSV86x64', 'GSV86release', com);
unloadlibrary('MEGSV86x64');
