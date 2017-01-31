%% Start Script
clc; clear; close all;


%% Import Data

load quicktest.mat %this should have once column 9 columns for each of the 5 sensors
Data1 = quicktest;
[rows1,columns] = size(Data1);

% assign variable names
time = Data1(:,1);

acc7 = Data1(:,2:4);
acc6 = Data1(:,11:13);
acc5 = Data1(:,20:22);
acc4 = Data1(:,29:31);
acc3 = Data1(:,38:40);

gyr7 = Data1(:,5:7);
gyr6 = Data1(:,14:16);
gyr5 = Data1(:,23:25);
gyr4 = Data1(:,32:34);
gyr3 = Data1(:,41:43);

mag7 = Data1(:,8:10);
mag6 = Data1(:,17:19);
mag5 = Data1(:,26:28);
mag4 = Data1(:,35:37);
mag3 = Data1(:,44:46);

sen7 = [time,acc7,gyr7,mag7];
sen6 = [time,acc6,gyr6,mag6];
sen5 = [time,acc5,gyr5,mag5];
sen4 = [time,acc4,gyr4,mag4];
sen3 = [time,acc3,gyr3,mag3];

sens = [sen3,sen4,sen5,sen6,sen7];





%% Initial Parameters

% algorithm parameters
sampleRate = .05;
Beta = .01;

% link lengths (cm)
l43 = 8;
l74 = 10;
l67= 10;
l57 = l67;

% initial sensor orientations
offsets = [ 0, 0,   0;   %from frame 4 to 7
          -90, 0, -30;   %from frame 7 to 6
           90, 0,-150;   %from frame 7 to 5
            0, 0,   0]'; %from frame 3 to 4

% initial unit vectors
origins = [ 0, 1,   0;   %from 4 to 7
         -.83, 0,-.27;   %initial vecotr in frame 6
         -.83, 0,-.27;   %initial vector in frame 5
            0, 1,   0]'; %from 3 to 4


       
       
 
       
%% Calculations

% run algorithm for each sensor
% input is raw data, output is euler angles for each step
euler7 = ExScriptFun(sen7,sampleRate,Beta);
euler6 = ExScriptFun(sen6,sampleRate,Beta);
euler5 = ExScriptFun(sen5,sampleRate,Beta);
euler4 = ExScriptFun(sen4,sampleRate,Beta);
euler3 = ExScriptFun(sen3,sampleRate,Beta);

for ii = 1:rows1 % for every time step
    
    % Calculate rotation matrix based on euler angles
    % Re = rotation matix euler
    Re7 = eulerT(euler7(ii,3),euler7(ii,2),euler7(ii,1));
    Re6 = eulerT(euler6(ii,3),euler6(ii,2),euler6(ii,1));
    Re5 = eulerT(euler5(ii,3),euler5(ii,2),euler5(ii,1));
    Re4 = eulerT(euler4(ii,3),euler4(ii,2),euler4(ii,1));
    Re3 = eulerT(euler3(ii,3),euler3(ii,2),euler3(ii,1));
    
    % Caculate relative position of each sensor from initial position
    % rpe = relative position euler
    rpe7(ii,:) = Re7*[0;1;0]; 
    rpe6(ii,:) = Re6*[0;1;0];
    rpe5(ii,:) = Re5*[0;1;0];
    rpe4(ii,:) = Re4*[0;1;0];
    rpe3(ii,:) = Re3*[0;1;0];
end

% Calculate absolute positions based on relative ones
% pe = (absolute) position euler
pe0 = zeros(rows1,3); 
pe3 = pe0;%rpe3 + pe0;
pe4 = rpe4 + pe3;
pe7 = rpe7 + pe4;
pe5 = rpe5 + pe7;
pe6 = rpe6 + pe7;





%% Plot

% plot euler angles for one sensor
f0 = figure('Name','Calculated Angles');
plot(time, euler7(:,1), 'r');
hold on
plot(time, euler7(:,2), 'k');
plot(time, euler7(:,3), 'b');
title('Euler angles 7');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi X', '\theta Y', '\psi Z');
hold off

f2 = figure;
plot(time, euler4(:,1), 'r');
hold on
plot(time, euler4(:,2), 'k');
plot(time, euler4(:,3), 'b');
title('Euler angles 4');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi X', '\theta Y', '\psi Z');
hold off

% plot motion replay with p1 = stick figure
% other plots = trace lines
f1 = figure('Name', 'Motion Replay');
pos = [pe3(1,:);pe4(1,:);pe7(1,:);pe6(1,:);pe7(1,:);pe5(1,:)];
p1 = plot3(pos(:,1),pos(:,3),pos(:,2),'r.-');
set(p1,'LineWidth',1.5,'MarkerSize',8)
xlim([-3,3]);
ylim([-3,3]);
zlim([-3,3]);
xlabel x
ylabel z
zlabel y
%view(90,0);
grid on
hold on

p2 = plot3(pe7(1,1),pe7(1,3),pe7(1,2),'.-');
set(p2,'MarkerSize',3,'LineWidth',.25,'Color',[.6,.1,.2])
legend('marshall','sensor 7')

p3 = plot3(pe4(1,1),pe4(1,3),pe4(1,2),'.-');
set(p3,'MarkerSize',3,'LineWidth',.25,'Color',[.4,.7,1])
legend('marshall','sensor 7')

% Update replay plot with each step of data
pause(.7)
for i = 1:rows1-1
    pos = [pe3(i,:);pe4(i,:);pe7(i,:);pe6(i,:);pe7(i,:);pe5(i,:)];
    set(p1,'xData',pos(:,1),'yData',pos(:,3),'zData',pos(:,2))
    set(p2,'xData',pe7(1:i,1),'yData',pe7(1:i,3),'zData',pe7(1:i,2))
    set(p3,'xData',pe4(1:i,1),'yData',pe4(1:i,3),'zData',pe4(1:i,2))
    pause(.01)
end





% plot raw sensor data for one sensor
figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, gyr7(:,1), 'r');
plot(time, gyr7(:,2), 'k');
plot(time, gyr7(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, acc7(:,1), 'r');
plot(time, acc7(:,2), 'k');
plot(time, acc7(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, mag7(:,1), 'r');
plot(time, mag7(:,2), 'k');
plot(time, mag7(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');


%% End of Script
