clc; clear; close all;

%% Things left to do

% adjust for drift

%% Import Data
load gyroData.mat;
[rows,cols] = size(gyr); % use this if you want all the data

% 244:343 picks the toe touch segment of the data
r_max = 343;
r_min = 244;
rows = r_max-r_min;

%% Initial Variables

dt = 0.5;  % sample rate used
w = 0.5;   % weight factor

% link lengths (cm)
l43 = 8;
l74 = 10;
l67= 10;
l57 = l67;

% initialize variables
smooth = zeros(r_max - r_min + 1, 15);
angles = zeros(rows + 1, 15);
rotations = zeros(rows + 1, 15);


offsets = [0,0,  0;    %from frame 4 to 7
          -90,0, -30;   %from frame 7 to 6
         90,0,-150;   %from frame 7 to 5
           0,0,   0]'; %from frame 3 to 4

% initial unit vectors
origins = [0,1,0;       %from 4 to 7
           -.83,0,-.27;   %initial vecotr in frame 6
           -.83,0,-.27;   %initial vector in frame 5
           0,1,0]';     %from 3 to 4

%% Calculations

smooth(1, :) = gyr(1, 3:end);

for  j = 1:15
    for   i = 2:rows
        % smooth data
        smooth(i,j) = w * gyr(r_min-1+i,j+2) + (1-w)*smooth(i-1,j);
        % integrate to get angles
        angles(i,j) = angles(i-1,j) + (smooth(i,j) + smooth(i-1,j))/2*dt;
        
    end
end


% linear drift correction
m = -.5;
m2 = .5;


% do transformations and get rotations
for j = 1:4 %don?t go to 5 because we don't have to do anything to o3
    for i = 1:rows
        psi =  angles(i,3*j-2);
        theta = angles(i,3*j-1);
        phi = angles(i,3*j);
        
        psi0 = offsets(1,j);
        theta0 = offsets(2,j);
        phi0 = offsets(3,j);
        R = transformation(psi,theta,phi,psi0,theta0,phi0);
        rotations(i,3*j-2:3*j) = R*origins(:,j);
    end
end

% unit positions are summations of rotations
positions3 = zeros(rows + 1, 3);
positions4 = rotations(:,10:12);
positions7 = rotations(:,1:3)+positions4;
positions6 = rotations(:,4:6)+positions7;
positions5 = rotations(:,7:9)+positions7;

% % scale positions with link lengths
% positions4 = l43*positions4;
% positions7 = l74*positions7;
% positions6 = l67*positions6;
% positions5 = l57*positions5;

%% Plot

% f1 = figure;
% plot(1:rows+1,angles(:,10),1:rows+1,angles(:,11),1:rows+1,-angles(:,12))
% legend 1 2 3
% hold on
% %plot(1:rows+1,smooth(:,12))
% hold off

% f2 = figure;
% plot(1:rows+1,positions5(:,:))
% legend x y z


f3 = figure;
pos = [positions3(1,:);positions4(1,:);positions7(1,:);positions6(1,:);positions7(1,:);positions5(1,:)];
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

p2 = plot3(positions5(1,1),positions5(1,3),positions5(1,2),'.-');
set(p2,'MarkerSize',3,'LineWidth',.25,'Color',[.5,1,.4])
p3 = plot3(positions6(1,1),positions6(1,3),positions6(1,2),'.-');
set(p3,'MarkerSize',3,'LineWidth',.25,'Color',[.4,.7,1])
legend('marshall','sensor 5','sensor 6')

pause(.6)
for i = 1:rows-1
    pos = [positions3(i,:);positions4(i,:);positions7(i,:);positions6(i,:);positions7(i,:);positions5(i,:)];
    set(p1,'xData',pos(:,1),'yData',pos(:,3),'zData',pos(:,2))
    set(p2,'xData',positions5(1:i,1),'yData',positions5(1:i,3),'zData',positions5(1:i,2))
    set(p3,'xData',positions6(1:i,1),'yData',positions6(1:i,3),'zData',positions6(1:i,2))
    pause(.05)
end
