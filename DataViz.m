%% Start Script
clc; clear; close all;


%% Import Data

load Data1.mat %this should have once column 9 columns for each of the 5 sensors
[rows1,columns] = size(Data1);
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


%% Initial Variables
sampleRate = .5;
Beta = .5;

% link lengths (cm)
l43 = 8;
l74 = 10;
l67= 10;
l57 = l67;

% initial sensor orientations
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
euler7 = ExScriptFun(sen7,sampleRate,Beta);
euler6 = ExScriptFun(sen6,sampleRate,Beta);
euler5 = ExScriptFun(sen5,sampleRate,Beta);
euler4 = ExScriptFun(sen4,sampleRate,Beta);
euler3 = ExScriptFun(sen3,sampleRate,Beta);

for ii = 1:rows1
    
    Re7 = eulerT(euler7(ii,3),euler7(ii,2),euler7(ii,1));
    Re6 = eulerT(euler6(ii,3),euler6(ii,2),euler6(ii,1));
    Re5 = eulerT(euler5(ii,3),euler5(ii,2),euler5(ii,1));
    Re4 = eulerT(euler4(ii,3),euler4(ii,2),euler4(ii,1));
    Re3 = eulerT(euler3(ii,3),euler3(ii,2),euler3(ii,1));
    
    rpe7(ii,:) = Re7*[0;1;0];
    rpe6(ii,:) = Re6*[0;1;0];
    rpe5(ii,:) = Re5*[0;1;0];
    rpe4(ii,:) = Re4*[0;1;0];
    rpe3(ii,:) = Re3*[0;1;0];
end

pe0 = zeros(rows1,3);
pe3 = rpe3 + pe0;
pe4 = rpe4 + pe3;
pe7 = rpe7 + pe4;
pe5 = rpe5 + pe7;
pe6 = rpe6 + pe7;





%% Plot

f1 = figure;
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
set(p2,'MarkerSize',3,'LineWidth',.25,'Color',[.4,.7,1])
legend('marshall','sensor 7')

pause(.6)
for i = 1:rows-1
    pos = [pe3(i,:);pe4(i,:);pe7(i,:);pe6(i,:);pe7(i,:);pe5(i,:)];
    set(p1,'xData',pos(:,1),'yData',pos(:,3),'zData',pos(:,2))
    set(p2,'xData',pe7(1:i,1),'yData',pe7(1:i,3),'zData',pe7(1:i,2))
    
    pause(.025)
end
