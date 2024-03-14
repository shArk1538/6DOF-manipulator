clear,clc,close all;
%% 建立机器人DH参数,初始姿态为竖直状态
%连杆偏移d，连杆长度a，连杆扭转角alpha
L1=RevoluteMDH('d',121.5,'a',0,'alpha',0); 
L2=RevoluteMDH('d',0,'a',0,'alpha',pi/2,'offset',-pi/2);
L3=RevoluteMDH('d',0,'a',-300,'alpha',0);
L4=RevoluteMDH('d',110.5,'a',-276,'alpha',0,'offset',-pi/2);
L5=RevoluteMDH('d',90,'a',0,'alpha',pi/2);
L6=RevoluteMDH('d',82,'a',0,'alpha',-pi/2);  
ROCR6II = SerialLink([L1 L2 L3 L4 L5 L6],'name','Arm6');
%% 正解，给定关节角，求末端位姿（关节角可修改）
Theta = [0 0 0 0 0 0];  %给定6个关节角度值,或可表示为Theta = zeros(1,6);
% Theta=[30 30 30 30 30 30];  %Theta = 30*ones(1,6);
%% 验证正逆解的结果
Theta=Theta/180*pi;  %换算成弧度
T = ROCR6II.fkine(Theta)  %求正解的齐次变换矩阵
W = [-1000,+1000,-1000,+1000,-1000,+1000];  %限制坐标轴范围
view(3)
ROCR6II.plot(Theta,'tilesize',150,'workspace',W);  %显示三维动画
q1 = ROCR6II.ikine(T)*180/pi;  %求逆解验证关节角
rpy = tr2rpy(T,'xyz')*180/pi  %求末端姿态，工具法兰为绕XYZ轴旋转
EUL = tr2eul(T,'deg')  %旋转矩阵反解出y-z-y欧拉角
[THETA,V] = tr2angvec(T,'deg')  %旋转矩阵反解出等效轴角坐标表示
ROCR6II.teach(T,'rpy')  %显示各个参数 R:roll翻滚角，P：pitch俯仰角，Y：yaw侧航角 【GUI界面可调】
Qua = UnitQuaternion(T)  %求解四元数