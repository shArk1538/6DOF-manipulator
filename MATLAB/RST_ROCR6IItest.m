clear,clc;close all;
ROCR6II=importrobot('ROCR6II_URDF_file.urdf'); %导入UDRF文件
showdetails(ROCR6II)  %显示连杆间的父子关系
show(ROCR6II,'Frames','off','Visuals' ,'on') %figure显示
%% 验证正解30*ones(1,6)
% Theta = 30*ones(1,6);
Theta = zeros(1,6);
Theta=Theta/180*pi;  %换算成弧度
initialguess = homeConfiguration(ROCR6II);
for i=1:6
    initialguess(i).JointPosition = Theta(i);
end
tform = getTransform(ROCR6II,initialguess,'tool0','world')
figure('Name','正解姿态显示')
show(ROCR6II,initialguess,'Frames','off','Visuals' ,'on');
axis([-0.6,0.6,-0.6,0.6,0,1]);
%% 创建ROCR6II模型的ik对象
ik = robotics.InverseKinematics('RigidBodyTree',ROCR6II);
%指定姿势不同分量的权重，关于权重的分配，估计和具体算法相关
%对于方向角分量来说，使用比位置分量小的权重
weights = [0.25 0.25 0.25 1 1 1];
%将机器人的home构型用作初始猜测的关节角
initialguess = ROCR6II.homeConfiguration;

%根据预期的末端执行器位姿来逆解得到关节角度
[configSoln,solninfo] = ik('tool0',tform,weights,initialguess);
rad2deg = 180/pi*ones(1,6);
%(configSoln.JointPosition)*rad2deg
%显示逆解得到的关节构型
%逆解得到的关节构型与目标构型有着细微的差别（误差）
%多次调用ik对象可以提供相似或非常不同的关节构型
figure('Name','逆解姿态显示')
show(ROCR6II,configSoln,'Frames','off','Visuals' ,'on')
axis([-0.6,0.6,-0.6,0.6,0,1]);