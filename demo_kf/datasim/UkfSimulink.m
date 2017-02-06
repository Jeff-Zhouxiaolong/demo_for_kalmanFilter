close all
clear all
t = 0;
T = 0.02;
t_stop = 164.0;

%%航迹发生器
atti = zeros(3,1);         %滚转、俯仰、偏航（单位：度）
atti_rate = zeros(3,1);    %滚转角速率、俯仰角速率、偏航角速率（单位：度/秒）
veloB = zeros(3,1);        %飞机运动速度--X右翼、Y机头、Z天向（单位：米/秒）
acceB = zeros(3,1);        %飞机运动加速度--X右翼、Y机头、Z天向（单位：米/秒/秒）
posi = zeros(3,1);         %航迹发生器初始位置经度、纬度、高度（单位：度、度、米）
posi = [26;70;-19];

atti(1,1) = 0;            %
atti(2,1) = 0;            %
atti(3,1) = 90;            %初始航向角（单位：度）

%%IMU输出
Wibb = zeros(3,1);         %机体系陀螺仪输出（单位：度/秒）
Fb = zeros(3,1);           %机体系加速度计输出（单位：米/秒/秒）
Gyro_fix = zeros(3,1);      %机体系陀螺仪固定误差输出（单位：弧度/秒）
Acc_fix = zeros(3,1);       %机体系加速度计固定误差输出（单位：米/秒/秒）
Gyro_b = zeros(3,1);       %陀螺随机常数（弧度/秒）
Gyro_r = zeros(3,1);       %陀螺一阶马尔可夫过程（弧度/秒）
Gyro_wg = zeros(3,1);      %陀螺白噪声（弧度/秒）
Acc_r = zeros(3,1);        %加速度一阶马尔可夫过程（米/秒）

%%UWB仿真输出
posiG = zeros(3,1);        %UWB输出的飞行器位置测距值

%%捷联惯导仿真
attiN = zeros(3,1);        %飞行器初始姿态
veloN = zeros(3,1);        %飞行器初始速度（相对于导航系）
posiN = zeros(3,1);        %飞行器初始位置
WnbbA_old = zeros(3,1);    %角速度积分输出（单位：弧度）

posiN = posi;
attiN = atti;

%%KALMAN滤波输出
T_D = 1;                   %离散周期
T_M = 0;                   %滤波量测产生时间（秒）
Xc = zeros(18,1);              %综合模型状态量
PK = zeros(18,18);               %协方差阵
Xerr = zeros(1,18);             %状态估计量的误差
kflag = 0;                 %GPS信号有效标志位（1-有效）

Acc_modi = zeros(3,1);     %加速度计误差修正值
Gyro_modi = zeros(3,1);     %陀螺仪误差修正值

%%初始对准
kc = 0;
tmp_Fb = zeros(3,1);
tmp_Wibb = zeros(3,1);
t_alig = 0;

old_veloB = veloB;
old_atti = atti;

deg_rad = pi/180;

TraceData = [];
IMUData =[];
UWBData = [];
kc = 0;bS = 1;
while t<=t_stop
	old_atti = atti;
    old_veloB = veloB;
    
    [t,atti,atti_rate,veloB,acceB] = trace(t,T,atti,atti_rate,veloB,acceB);
    
    [Wibb,Fb,posi,veloN,acceN] = IMUout(T,posi,atti,atti_rate,veloB,acceB,old_veloB,old_atti);

	if kc == 4
       [Ranging] = UwbOut(bS,posi);
	   Uwbranging_vector = Ranging';stationnumber_vector=bS;UWBBroadTime_vector=t;
	    UWBData = [UWBData;[Uwbranging_vector stationnumber_vector UWBBroadTime_vector]];
		kc = 0;
% 		bS = mod(bS ,5) + 1;
		switch bS
			case 1 
				bS = 3;
			case 2
				bS = 4;
		    case 3
				bS = 5;
			case 4
				bS = 1;
			case 5
				bS = 2;
		end
	end
    [Gyro_b,Gyro_r,Gyro_wg,Acc_r] = imu_err_random(t,T,Gyro_b,Gyro_r,Gyro_wg,Acc_r);
    
	%%%%%加入噪声
    Fb = Fb+Acc_r;
    Wibb = Wibb+Gyro_b/deg_rad+Gyro_r/deg_rad+Gyro_wg/deg_rad;   %%%deg/s
	
	    %%保存数据
    TraceData = [TraceData;[t,atti',veloN',posi',Gyro_r',Acc_r',acceN']];
    
	a_vector =  Fb';g_vector=Wibb';  Temperature = 36.1;SampleTimePoint=t;
    IMUData = [IMUData;[a_vector,g_vector,Temperature,SampleTimePoint]];
   
   kc = kc + 1;
    t = t+T;
end
bSPcs = 5;
Uwbranging_vector = UWBData(:,1:bSPcs);
stationnumber_vector = UWBData(:,bSPcs+1);
UWBBroadTime_vector = UWBData(:,bSPcs+2);
save uwb_HandledFileToMatData.mat Uwbranging_vector stationnumber_vector UWBBroadTime_vector

a_vector =  IMUData(:,1:3);
g_vector =  IMUData(:,4:6);
Temperature =  IMUData(:,7);
SampleTimePoint =  IMUData(:,8);
save imu_HandledFileToMatData.mat a_vector g_vector  Temperature SampleTimePoint

hold on
plot(TraceData(:,8),TraceData(:,9),'r','DisplayName','Trace')
% axis equal;
plot(TraceData(:,1),TraceData(:,10),'r','DisplayName','Speed')

grid on
save TraceData.mat TraceData

