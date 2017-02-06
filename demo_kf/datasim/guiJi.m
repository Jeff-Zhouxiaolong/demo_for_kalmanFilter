load TraceSim.mat
via = [via;via(1,:)];
Point = 150;
Tf = 1/50;
plot3(via(:,1),via(:,2),via(:,3),'r-o');grid on;
xlabel('x方向');ylabel('y方向');zlabel('z方向');
figure 
plot(via(:,1),via(:,2),'r-o');grid on;

theta = zeros(Pcs,3);
%%%%%%1.轨迹1
theta(1:21,:) = repmat(([0,0,0] + 5*randn(1,3))/180*pi,21,1);
theta(22:25,:) = repmat(([0,0,45] + 5*randn(1,3))/180*pi,4,1);
theta(26:33,:) = repmat(([0,0,90] + 5*randn(1,3))/180*pi,8,1);
theta(34:37,:) = repmat(([0,0,135] + 5*randn(1,3))/180*pi,4,1);
theta(38:51,:) = repmat(([0,0,180] + 5*randn(1,3))/180*pi,14,1);
theta(52:56,:) = repmat(([0,0,225] + 5*randn(1,3))/180*pi,5,1);
theta(57:65,:) = repmat(([0,0,270] + 5*randn(1,3))/180*pi,9,1);
theta(66,:) = repmat(([0,0,0] + 5*randn(1,3))/180*pi,1,1);




T = zeros(4,4,Pcs+1);
TSeq = zeros(4,4,Pcs*Point);

for ki=1:Pcs+1
   Tk = transl(via(ki,:))*trotx(theta(ki,1))*troty(theta(ki,2))*trotz(theta(ki,3));
   T(:,:,ki) = Tk;
end


for ki=1:Pcs
   Ti = ctraj(T(:,:,ki), T(:,:,ki+1), Point);
   TSeq(:,:,(ki-1)*Point+1:ki*Point) = Ti;
end
about TSeq

% We can plot the motion of this coordinate frame by

% clf; tranimate(TSeq)


%%%%%%------------将姿态矩阵转化为轨迹xyz和欧拉角Euler
x = TSeq(1,4,:);x = x(:);
y = TSeq(2,4,:);y = y(:);
z = TSeq(3,4,:);z = z(:);
position = [x,y,z];
speed = diff(position)/Tf;
accel = diff(speed)/Tf;

euler = [];
for ki=1:size(TSeq,3)
    euler = [euler;tr2rpy(TSeq(:,:,ki))];
end

plot3(x,y,z,'b-o');grid on;
xlabel('x方向');ylabel('y方向');zlabel('z方向');
axis equal
figure
plot(euler(:,3)/pi*180,'r');grid on;hold on;
plot(euler(:,2)/pi*180,'g');grid on;hold on;
plot(euler(:,1)/pi*180,'b');grid on;hold on;
legend('heading','pitch','roll')

save TSeq.mat TSeq

close all
addpath('datasim')
t = 0;
T = 0.02;
t_stop = 189.0;

%%航迹发生器
atti = zeros(3,1);         %滚转、俯仰、偏航（单位：度）
atti_rate = zeros(3,1);    %滚转角速率、俯仰角速率、偏航角速率（单位：度/秒）
veloB = zeros(3,1);        %飞机运动速度--X右翼、Y机头、Z天向（单位：米/秒）
acceB = zeros(3,1);        %飞机运动加速度--X右翼、Y机头、Z天向（单位：米/秒/秒）
posi = zeros(3,1);         %航迹发生器初始位置经度、纬度、高度（单位：度、度、米）
posi = [60;250;-1.9];

atti(1,1) = 0;            %
atti(2,1) = 0;            %
atti(3,1) = 90;            %初始航向角（单位：度）

%%IMU输出
Wibb = zeros(3,1);         %机体系陀螺仪输出（单位：度/秒）
Fb = zeros(3,1);           %机体系加速度计输出（单位：米/秒/秒）
Gyro_fix = zeros(3,1);      %机体系陀螺仪固定误差输出（单位：弧度/秒）
Acc_fix = zeros(3,1);       %机体系加速度计固定误差输出（单位：米/秒/秒）

Gyro_w = zeros(3,1);       %陀螺随机常数（弧度/秒）
Gyro_rw = zeros(3,1);       %陀螺一阶马尔可夫过程（弧度/秒）
Acc_w = zeros(3,1);      %陀螺白噪声（弧度/秒）
Acc_rw = zeros(3,1);        %加速度一阶马尔可夫过程（米/秒）

%%UWB仿真输出
posiG = zeros(3,1);        %UWB输出的飞行器位置测距值

%%捷联惯导仿真
attiN = zeros(3,1);        %飞行器初始姿态
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
ALLNUM = size(accel,1);
for	ki = 1:ALLNUM

    
%     [t,atti,atti_rate,veloB,acceB] = trace_(t,T,atti,atti_rate,veloB,acceB);
    
    atti = euler(ki,:)';
    atti_rate = (euler(ki+1,:)'-euler(ki,:)')/pi*180; %%deg
    
    R = rpy2r(atti');
    posi = position(ki,:)';
    veloB = (R*speed(ki,:)');
    acceB = (R*accel(ki,:)');
    
    [Wibb,Fb,posi,veloN,acceN] = IMUout(T,posi,atti,atti_rate,veloB,acceB,old_veloB,old_atti);

    old_atti = atti;
    old_veloB = veloB;
    
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
%     [Gyro_b,Gyro_r,Gyro_wg,Acc_r] = imu_err_random(t,T,Gyro_b,Gyro_r,Gyro_wg,Acc_r);
    [Gyro_w,Gyro_rw,Acc_w,Acc_rw] = imu_err_random( t,Gyro_w,Gyro_rw,Acc_w,Acc_rw);
    
	%%%%%加入噪声
    Fb = Fb + Acc_w + Acc_rw;
    Wibb = Wibb + Gyro_w/deg_rad + Gyro_rw/deg_rad;   %%%deg/s
	
	%%保存数据
    TraceData = [TraceData;[t,posi',veloN',atti',...
                           (Acc_w + Acc_rw)',(Gyro_w/deg_rad + Gyro_rw/deg_rad)',acceN']];

    
	a_vector =  Fb';g_vector=Wibb';  Temperature = 36.1;SampleTimePoint=t;
    IMUData = [IMUData;[a_vector,g_vector,Temperature,SampleTimePoint]];
   
   kc = kc + 1;
    t = t+Tf;
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
plot(TraceData(:,2),TraceData(:,3),'r','DisplayName','Trace')
axis equal;
% plot(TraceData(:,1),TraceData(:,10),'r','DisplayName','Speed')

grid on
save TraceData.mat TraceData

