t = 0;
T = 0.01;
t_stop = 1800.0;

%%航迹发生器
atti = zeros(3,1);         %滚转、俯仰、偏航（单位：度）
atti_rate = zeros(3,1);    %滚转角速率、俯仰角速率、偏航角速率（单位：度/秒）
veloB = zeros(3,1);        %飞机运动速度--X右翼、Y机头、Z天向（单位：米/秒）
acceB = zeros(3,1);        %飞机运动加速度--X右翼、Y机头、Z天向（单位：米/秒/秒）
posi = zeros(3,1);         %航迹发生器初始位置经度、纬度、高度（单位：度、度、米）
posi = [110;20;500];

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

%%GPS仿真输出
posiG = zeros(3,1);        %GPS输出的飞行器位置（经度（度）、纬度（度）、高度（米））
veloG = zeros(3,1);        %GPS输出的飞行器速度（东向（米/秒），北向（米/秒），天向（米/秒））


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

while t<=1
    [t_alig,atti,atti_rate,veloB,acceB] = trace(0,T,atti,atti_rate,veloB,acceB);
    
    %%IMU仿真输出
    [Wibb,Fb,posi] = IMUout(T,posi,atti,atti_rate,veloB,acceB,old_veloB,old_atti);
    [Gyro_b,Gyro_r,Gyro_wg,Acc_r] = imu_err_random(t,T,Gyro_b,Gyro_r,Gyro_wg,Acc_r);
    
    Fb = Fb+Acc_r;
    Wibb = Wibb+Gyro_b/deg_rad+Gyro_r/deg_rad+Gyro_wg/deg_rad;
    
    tmp_Fb = tmp_Fb+Fb;
    tmp_Wibb = tmp_Wibb+Wibb;
    
    kc = kc+1;
    t = t+T;
end

Fb = tmp_Fb/kc;
Wibb = tmp_Wibb/kc;

attiN = atti;
[attiN] = align_cal(Wibb,Fb,posiN);
[veloN] = veloN0(atti,veloB);


input('press any key to continue...');


%%%%%%%%%%%%%%%%%%%%%%%%%%%数据记录%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

TraceData = zeros(t_stop/T,10);
IMUData = zeros(t_stop/T,7);
GPSData = zeros(t_stop+1,7);
SinsData = zeros(t_stop/T,10);
KALData = zeros(t_stop+1,19);
ErrData = zeros(t_stop+1,19);

kc = 1;
kc_sins = 1;    
kc_kal = 1;
t = 0;                   %导航开始

[posiG,veloG] = simu_gps(t,posi,atti,veloB);                       %GPS仿真输出

[Xc,PK,Xerr] = kalman_gps_init(posiN,Xc,PK,Xerr);                  %Kalman滤波初始化

GPSData(kc_kal,:) = [t,posiG',veloG'];
KALData(kc_kal,:) = [t,Xerr];
ErrData(kc_kal,:) = [t,Xc'];                                       %保存初始数据

while t<=t_stop
    if (t>=kc*50-T && t<kc*50)
        kc = kc+1;
        disp(t);
    end
    
    old_atti = atti;
    old_veloB = veloB;
    
    [t,atti,atti_rate,veloB,acceB] = trace(t,T,atti,atti_rate,veloB,acceB);
    
    [Wibb,Fb,posi] = IMUout(T,posi,atti,atti_rate,veloB,acceB,old_veloB,old_atti);
    
    [Gyro_b,Gyro_r,Gyro_wg,Acc_r] = imu_err_random(t,T,Gyro_b,Gyro_r,Gyro_wg,Acc_r);
    
    Fb = Fb+Acc_r;
    Wibb = Wibb+Gyro_b/deg_rad+Gyro_r/deg_rad+Gyro_wg/deg_rad;
    
    
    %%姿态解算
%     [ attiN,WnbbA_old ] = atti_cal_cq_modi( T,Wibb,attiN,veloN,posiN,WnbbA_old );
    attiN = atti;
    %%速度解算
    [veloN] = velo_cal( T,posiN,attiN,veloN,Fb );
    
    %%位置解算
    [ posiN ] = posi_cal( T,posiN,veloN );
    
    %%%%%%%%%%%%%%%%%%%%%%%%此处模拟GPS仿真，输出频率1HZ%%%%%%%%%%%%%%%%%%%%%
    if T_M < 1
        T_M = T_M+T;
    end
    
    if (T_M >(1-T))&&(T_M <(1+T))
        kflag = 1;
        kc_kal = kc_kal+1;
        [posiG,veloG] = simu_gps(t,posi,atti,veloB);                       %GPS仿真输出 
        
        [ Xc,PK,Xerr ] = Kalman_GPS( T_D,Fb,attiN,posiN,veloN,posiG,veloG,Xc,PK,Xerr,kflag );
        [ attiN,veloN,posiN ] = kalm_modi( attiN,veloN,posiN,Xc );
        
        GPSData(kc_kal,:) = [t,posiG',veloG'];
        KALData(kc_kal,:) = [t,Xerr];
        ErrData(kc_kal,:) = [t,Xc'];                        %保存数据
        
        Xc = zeros(18,1);
        
        kflag = 0;
        T_M = 0;
    end
    
    
    %%保存数据
    TraceData(kc_sins,:) = [t,atti',veloB',posi'];
    
    IMUData(kc_sins,:) = [t,Wibb',Fb'];
    
    SinsData(kc_sins,:) = [t,attiN',veloN',posiN'];
    
    kc_sins = kc_sins+1;
    
    t = t+T;                     %时间更新
    
end
