function [ Gyro_b,Gyro_r,Gyro_wg,Acc_r ] = imu_err_random( t,T,Gyro_b,Gyro_r,Gyro_wg,Acc_r )
% IMU的输出模型
g = 9.7803698;                       %重力加速度（单位：米/秒/秒）
Wie = 7.292115147e-5;                %地球自转角速率（单位：弧度/秒）
deg_rad = pi/180;

Da_bias = [0.0001; 0.001;0.001]*g;   %加速度零偏（应与非随机性误差中加速度零偏保持一致）

Ta = 1800;                           %加速度一阶马尔可夫过程相关时间
Tg = 3600;                           %陀螺一阶马尔可夫过程相关事件

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%随机性误差%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if t == 0
    Acc_r = Da_bias.*randn(3,1);
    
    Gyro_b = 0.1*deg_rad/3600*randn(3,1);   %随机常数0.1（deg/h）
    Gyro_r = 0.1*deg_rad/3600*randn(3,1);   %陀螺一阶马尔可夫过程0.1（deg/h）
    Gyro_wg = 0.1*deg_rad/3600*randn(3,1);  %陀螺白噪声0.1（deg/h）
else
    Acc_wa = sqrt(2*T/Ta)*Da_bias.*randn(3,1);  %加速度一阶马尔可夫过程白噪声
    Acc_r = exp(-1.0*T/Ta)*Acc_r+Acc_wa;        %加速度一阶马尔可夫过程
    
    Gyro_wr = sqrt(2*T/Tg)*0.1*deg_rad/3600*randn(3,1); %陀螺一阶马尔可夫过程白噪声0.1（deg/h）
    Gyro_r = exp(-1.0*T/Tg)*Gyro_r+Gyro_wr; %陀螺一阶马尔可夫过程0.1（deg/h）
    Gyro_wg = 0.1*deg_rad/3600*randn(3,1);  %陀螺白噪声0.1（deg/h）
end

end