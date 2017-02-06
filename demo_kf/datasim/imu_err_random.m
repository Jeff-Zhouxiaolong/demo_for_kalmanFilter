function [Gyro_w,Gyro_rw,Acc_w,Acc_rw] = imu_err_random( t,Gyro_w,Gyro_rw,Acc_w,Acc_rw)
% IMU的输出模型

%%%%Gyro: 偏差不稳定性+白噪声+随机游走噪声
%%%%Accel: 偏差不稳定性+白噪声+随机游走噪声
g = 9.7803698;                     %重力加速度（单位：米/秒/秒）
mg = g/1000;

%%%%%噪声参数
ProcessNoiseVariance = [3.9e-04    4.5e-4       7.9e-4;   %%%Accelerate_Variance
                        1.9239e-7, 3.5379e-7, 2.4626e-7;%%%Accelerate_Bias_Variance
                        8.7e-04,1.2e-03,1.1e-03;      %%%Gyroscope_Variance
                        1.3111e-9,2.5134e-9,    2.4871e-9    %%%Gyroscope_Bias_Variance
                      ];
                                              
%%%%%开机上电 零偏
AccelInit_bias = [5;5;8]*mg;   %%x/y: +-50 mg  z: +-80 mg
GyroInit_bias = 1/400;   %%%+-2 deg/s

%%%%%白噪声
Da_w_sigma = sqrt(ProcessNoiseVariance(1,:)');   
Dg_w_sigma = sqrt(ProcessNoiseVariance(3,:)');   

%%%%%随机游走噪声:w = 积分(白噪声)
Da_rw_sigma = sqrt(ProcessNoiseVariance(2,:)');   
Dg_rw_sigma = sqrt(ProcessNoiseVariance(4,:)');   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%随机性误差%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if t == 0

    Acc_b = AccelInit_bias.*randn(3,1);    %%% 加速度计初始化零偏
    Gyro_b = GyroInit_bias*randn(3,1);   %%% 陀螺仪初始化零偏

    Acc_rw = Acc_b;
    Gyro_rw = Gyro_b;
else
    Acc_w = Da_w_sigma.*randn(3,1);               %加速度白噪声
    Acc_rw = Acc_rw + Da_rw_sigma.*randn(3,1);    %加速度随机游走噪声(积分)

    Gyro_w = Dg_w_sigma.*randn(3,1);               %角速度白噪声
    Gyro_rw = Gyro_rw + Dg_rw_sigma.*randn(3,1);   %角速度随机游走噪声(积分)
end

end



