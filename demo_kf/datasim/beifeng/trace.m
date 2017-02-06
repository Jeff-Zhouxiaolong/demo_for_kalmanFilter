function [ t,atti,atti_rate,veloB,acceB ] = trace( t,T,atti,atti_rate,veloB,acceB )
%                   航迹发生器
% 
%   输入参数：
%   t           仿真时间
%   T           仿真步长
%   atti        横滚、俯仰、航向（单位：度）
%   atti_rate   横滚速率、俯仰速率、航向速率（单位：度/秒）
%   veloB       飞机运动速度--X右翼、Y机头、Z天向（单位：米/秒）
%   acceB       飞机运动加速度--X右翼、Y机头、Z天向（单位：米/秒/秒）
%   posi         航迹发生器初始位置经度、纬度、高度（单位：度、度、米）

    if t==0
        acceB(2,1) = 0;                                     %初始对准时间
    elseif t<=20
        acceB(2,1) = 8.0;                                   %滑跑
    elseif t<=24
        atti_rate(2,1)= 7.5;
        acceB(2,1) = 10;                                    %加速拉起
    elseif t<=64
        atti_rate(2,1)= 0;                                  %爬高
    elseif t<=68
        atti_rate(2,1)= -7.5;                               %把机头拉平
        acceB(2,1) = 0;
    elseif t<=668
        atti_rate(2,1)= 0;                                  %平直飞行
    elseif t<=671
        atti_rate(1,1)= 10;                                 %倾斜转弯
    elseif t<=731
        atti_rate(1,1)= 0;
        atti_rate(3,1)= 1.5;                                %转弯
    elseif t<=734
        atti_rate(3,1)=0;
        atti_rate(1,1)= -10;                                %拉平
    elseif t<=1334
        atti_rate(1,1)= 0;                                  %平行直飞
    elseif t<=1342
        atti_rate(2,1)=7.5;
        acceB(2,1) = 2;                                     %加速拉起
    elseif t<=1374
        atti_rate(2,1)=0;
    elseif t<=1382
        acceB(2,1) = 0; 
        atti_rate(2,1)=-7.5;                                %拉平
    elseif t<=1862
        atti_rate(2,1)= 0;                                  %平直飞行
    elseif t<=1902
        acceB(2,1) = -2.5;                                  %减速飞行
    elseif t<=1905
        atti_rate(1,1)=10;
        acceB(2,1) = 0;                                     %倾斜预转弯
    elseif t<=1965
        atti_rate(1,1)=0;
        atti_rate(3,1)=1.5;                                 %转弯
    elseif t<=1968
        atti_rate(1,1)=-10;
        atti_rate(3,1)=0;                                   %改平
    elseif t<=2568
        atti_rate(1,1)=0;                                   %平直飞行
    elseif t<=2574
        atti_rate(2,1)=-7.5;                                %低头
    elseif t<=2594
        atti_rate(2,1)= 0;                                  %俯冲 
    elseif t<=2600
        atti_rate(2,1)= 7.5;                                %改平
    elseif t<=2603
        atti_rate(2,1)= 0;
        atti_rate(1,1)= 10;                                 %倾斜转弯
    elseif t<=2663
        atti_rate(3,1)= 1.5;
        atti_rate(1,1)= 0;                                  %转弯
    elseif t<=2666
        atti_rate(3,1)= 0;
        atti_rate(1,1)= -10;                                %改平
    elseif t<=3600
        atti_rate(1,1)=0;                                   %平直飞行
    end
    
    veloB(2,1)=veloB(2,1)+acceB(2,1)*T;
    atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
    atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
    atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
    
    
end