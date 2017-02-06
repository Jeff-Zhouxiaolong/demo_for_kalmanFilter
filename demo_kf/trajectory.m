function [ t,atti,atti_rate,veloB,acceB ] = trajectory( t,T,atti,atti_rate,veloB,acceB )
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

	global traceLine;
    
    veloB(1,1)=veloB(1,1)+acceB(1,1)*T;
	veloB(2,1)=veloB(2,1)+acceB(2,1)*T;
	veloB(3,1)=veloB(3,1)+acceB(3,1)*T;
    atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
    atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
    atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
    
    
end