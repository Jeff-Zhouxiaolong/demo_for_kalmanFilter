function [ t,atti,atti_rate,veloB,acceB ] = trace_( t,T,atti,atti_rate,veloB,acceB )
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
    Temp = t;
    if t > 10
		     t = t - 10;
			if t > 10
		    	t = t - 10;
			end
	end

    if t==0
        acceB(2,1) = 0;                                     %初始对准时间
	elseif t< 10
        acceB = [0;0;0];                                     %初始对准时间
	elseif t < 16
		acceB = [0.1;0;0];                                   %X  0
	elseif t<= 36
		acceB = randn(3,1)/20;                          %匀速运动
	elseif t<=32+10
        acceB = [-0.1;0.1;0];                            %Y
		atti_rate = [7;-7;7];
	elseif t<=38+10
        acceB = [-0.1;-0.1;0];                          %-X
		atti_rate = [-3.5;3.5;-3.5];
	elseif t<=44+10
        acceB = [0.1;-0.1;0];                             %-Y	
		atti_rate =  [-3.5;3.5;-3.5];
	elseif t < 50+10
		acceB = [0.15;0.1;0];                                %X 0
		atti_rate =  randn(3,1)/10;
	elseif t<=56+10
        acceB = [-0.15;0.15;0];                            %Y
		atti_rate =  randn(3,1)/10;
	elseif t < 56+10+15
		acceB = [0;0;0];                                %匀速运动
		atti_rate =  [0;0;0];
	elseif t<=62+10+15
        acceB = [-0.15;-0.15;0];                          %-X
		atti_rate =  randn(3,1)/2;
	elseif t<=62+10+15+20
        acceB = [0;-0;0];                             %匀速运动
		atti_rate =  randn(3,1)/2;
	elseif t<=68+10+15+20
		acceB = [0.15;-0.15;0];                             %-Y
		atti_rate =  randn(3,1)/2;	
	elseif t < 74+10+15+20
		acceB = [0;0.15;0.12];                            %X 0
		atti_rate =  randn(3,1)/2;
	elseif t<=80+10+15+20
        acceB = [0;-0.12;-0.12];                            %Y
		atti_rate =  randn(3,1)/2;
	elseif t<=86+10+15+20
        acceB = [0.05;0.12;-0.12];                          %+X
		atti_rate =  zeros(3,1);
	elseif t<=92+10+15+20
        acceB = [-0.05;-0.12;0.12];                             %-Y
    elseif t < 98+10+15+20+2
		acceB = [0.13;0.12;0.14];                                %X 0
		atti_rate = -[7;-7;7];
	elseif t<=104+10+15+20+4
        acceB = [-0.13;0.13;-0.14];                            %Y
		atti_rate = -[-3.5;3.5;-3.5];
	elseif t<=110+10+15+20+6
        acceB = [-0.13;-0.13;-0.14];                          %-X
		atti_rate =  -[-3.5;3.5;-3.5];
	elseif t<=116+10+15+20+8
        acceB = [0.13;-0.0;0.14];                             %-Y
	end
	t = Temp;
	acceB(1:2) = acceB(1:2)*8;acceB(3) = acceB(3)*3;
	atti_rate = atti_rate;
	
    veloB(1,1)=veloB(1,1)+acceB(1,1)*T;
	veloB(2,1)=veloB(2,1)+acceB(2,1)*T;
	veloB(3,1)=veloB(3,1)+acceB(3,1)*T;
    atti(1,1)=atti(1,1)+atti_rate(1,1)*T;
    atti(2,1)=atti(2,1)+atti_rate(2,1)*T;
    atti(3,1)=atti(3,1)+atti_rate(3,1)*T;
    
    
end