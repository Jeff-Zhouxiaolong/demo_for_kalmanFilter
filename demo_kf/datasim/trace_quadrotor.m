function [ t,atti,atti_rate,veloB,acceB,acce ] = trace_quadrotor( t,T,atti,atti_rate,veloB,acceB,acce )

format long;

if t == 0
    acce = [0,0,0]';                              %初始对准时间
elseif t<=10
    acce = [0,0,0.6]';                            %垂直飞行,直至最大上升速度
elseif t<=510
    acce = [0,0,0]';                              %当前高度30，最大飞行速度垂直爬升,爬升3000m
elseif t<=520
    acce = [0,0,-0.6]';                           %垂向减速至悬停
elseif t<=525
    acce = [0,0,0]';                              %悬停状态，维持5s,高度3060m
elseif t<=525+2.5
    atti_rate = [0,-6,0]';                         
    acce = [0,2,0]';                              %低头前飞
elseif t<=535
    atti_rate = [0,0,0]';
elseif t<=545
    acce = [0,0,0]';                              %匀速前飞
elseif t<=645                      
    atti_rate = [0,0,3.6]';                        %圆周运动
%  elseif t<=755
     
end

roll = atti(1,1)*pi/180;
pitch = atti(2,1)*pi/180;
head = atti(3,1)*pi/180;

atti=atti+atti_rate*T;

Cbn = [cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
   cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
   sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];

acceB = Cbn*acce;

veloB = veloB+acceB*T;

end
