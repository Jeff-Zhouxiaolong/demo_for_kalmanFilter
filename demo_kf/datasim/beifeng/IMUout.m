function [ Wibb,Fb,posi ] = IMUout( T,posi,atti,atti_rate,veloB,acceB,old_veloB,old_atti )

%%%%%%%%%%%%%%%%%%%%%与地球有关的参数%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Re = 6378137.0;               %地球半径（单位：米）
f = 1/298.257;                %地球椭圆率
Wie = 7.292115147e-5;         %地球自转角速率（单位：弧度/秒）
g = 9.7803698;                %重力加速度（单位：米/秒/秒）

%%飞行器位置
lon = posi(1,1)*pi/180;
lat = posi(2,1)*pi/180;
height = posi(3,1);           

%%地球曲率半径
Rm = Re*(1-2*f+3*f*sin(lat)*sin(lat));
Rn = Re*(1+f*sin(lat)*sin(lat));   
                              
%%姿态角和姿态角速率
roll = old_atti(1,1)*pi/180;
pitch = old_atti(2,1)*pi/180;
head = old_atti(3,1)*pi/180;
droll = atti_rate(1,1)*pi/180;
dpitch = atti_rate(2,1)*pi/180;
dhead = atti_rate(3,1)*pi/180;

%%DCM，坐标系N-->B
Cbn = [cos(roll)*cos(head)+sin(roll)*sin(pitch)*sin(head), -cos(roll)*sin(head)+sin(roll)*sin(pitch)*cos(head), -sin(roll)*cos(pitch);
       cos(pitch)*sin(head),                               cos(pitch)*cos(head),                                sin(pitch);
       sin(roll)*cos(head)-cos(roll)*sin(pitch)*sin(head), -sin(roll)*sin(head)-cos(roll)*sin(pitch)*cos(head), cos(roll)*cos(pitch)];
   
%%欧拉角变换矩阵
Eluer_M=[cos(roll), 0, sin(roll)*cos(pitch);
         0,         1, -sin(pitch);
         sin(roll), 0, -cos(pitch)*cos(roll)];
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%陀螺仪输出%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Wnbb = Eluer_M*[dpitch,droll,dhead]';

veloN = Cbn'*old_veloB;
Ve = veloN(1,1);
Vn = veloN(2,1);
Vu = veloN(3,1);

Wenn = [-Vn/(Rm+height), Ve/(Rn+height), Ve/(Rn+height)*tan(lat)]';
Wien = [0,               Wie*cos(lat),   Wie*sin(lat)]';

Wibb = Cbn*(Wien+Wenn)+Wnbb;    %单位：弧度/秒
Wibb = Wibb/pi*180;             %单位：度/秒

%%%%%%%%%%%%%%%%%%%%%%%%%%%%加速度计输出%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
roll = atti(1,1)*pi/180;
pitch = atti(2,1)*pi/180;
head = atti(3,1)*pi/180;

acceN = Cbn'*(acceB+cross(Wnbb,old_veloB));
Fn = acceN+cross(2*Wien+Wenn,veloN)-[0,0,-1*g]';
Fb = Cbn*Fn;                    %单位：米/秒/秒

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%位置计算%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
veloN = Cbn'*veloB;
Ve = veloN(1,1);
Vn = veloN(2,1);
Vu = veloN(3,1);

height = height+T*Vu;
lat = lat+T*(Vn/(Rm+height));
lon = lon+T*(Ve/(Rn+height)*cos(lat));

posi(1,1) = lon*180/pi;
posi(2,1) = lat*180/pi;
posi(3,1) = height;

end

