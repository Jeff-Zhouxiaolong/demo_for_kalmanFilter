function traceGenerate()


disp('    获取水平面内的坐标位置')
h1 = figure;
title('平面标定')
xlim([0,200]);ylim([0,100]);grid on;box on;
Pcs = 25+18+2+20;px=[];py=[];pz=[];
for ki=1:Pcs
   [x,y,z]=ginput(1);
   px = [px;x];py = [py;y];
   plot(px,py,'r-o');hold on;
   title(['平面标定剩余点数:',num2str(Pcs-ki),'点'])
   xlim([0,200]);ylim([0,100]);grid on;box on;
end
close(h1);

disp('    获取垂直内的坐标位置')
h2= figure;title('高度标定')
h3 = figure;title('轨迹图')
figure(h2)
xlim([0,100]);ylim([0,10]);grid on;box on;
for ki=1:Pcs
   figure(h2);
   [x,z]=ginput(1);
   pz = [pz;z];
   figure(h3);
   num = length(pz);
   hAx = plot3(px(1:num),py(1:num),pz,'r-o');hold on;
   title(['高度标定剩余点数:',num2str(Pcs-ki),'点'])
   xlim([0,200]);ylim([0,100]);zlim([0,10]);grid on;
   xlabel('x方向');ylabel('y方向');zlabel('z方向');
end

%% Frist path generation
via = [px,py,pz];
save TraceSim.mat via Pcs

end