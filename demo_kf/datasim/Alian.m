function [xlog,ylog] = Alian(NoiseSeq,SamplePeriod,L)
%%%%%%%%%%阿兰方差----------------------------------
progressBar = 0;
% SamplePeriod = 1/50.0;  %%%%采样周期

figure
sensorNobias = NoiseSeq;
N = size(sensorNobias,1);
seq = 1:L:floor(N/10);
timeTau = zeros(length(seq),1);
alianVariance = zeros(length(seq),1);
% 	clear a_vector;
	for kn=seq
			m = floor(N/kn);  %%%%获取分组个数m
% 			Tau = kn*SamplePeriod;
			timeTau(kn) = kn*SamplePeriod;
			%%%%对数据进行分组，分为m组，每组kn个数据
			FenzuMatrix = reshape(sensorNobias(1:kn*m),kn,m);  
			A_t = mean(FenzuMatrix);   %%%%每个分组的均值形成的数组
% 			quadraticSum = alianVarianceValue(A_t);

			alianVariance(kn) = alianVarianceValue(A_t);     
			progressBar = progressBar + 1;
			if progressBar == 1000
				  disp(['      程序已经进行到:',num2str(100*kn/floor(N/10)),'  %']);
				  progressBar = 0;
			end
	end
	
	progressBar = 0;
	xlog = log(timeTau);
	ylog=log(alianVariance);
	plot(xlog,ylog,'.');
	grid on;

	% 创建 xlabel
	xlabel('时间\tau: s');

	% 创建 ylabel
	ylabel('alianVariance','FontWeight','bold');

	% 创建 title
	title(['惯性元件','Random walk','阿兰方差分析'],'FontWeight','bold','FontSize',12);
	
end

function quadraticSum = alianVarianceValue(A_t)
	A_t = diff(A_t); 
	A_t = A_t.^2;
	quadraticSum = sum(A_t/2/(size(A_t,2)-1));

end