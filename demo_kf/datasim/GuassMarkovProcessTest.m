clear all
%%%%%%Guass-Markov Process Noise Simlinker-------------------------------
Tc = 360;  %%%相关时间
beta = 1/Tc;
dT = 0.02;
whiteNoiseStd = sqrt(1e-7);
Pcs = 960000;
NoiseSeq = zeros(Pcs,1);

NoiseSeq(1) = whiteNoiseStd*randn(1);
Noise = NoiseSeq(1);
for ki=2:Pcs   %%%5.3个小时数据
         Noise = exp(-beta*dT)*Noise + whiteNoiseStd*randn(1);
		 NoiseSeq(ki) = Noise;
end
GMp = NoiseSeq;
t=0.02:0.02:length(NoiseSeq)*0.02;
plot(t/60,NoiseSeq,'r');hold on
% 
% [r lag] = xcorr(NoiseSeq,'coeff');
% plot(lag*0.02,r,'.-')


%%%%%%Random walk Noise Simlinker--------------------------------------
whiteNoiseStd = sqrt(2.6e-6);
NoiseSeq = zeros(Pcs,1);

NoiseSeq(1) = whiteNoiseStd*randn(1);
Noise = NoiseSeq(1);
for ki=2:Pcs   %%%5.3个小时数据
         Noise = Noise + whiteNoiseStd*randn(1);
		 NoiseSeq(ki) = Noise;
end
RWp = NoiseSeq;
t=0.02:0.02:length(NoiseSeq)*0.02;
plot(t/60,NoiseSeq,'b');hold on;grid on



%%%%%%Guass Distribution  Noise Simlinker----------------------------------
whiteNoiseStd = sqrt(7.3e-4);
GuassNoise = randn(Pcs,1)*sqrt(whiteNoiseStd);
plot(t,GuassNoise,'g')


%%%%%%混合噪声------------------------------------------------------------
HybridNoise = GuassNoise + GMp + RWp;
plot(t,HybridNoise,'k')

legend('Guass Markov Noise','Random Walk Noise','Guass Noise','Hybrid Noise')
xlabel('Time:/min')
ylabel('Accel:m/s^2')

% figure
% [r lag] = xcorr(NoiseSeq,'coeff');
% plot(lag*0.02,r,'.-')
clear NoiseSeq GuassNoise GMp  RWp t;
SamplePeriod = 0.02;
[xlog,ylog] = Alian(HybridNoise,SamplePeriod,3);

