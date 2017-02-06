function [Ranging] = UwbOut(bS,position)
    UKF.BSOneCoordinate = [9.21;1.08;-0.17];%4.08
    UKF.BSTwoCoordinate = [0;0;-1.885];
    UKF.BSThreeCoordinate = [0;6.281;-1.37];
    UKF.BSFourCoordinate = [1.705;12.88;-2.27];
    UKF.BSFiveCoordinate = [9.31;11.59;-0.52];
    UKF.BaseS_Position = [UKF.BSOneCoordinate,UKF.BSTwoCoordinate,...
                          UKF.BSThreeCoordinate,UKF.BSFourCoordinate,...
                          UKF.BSFiveCoordinate]*30;

    UKF.bSPcs = 5;
    bSPcs = UKF.bSPcs;
    MeasureNoiseVariance =[2.98e-03,2.9e-03,...
                        1.8e-03,1.2e-03,...
                        2.4e-03,2.69e-03];%%%%UWB定位的测量噪声
    Rk = diag(MeasureNoiseVariance);
    UKF.Rk = Rk;  

    Ranging = [];
    TM = repmat(position,1,bSPcs) - UKF.BaseS_Position;%%/UKF.LightSpeed;
    for ki=1:bSPcs
        noise = randn(1)*sqrt(MeasureNoiseVariance(ki));
        noise = 0;
        Ranging = [Ranging;norm(TM(:,ki)) + noise];
    end
end

