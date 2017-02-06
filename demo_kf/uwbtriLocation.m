function xyz = uwbtriLocation(rangings)
UKF.BSOneCoordinate = [9.21;1.08;-0.17];%4.08
UKF.BSTwoCoordinate = [0;0;-1.885];
UKF.BSThreeCoordinate = [0;6.281;-1.37];
UKF.BSFourCoordinate = [1.705;12.88;-2.27];
UKF.BSFiveCoordinate = [9.31;11.59;-0.52];
UKF.BaseS_Position = [UKF.BSOneCoordinate,UKF.BSTwoCoordinate,...
					  UKF.BSThreeCoordinate,UKF.BSFourCoordinate,...
					  UKF.BSFiveCoordinate]*30;
UKF.changshu =           [norm(UKF.BSOneCoordinate),norm(UKF.BSTwoCoordinate),...
										 norm(UKF.BSThreeCoordinate),norm(UKF.BSFourCoordinate),...
										 norm(UKF.BSFiveCoordinate)]'*30;
UKF.bSPcs = 5;
bSCoordinate=UKF.BaseS_Position';
bSPcs = UKF.bSPcs;
A = bSCoordinate(1:bSPcs-1,:)  - repmat(bSCoordinate(bSPcs,:),bSPcs-1,1);
M = -0.5*inv(A'*A)* A';
c = (UKF.changshu(1:bSPcs-1,:).^2 -UKF.changshu(bSPcs,:).^2);
xyz = M*(rangings(1:bSPcs-1,:).^2 - rangings(bSPcs,:).^2 - c ); 

end