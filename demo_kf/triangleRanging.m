function triangleRanging()
close all
global UKF;
load uwb_HandledFileToMatData.mat;
UKF.BSOneCoordinate = [6.25;4.08;-0.17];%4.08
UKF.BSTwoCoordinate = [0;0;-1.885];
UKF.BSThreeCoordinate = [0.04;9.281;-0.17];
UKF.BSFourCoordinate = [1.705;12.88;-6.27];
UKF.BSFiveCoordinate = [6.35;11.59;-0.12];
% UKF.BaseS_Position = [UKF.BSOneCoordinate,UKF.BSTwoCoordinate,...
% 										 UKF.BSThreeCoordinate,UKF.BSFourCoordinate,...
% 										 UKF.BSFiveCoordinate];
 UKF.BaseS_Position = [UKF.BSOneCoordinate,...
									    	  UKF.BSThreeCoordinate,...
										      UKF.BSFiveCoordinate,UKF.BSTwoCoordinate,UKF.BSFourCoordinate];
UKF.bSPcs = 5;

UKF.changshu =           [norm(UKF.BSOneCoordinate),...
										 norm(UKF.BSThreeCoordinate),...
										 norm(UKF.BSFiveCoordinate),norm(UKF.BSTwoCoordinate),norm(UKF.BSFourCoordinate)]';
CardCoordinate = [3.15,8.24,-0.48]';
% norm(UKF.BSOneCoordinate - CardCoordinate)
bSCoordinate=UKF.BaseS_Position';
bSPcs=5;
A = bSCoordinate(1:bSPcs-1,:)  - repmat(bSCoordinate(bSPcs,:),bSPcs-1,1);
% A = A(:,1:2);
% M = -0.5*inv(A'*A)*A';
M = -0.5*inv(A'*A);
lamda = 1; Iz = [0,0,1]';
c = -(UKF.changshu(1:4,:).^2 -UKF.changshu(5,:).^2);

window = 5;ranging = zeros(5,1);
curRanging = [];
for ki=1:length(Uwbranging_vector)
	  curRanging = [curRanging ;stationnumber_vector(ki),Uwbranging_vector(ki)];
	  if length(curRanging) > 5
		     curRanging(1,:) = [];
			ranging = zeros(5,1);
			  for kj=1:5
				   ranging(curRanging(kj,1)) = curRanging(kj,2) - 0.3;
				   if curRanging(kj,1) == 1
					   ranging(curRanging(kj,1)) = ranging(curRanging(kj,1)) + 0.6493;
				   end
			  end
			  xyz = M* A'*(ranging(1:4,:).^2 - ranging(5,:).^2 + c ) %+ M*lamda*Iz
			  r = getCurXYZError(xyz,UKF.BaseS_Position,ranging);

     end

end


end

function r = getCurXYZError(XYZ,bSCoordinate,uwbRanging)
	r = [];
	dr = [];
	 global UKF;
	bSPcs = UKF.bSPcs;
	for ki=1:bSPcs
		r = [r ,norm(bSCoordinate(:,ki) -XYZ)];
		
	end
% 	disp(['               求解出来的坐标值                                              :',num2str(XYZ')]);
                disp(['                坐标值   :',num2str(XYZ')]);

	if isempty(uwbRanging) ~= 1
      	dr = r' - uwbRanging;
		        disp(['                测距误差:       ',num2str(dr')]);
				disp(['                测量的测距值: ',num2str(uwbRanging')]);
	end
	           disp(['                理想距离:',num2str(r)]);
end
