function attitude321 = EulerAngles321(DCM)
% EulerAngles321 - extracts euler angles from 3-2-1 DCM
% attitude321 = EulerAngles321(DCM)
% Inputs: DCM = 3x3 matrix for 
%
    attitude321(1) = atan2(DCM(2,3), DCM(3,3));
    attitude321(2) = asin(-DCM(1, 3));
    attitude321(3) = atan2(DCM(1,2), DCM(1,1));
end
