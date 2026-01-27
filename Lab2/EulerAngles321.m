function attitude321 = EulerAngles321(DCM)
    attitude321(1) = atan2(DCM(2,3), DCM(3,3));
    attitude321(2) = asin(-DCM(1, 3));
    attitude321(3) = atan2(DCM(1,2), DCM(1,1));
end