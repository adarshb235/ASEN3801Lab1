function attitude313 = EulerAngles313(DCM)
    attitude313(2) = acos(DCM(3,3));
    attitude313(1) = atan2(DCM(2,3), DCM(1,3));
    attitude313(3) = atan2(DCM(3,2), -DCM(3,1));
end