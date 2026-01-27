function DCM = RotationMatrix313(attitude313)
% RotationMatrix321 - Computes DCM for 3-1-3 Euler angle sequence
%   DCM = RotationMatrix313(attitude313)
%   
%   Inputs:
%       attitude313 = [phi; theta; psi] (roll, pitch, yaw in radians)
%   Outputs:
%       DCM = 3x3 Direction Cosine Matrix (rotation from body to inertial)

c_a = cos(attitude313(1));
c_b = cos(attitude313(2));
c_g = cos(attitude313(3));

s_a = sin(attitude313(1));
s_b = sin(attitude313(2));
s_g = sin(attitude313(3));


DCM = [c_g*c_a - s_g*s_a*c_b c_g*s_a + s_g*c_b*c_a s_g*s_b; ...
       -s_g*c_a - c_g*s_a*c_b -s_g*s_a + c_g*c_b*c_a c_g*s_b; ...
       s_b*s_g -s_b*c_a c_b];

end