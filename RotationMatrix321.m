function DCM = RotationMatrix321(attitude321)
% RotationMatrix321 - Computes DCM for 3-2-1 Euler angle sequence
%   DCM = RotationMatrix321(attitude321)
%   
%   Inputs:
%       attitude321 = [phi; theta; psi] (roll, pitch, yaw in radians)
%   Outputs:
%       DCM = 3x3 Direction Cosine Matrix (rotation from body to inertial)

c_a = cos(attitude321(1));
c_b = cos(attitude321(2));
c_g = cos(attitude321(3));

s_a = sin(attitude321(1));
s_b = sin(attitude321(2));
s_g = sin(attitude321(3));


DCM = [c_b*c_g c_b*s_g -s_b; ...
    -c_a*s_g + s_a*s_b*c_g c_a*c_g + s_a*s_b*s_g ...
    s_a*c_b; s_a*s_g + c_a*s_b*c_g -s_a*c_g + c_a*s_b*s_g c_a*c_b];
end