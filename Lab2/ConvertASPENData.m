% Eric W. Frew
% ASEN 3801
% ConvertASPENData
% Created: 9/21/23


function [pos_av_class, att_av_class, pos_tar_class, att_tar_class] = ConvertASPENData(pos_av_aspen, att_av_aspen,  pos_tar_aspen, att_tar_aspen)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inputs:	pos_av_aspen = 3 x n array of position vectors in coordinate
%                           system of the ASPEN motion capture system
%           att_av_aspen = 3 x n array of attitude vectors using the 'helical angles'
%                           provided by the ASPEN motion capture system 
%           pos_tar_aspen = 3 x n array of position vectors in coordinate
%                           system of the ASPEN motion capture system
%           att_tar_aspen = 3 x n array of attitude vectors using the 'helical angles'
%                           provided by the ASPEN motion capture system                               
%
%
% Outputs:	pos_av_class, att_av_class, pos_tar_class, att_tar_class
%
%	        pos_av_class = 3 x n array of position vectors in 'class' coordinate
%                           system with same x-axis as ASPEN and y-axis and
%                           z-axis flipped so z is down
%           att_av_class = 3 x n array of attitude vectors using NASA Standard 
%                           321 Euler angles (Given in order [roll, pitch, yaw])
%           pos_tar_class = 3 x n array of position vectors in 'class' coordinate
%                           system with same x-axis as ASPEN and y-axis and
%                           z-axis flipped so z is down
%           att_tar_class = 3 x n array of attitude vectors using NASA Standard 
%                           321 Euler angles (Given in order [roll, pitch, yaw])
%
%
%
% Methodology: Converts the data provided by the ASPEN lab motion capture system into
% the convention used for ASEN 3801:
%   The inertial frame has down as positive z
%   The body frame has x out front, y to the right, z down
%   The angles are the NASA Standard 321 Euler angles
%
% Position data is converted by changing the signs of the y- and z-axes
%
% Attitude data is convereted by calculating the Direction Cosine Matrix
% from the ASPEN helical angles and then determining the Euler angles from
% the DCM. The conversion equations were taken from:
%   https://academicflight.com/articles/kinematics/rotation-formalisms/euler-angles/
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


[~,len] = size(pos_av_aspen);

% This matrix converts from the ASPEN frame to the "class inertial" frame
% with Z down. Because the inverse of the matrix is the matrix I dont need
% an inverse or transpose in lines 72 or 89
P = [1 0 0;
    0 -1 0;
    0 0 -1];

for i=1:len

    %%% Aerospace Vehicle data
    % Input arrays have NaN if there was no data in the original file
    % Replace with all zeros if that is the case
    if isnan(pos_av_aspen(1,i))
        pos_av_class(:,i) = zeros(3,1);
        att_av_class(:,i) = zeros(3,1);

    else
        pos_av_class(:,i) = [pos_av_aspen(1,i); -pos_av_aspen(2,i); -pos_av_aspen(3,i)];

        xfm_av = makehgtform('xrotate',att_av_aspen(1,i),'yrotate',att_av_aspen(2,i),'zrotate',att_av_aspen(3,i));
        DCM_av = P*xfm_av(1:3,1:3)'*P;
        att_av_class(1,i) = atan2(DCM_av(2,3), DCM_av(3,3)); 
        att_av_class(2,i) = -asin(DCM_av(1,3));
        att_av_class(3,i) = atan2(DCM_av(1,2), DCM_av(1,1));
    end


    %%% Target data
    % Input arrays have NaN if there was no data in the original file
    % Replace with all zeros if that is the case
    if isnan(pos_tar_aspen(1,i))
        pos_tar_class(:,i) = zeros(3,1);
        att_tar_class(:,i) = zeros(3,1);
    else 
        pos_tar_class(:,i) = [pos_tar_aspen(1,i); -pos_tar_aspen(2,i); -pos_tar_aspen(3,i)];

        xfm_tar = makehgtform('xrotate',att_tar_aspen(1,i),'yrotate',att_tar_aspen(2,i),'zrotate',att_tar_aspen(3,i));
        DCM_tar = P*xfm_tar(1:3,1:3)'*P;
        att_tar_class(1,i) = atan2(DCM_tar(2,3), DCM_tar(3,3));
        att_tar_class(2,i) = -asin(DCM_tar(1,3));
        att_tar_class(3,i) = atan2(DCM_tar(1,2), DCM_tar(1,1));
    end

end

    
