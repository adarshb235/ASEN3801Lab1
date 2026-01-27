function [t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filename)
% LoadASPENData - Load and process ASPEN lab motion capture data
%
% Inputs:
%   filename - string containing the name of the .csv file with motion capture data
%
% Outputs:
%   t_vec - 1 x n time vector in seconds
%   av_pos_inert - 3 x n matrix of aerospace vehicle position vectors in meters (Frame E)
%   av_att - 3 x n matrix of aerospace vehicle 3-2-1 Euler angles in radians (Frame E)
%   tar_pos_inert - 3 x n matrix of target position vectors in meters (Frame E)
%   tar_att - 3 x n matrix of target 3-2-1 Euler angles in radians (Frame E)

    data = readmatrix(filename);
    data = rmmissing(data);
    data(:,2) = [];

    t_vec = data(:,1)' / 100;

    [av_pos_inert, av_att, tar_pos_inert, tar_att] = ConvertASPENData(data(:,11:13)', data(:,8:10)', data(:,5:7)', data(:,2:4)');
end