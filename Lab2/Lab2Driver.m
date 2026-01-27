clc;
clear;
close all;


filenames = ["3801_Sec002_Test1.csv" "3801_Sec002_Test2.csv" "3801_Sec002_Test3.csv"];
for i = 1:1
    data = readmatrix(filenames(i));
    data = rmmissing(data);
    data(:,2) = [];

    pos_av_aspen = data(:,11:13)'; % N frame data
    att_av_aspen = data(:,8:10)';
    pos_tar_aspen = data(:,5:7)';
    att_tar_aspen = data(:,2:4)';

    [t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filenames(i));

    % Problem 3
    figure;
    hold on;
    grid on;
    box on;
    title("Problem 3: Trajectory of Aerospace Vehicle and Target")
    plot3(pos_av_aspen(1, :), pos_av_aspen(2, :), pos_av_aspen(3, :), '-b');
    plot3(pos_tar_aspen(1, :), pos_tar_aspen(2, :), pos_tar_aspen(3, :), '--r');
    view(3)

    % Problem 4.1
    figure;
    hold on;
    grid on;
    box on;
    sgtitle("Problem 4.1: Components of Position over Time")

    subplot(3, 1, 1)
    hold on;
    plot(t_vec, av_pos_inert(1, :), 'b');
    plot(t_vec, tar_pos_inert(1, :), 'r');
    ylabel('X Position (m)');
    title('Position over Time');

    subplot(3, 1, 2)
    hold on;
    plot(t_vec, av_pos_inert(2, :), 'b');
    plot(t_vec, tar_pos_inert(2, :), 'r');
    ylabel('Y Position (m)');
    title('Position vs Time');

    subplot(3, 1, 3)
    hold on;
    plot(t_vec, av_pos_inert(3, :), 'b');
    plot(t_vec, tar_pos_inert(3, :), 'r');
    ylabel('Z Position (m)');
    title('Position vs Time');
    
    % Problem 4.2
    figure;
    hold on;
    grid on;
    box on;
    sgtitle("Problem 4.2: Euler Angles over Time")


    subplot(3, 1, 1)
    hold on;
    plot(t_vec, av_att(1, :), 'b');
    plot(t_vec, tar_att(1, :), 'r');
    ylabel('Pitch Angle (rad)');
    title('Average Pitch vs Time');

    subplot(3, 1, 2)
    hold on;
    plot(t_vec, av_att(2, :), 'b');
    plot(t_vec, tar_att(2, :), 'r');
    ylabel('Roll Angle (rad)');
    title('Average Roll vs Time');

    subplot(3, 1, 3)
    hold on;
    plot(t_vec, av_att(3, :), 'b');
    plot(t_vec, tar_att(3, :), 'r');
    ylabel('Yaw Angle (rad)');
    title('Average Yaw vs Time');

    eulerAngles313_tar = zeros(3, length(att_tar_aspen));
    eulerAngles313_av = zeros(3, length(att_tar_aspen));
    
    for j = 1:length(att_tar_aspen)
        DCM = RotationMatrix321(av_att(:, j));
        temp_att_313 = EulerAngles313(DCM); 
        eulerAngles313_av(:, j) = temp_att_313;

        DCM = RotationMatrix321(tar_att(:, j));
        temp_att_313 = EulerAngles313(DCM); 
        eulerAngles313_tar(:, j) = temp_att_313;
    end

    % Problem 5
    figure;
    hold on;
    grid on;
    box on;
    sgtitle("Problem 5: 3-1-3 Euler Angles over Time")


    subplot(3, 1, 1)
    hold on;
    plot(t_vec, eulerAngles313_av(1, :), 'b');
    plot(t_vec, eulerAngles313_tar(1, :), 'r');
    ylabel('Pitch Angle (rad)');
    title('Average Pitch vs Time');

    subplot(3, 1, 2)
    hold on;
    plot(t_vec, eulerAngles313_av(2, :), 'b');
    plot(t_vec, eulerAngles313_tar(2, :), 'r');
    ylabel('Roll Angle (rad)');
    title('Average Roll vs Time');

    subplot(3, 1, 3)
    hold on;
    plot(t_vec, eulerAngles313_av(3, :), 'b');
    plot(t_vec, eulerAngles313_tar(3, :), 'r');
    ylabel('Yaw Angle (rad)');
    title('Average Yaw vs Time');
end

% Problem 5

pos_rel_E = tar_pos_inert - av_pos_inert;

figure;
hold on;
grid on;
box on;
sgtitle("Problem 6: Relative Positions of Target With Respect to the Aerospace Vehicle in E Frame")


subplot(3, 1, 1)
hold on;
plot(t_vec, pos_rel_E(1,:), 'b');
ylabel('X position (m)');
title('Relative X Position of Target With Respect to the Aerospace Vehicle in E Frame');

subplot(3, 1, 2)
hold on;
plot(t_vec, pos_rel_E(2,:), 'b');
ylabel('Y position (m)');
title('Relative Y Position of Target With Respect to the Aerospace Vehicle in E Frame');

subplot(3, 1, 3)
hold on;
plot(t_vec, pos_rel_E(3,:), 'b');
ylabel('Z position (m)');
title('Relative Z Position of Target With Respect to the Aerospace Vehicle in E Frame');
