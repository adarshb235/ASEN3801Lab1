clc;
clear;
close all;


filenames = ["3801_Sec002_Test1.csv" "3801_Sec002_Test2.csv" "3801_Sec002_Test3.csv"];
for i = 1:length(filenames)
    data = readmatrix(filenames(i)); % treat data
    data = rmmissing(data);
    data(:,2) = [];

    pos_av_aspen = data(:,11:13)'; % N frame data
    att_av_aspen = data(:,8:10)';
    pos_tar_aspen = data(:,5:7)';
    att_tar_aspen = data(:,2:4)';

    [t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filenames(i)); % read in E frame data

    % Problem 3
    figure(i * 100 + 1);
    hold on;
    grid on;
    box on;
    title("Problem 3: Trajectory of Aerospace Vehicle and Target")
    plot3(pos_av_aspen(1, :), pos_av_aspen(2, :), pos_av_aspen(3, :), '-b');
    plot3(pos_tar_aspen(1, :), pos_tar_aspen(2, :), pos_tar_aspen(3, :), '--r');
    view(3)
    legend('Aerospace Vehicle', 'Target', 'Location', 'best')

    % Problem 4.1
    figure(i * 100 + 2);
    hold on;
    grid on;
    box on;
    sgtitle("Problem 4.1: Components of Position over Time")

    subplot(3, 1, 1)
    hold on;
    grid on;
    plot(t_vec, av_pos_inert(1, :), 'b');
    plot(t_vec, tar_pos_inert(1, :), 'r');
    ylabel('X Position (m)');
    title('X Position over Time');
    legend('Aerospace Vehicle', 'Target', 'Location', 'best')


    subplot(3, 1, 2)
    hold on;
    grid on;
    plot(t_vec, av_pos_inert(2, :), 'b');
    plot(t_vec, tar_pos_inert(2, :), 'r');
    ylabel('Y Position (m)');
    title('Y Position vs Time');
    legend('Aerospace Vehicle', 'Target', 'Location', 'best')

    subplot(3, 1, 3)
    hold on;
    grid on;
    plot(t_vec, av_pos_inert(3, :), 'b');
    plot(t_vec, tar_pos_inert(3, :), 'r');
    ylabel('Z Position (m)');
    title('Z Position vs Time');
    legend('Aerospace Vehicle', 'Target', 'Location', 'best')
    
    % Problem 4.2
    figure(i * 100 + 3);
    hold on;
    grid on;
    box on;
    sgtitle("Problem 4.2: Euler Angles over Time")


    subplot(3, 1, 1)
    hold on;
    grid on;
    plot(t_vec, av_att(1, :), 'b');
    plot(t_vec, tar_att(1, :), 'r');
    ylabel('Pitch Angle (rad)');
    title('Average Pitch vs Time');
    legend('Aerospace Vehicle', 'Target', 'Location', 'best')

    subplot(3, 1, 2)
    hold on;
    grid on;
    plot(t_vec, av_att(2, :), 'b');
    plot(t_vec, tar_att(2, :), 'r');
    ylabel('Roll Angle (rad)');
    title('Average Roll vs Time');
    legend('Aerospace Vehicle', 'Target', 'Location', 'best')

    subplot(3, 1, 3)
    hold on;
    grid on;
    plot(t_vec, av_att(3, :), 'b');
    plot(t_vec, tar_att(3, :), 'r');
    ylabel('Yaw Angle (rad)');
    title('Average Yaw vs Time');
    legend('Aerospace Vehicle', 'Target', 'Location', 'best')

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
    figure(i * 100 + 4);
    hold on;
    grid on;
    box on;
    sgtitle("Problem 5: 3-1-3 Euler Angles over Time")


    subplot(3, 1, 1)
    hold on;
    grid on;
    plot(t_vec, eulerAngles313_av(1, :), 'b');
    plot(t_vec, eulerAngles313_tar(1, :), 'r');
    ylabel('Pitch Angle (rad)');
    title('Average Pitch vs Time');
    legend('Aerospace Vehicle', 'Target', 'Location', 'best')

    subplot(3, 1, 2)
    hold on;
    grid on;
    plot(t_vec, eulerAngles313_av(2, :), 'b');
    plot(t_vec, eulerAngles313_tar(2, :), 'r');
    ylabel('Roll Angle (rad)');
    title('Average Roll vs Time');
    legend('Aerospace Vehicle', 'Target', 'Location', 'best')

    subplot(3, 1, 3)
    hold on;
    grid on;
    plot(t_vec, eulerAngles313_av(3, :), 'b');
    plot(t_vec, eulerAngles313_tar(3, :), 'r');
    ylabel('Yaw Angle (rad)');
    title('Average Yaw vs Time');
    legend('Aerospace Vehicle', 'Target', 'Location', 'best')

    % Problem 6
    rel_p_inert = tar_pos_inert - av_pos_inert;

    figure(i * 100 + 5);
    sgtitle("Problem 6: Components of Relative Position in Inertial Frame")

    subplot(3, 1, 1)
    hold on;
    grid on;
    title("X Position vs Time")
    xlabel("Time (s)")
    ylabel("X Position (m)")
    plot(t_vec, rel_p_inert(1, :), 'k')

    subplot(3, 1, 2)
    hold on;
    grid on;
    title("Y Position vs Time")
    xlabel("Time (s)")
    ylabel("Y Position (m)")
    plot(t_vec, rel_p_inert(2, :), 'k')

    subplot(3, 1, 3)
    hold on;
    grid on;
    title("Z Position vs Time")
    xlabel("Time (s)")
    ylabel("Z Position (m)")
    plot(t_vec, rel_p_inert(3, :), 'k')

    % Problem 7

    rel_p_body = zeros(3, length(av_att));
    for j = 1:length(av_att)
        DCM = RotationMatrix321(av_att(:, j));
        rel_p_body(:, j) = DCM * rel_p_inert(:, j);
    end

    figure(i * 100 + 6);
    sgtitle("Problem 7: Components of Relative Position in Body Frame")

    subplot(3, 1, 1)
    hold on;
    grid on;
    title("X Position vs Time")
    xlabel("Time (s)")
    ylabel("X Position (m)")
    plot(t_vec, rel_p_body(1, :), 'k')

    subplot(3, 1, 2)
    hold on;
    grid on;
    title("Y Position vs Time")
    xlabel("Time (s)")
    ylabel("Y Position (m)")
    plot(t_vec, rel_p_body(2, :), 'k')

    subplot(3, 1, 3)
    hold on;
    grid on;
    title("Z Position vs Time")
    xlabel("Time (s)")
    ylabel("Z Position (m)")
    plot(t_vec, rel_p_body(3, :), 'k')

end
