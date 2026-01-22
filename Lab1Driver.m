clc; % reset 
clear;
close all;

[rho, a, temp, press, kvisc, ZorH] = stdatmo(1655); % std atmosphere call for original

Cd = 0.6; % define constants
d = 0.02;
A = pi * d^2 / 4;
m = 0.05;
g = 9.81;
const = [Cd d A m g];

v_i = [0; 20; -20]; % initial velocity trajectory
p_i = [0; 0; 0]; % original position
x_i = [p_i; v_i]; % initial state vec
wind_vel = [0; 0; 0]; % 0 wind vector


options = odeset('Events', @hitGround, 'RelTol', 1e-8, 'AbsTol', 1e-8); % event function setup
[t_final, x_final] = ode45(@(t, x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel), [0 100], x_i, options); % original ode call

figure(1); % original trajectory plot
hold on;
grid on;
box on;
title('Problem 2.c: Initial Test Trajectory')
plot3(x_final(:,1), x_final(:, 2), x_final(:, 3), 'LineWidth', 1.5);
set(gca, 'ZDir', 'reverse'); % flip for plot readability
view(3)
xlabel('North'); ylabel('East'); zlabel('Down');


%% SECTION INTENDED TO TEST BEHAVIOR IN THE WIND

wind_vel_list = 0:5:60; % create vector of tested wind speeds
wind_vel_mat = [wind_vel_list' zeros(length(wind_vel_list), 1) zeros(length(wind_vel_list), 1)]; % create matrix with 0 in y and z directions
wind_vel_mat = transpose(wind_vel_mat); 

resultsD = cell(length(wind_vel_list), 1);

figure(2); % top down view of trajectory deflection. NOT REQUIRED FOR SUBMISSION BUT IT IS USEFUL FOR UNDERSTANDING
hold on;
grid on;
box on;
title('Problem 2.d.12: Top Down View of Trajectory Deflection due to Cross Wind')
ylabel('North Position')
xlabel('East Position')
for i = 1:length(wind_vel_list) % iterate ode call for each wind value
    [t_final_1, x_final_1] = ode45(@(t, x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel_mat(:, i)), [0 100], x_i, options);
    resultsD{i}.t = t_final_1; % sort into struct
    resultsD{i}.windspeed = wind_vel_list(i);
    resultsD{i}.x = x_final_1;
    totalDist(i) = norm(x_final_1(end, 1:3) - p_i'); % calculate total distance
    xDist(i) = x_final_1(end, 1) - p_i(1); % calculate x distance
    plot(x_final_1(:, 2), x_final_1(:, 1), 'LineWidth', 1.5); % plot
end
legend(string(wind_vel_list) + " m/s (x)", 'Location', 'Best');

figure(3) % x displacement plot
hold on;
grid on;
box on;
plot(wind_vel_list, xDist, 'LineWidth', 1.3);
title('Problem 2.d.1: Origin to Landing X Displacement vs Crosswind Speed')
xlabel('Wind Speed (m/s)');
ylabel('X Displacement (m)')

figure(4); % total displacement plot
hold on;
grid on;
box on;
plot(wind_vel_list, totalDist, 'LineWidth', 1.3);
title('Problem 2.d.2: Origin to Landing Total Displacement vs Crosswind Speed');
xlabel('Wind Speed (m/s)');
ylabel('Displacement (m)')


figure(5); % NOT REQUIRED PLOT. FUN VISUAL 
hold on;
title('3D Trajectory of Crosswind Tests')
for j = 1:length(wind_vel_list)
    plot3(resultsD{j}.x(:, 1), resultsD{j}.x(:, 2), resultsD{j}.x(:, 3), 'LineWidth', 1.3)
end
view(3)
set(gca, 'ZDir', 'reverse'); 
xlabel('North'); ylabel('East'); zlabel('Down');
legend(string(wind_vel_list) + " m/s")
grid on;
box on;

altVec = 0:300:1800; % create vector of altitudes to test

resultsE = cell(length(wind_vel_list), length(altVec));
distanceE = zeros(length(wind_vel_list), length(altVec));


for k = 1:length(altVec)
    [rho, ~, ~, ~, ~, ~] = stdatmo(altVec(k)); % std atm call for each alt, only need air dens
    for i = 1:length(wind_vel_list) % iterate nested loop to get ode for each pair of alt and wind
        [t_2, x_2] = ode45(@(t, x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel_mat(:, i)), [0 100], x_i, options);
        resultsE{i, k}.x = x_2; % sort into struct
        resultsE{i, k}.t = t_2;
        resultsE{i, k}.windspeed = wind_vel_list(i);
        resultsE{i, k}.distance = norm(x_2(end, 1:3) - p_i');
        distanceE(i, k) = norm(x_2(end, 1:3) - p_i'); % save in mat
    end
end



figure(6); % plot for distance travelled vs wind
hold on;
plot(wind_vel_list, distanceE, 'LineWidth', 1.3);
xlabel("Wind Speed (m/s)")
ylabel("Distance Travelled (m)")
title("Problem 2.e.1: Distance @ Land vs Cross Wind Speed")
legend(string(altVec) + " m altitude", 'Location', 'best')
grid on;
box on;



figure(7); % plot for min distance travelled vs alt
hold on;
grid on;
box on;
minDist = min(distanceE); % find min along each column to get min for each alt
plot(altVec, minDist, 'o-', 'LineWidth', 1.3)
title('Problem 2.e.2: Minimum Origin to Landing Distance Travelled vs Altitude')
xlabel('Altitude (m)')
ylabel('Distance Travelled')

[rho, a, temp, press, kvisc, ZorH] = stdatmo(1655); % reset conditions back to original

kinEn = 0.5 * m * norm(v_i)^2; % fix kinetic energy
normaV = v_i ./ norm(v_i); % unit vector for trajectory
m_list = 0.005:0.005:0.1; % list of masses to test
mag_v_list = sqrt(kinEn * 2 ./ m_list); % find corresponding velocity mag
v_list = normaV * mag_v_list; % get final initial velocity vectors to test (in a matrix)

resultsF = cell(length(wind_vel_list), length(m_list));
distanceF = zeros(length(wind_vel_list), length(m_list));

for i = 1:length(v_list) % iterate ode for each mass and each wind
    for j = 1:length(wind_vel_list)
        x_i_2 = [p_i; v_list(:, i)]; % update initial state vector with new initial vel
        [t_3, x_3] = ode45(@(t, x) objectEOM(t,x,rho,Cd,A,m_list(i),g,wind_vel_mat(:, j)), [0 100], x_i_2, options);
        resultsF{j, i}.x = x_3; % sort into struct
        resultsF{j, i}.t = t_3;
        resultsF{j, i}.windspeed = wind_vel_list(j);
        resultsF{j, i}.initVel = v_list(:, i);
        distanceF(j, i) = norm(x_3(end, 1:3) - p_i');
    end
end

figure; % plot for minimum wind and distance
hold on;
grid on;
box on;
plot(m_list, distanceF(1, :), 'LineWidth', 1.3);
xlabel('Mass (kg)')
ylabel('Distance Travelled (m)')
title('Problem 2.f.1: Origin to Landing Distance vs Mass (Min Wind)')
legend(string(wind_vel_list(1)) + " m/s")


figure; % plot for max wind and distance
hold on;
grid on;
box on;
plot(m_list, distanceF(end, :), 'LineWidth', 1.3);
xlabel('Mass (kg)')
ylabel('Distance Travelled (m)')
title('Problem 2.f.2: Origin to Landing Distance vs Mass (Max Wind)')
legend(string(wind_vel_list(end)) + " m/s")

figure; % plot for distance vs mass for all winds
hold on;
grid on;
box on;
plot(m_list, distanceF', 'LineWidth', 1.3);
xlabel('Mass (kg)')
ylabel('Distance Travelled (m)')
title('Problem 2.f.3: Origin to Landing Distance vs Mass (All Winds)')
legend(string(wind_vel_list) + " m/s", 'Location', 'Best')




%% Event function

function [value, isterminal, direction] = hitGround(t, x)
    value = x(3);          % The condition we are tracking (z-position = 0)
    isterminal = 1;        % Halt integration when condition is met
    direction = 1;         % Only trigger when z is increasing (moving toward/through the ground)
end

%% ODE function

function xdot = objectEOM(t,x,rho,Cd,A,m,g,wind_vel)
    
    relative_vel = x(4:6) - wind_vel;
    force_drag = -0.5 * Cd * A * rho * relative_vel .* norm(relative_vel);
    F_z = force_drag(3) + m * g;
    accel_z = F_z / m;
    accel_x = force_drag(1) / m;
    accel_y = force_drag(2) / m;
    
    xdot = [x(4) x(5) x(6) accel_x accel_y accel_z]';

end

