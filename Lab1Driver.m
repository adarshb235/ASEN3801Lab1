clc;
clear;
close all;

[rho, a, temp, press, kvisc, ZorH] = stdatmo(1655);

Cd = 0.6;
d = 0.02;
A = pi * d^2 / 4;
m = 0.05;
g = 9.81;
const = [Cd d A m g];

v_i = [0; 20; -20];
p_i = [0; 0; 0];
x_i = [p_i; v_i];
wind_vel = [0; 0; 0];


options = odeset('Events', @hitGround, 'RelTol', 1e-8, 'AbsTol', 1e-8);
[t_final, x_final] = ode45(@(t, x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel), [0 100], x_i, options);

figure;
plot3(x_final(:,1), x_final(:, 2), x_final(:, 3));
set(gca, 'ZDir', 'reverse'); 
xlabel('North'); ylabel('East'); zlabel('Down');
grid on;

%% SECTION INTENDED TO TEST BEHAVIOR IN THE WIND

wind_vel_test = 0:3:60;
wind_vel_tests = [wind_vel_test' zeros(length(wind_vel_test), 1) zeros(length(wind_vel_test), 1)];
wind_vel_tests = transpose(wind_vel_tests);

resultsD = cell(length(wind_vel_test), 1);

figure;
hold on;
grid on;
box on;
ylabel('North Position')
xlabel('East Position')
for i = 1:length(wind_vel_test)
    [t_final_1, x_final_1] = ode45(@(t, x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel_tests(:, i)), [0 100], x_i, options);
    resultsD{i}.time = t_final_1;
    resultsD{i}.windspeed = wind_vel_test(i);
    resultsD{i}.x = x_final_1;
    deflection(i) = (x_final(end, 1) - resultsD{i}.x(end, 1));
    totalDist(i) = norm(x_final_1(end, 1:3) - p_i');
    names(i) = "X Wind Vel" + wind_vel_test(i);
    plot(x_final_1(:, 2), x_final_1(:, 1));
end
legend(names);
hold off;
grid off;
box off;

figure;
hold on;
plot(wind_vel_test, totalDist);
grid on;
box on;

figure;
plot(resultsD{1}.time(:), resultsD{1}.x(:, 1));

figure;
hold on;
for j = 1:length(wind_vel_test)
    plot3(resultsD{j}.x(:, 1), resultsD{j}.x(:, 2), resultsD{j}.x(:, 3))
end
view(3)
set(gca, 'ZDir', 'reverse'); 
xlabel('North'); ylabel('East'); zlabel('Down');
grid on;
box on;

altVec = 0:200:1800;

resultsE = cell(length(wind_vel_test), length(altVec));
distanceE = zeros(length(wind_vel_test), length(altVec));

figure
hold on;
for k = 1:length(altVec)
    [rho, a, temp, press, kvisc, ZorH] = stdatmo(altVec(k));
    for i = 1:length(wind_vel_test)
        [t_2, x_2] = ode45(@(t, x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel_tests(:, i)), [0 100], x_i, options);
        resultsE{i, k}.x = x_2;
        resultsE{i, k}.t = t_2;
        resultsE{i, k}.windspeed = wind_vel_test(i);
        resultsE{i, k}.distance = norm(x_2(end, 1:3) - p_i');
        distanceE(i, k) = norm(x_2(end, 1:3) - p_i');
        plot3(x_2(:, 1), x_2(:, 2), x_2(:, 3));
    end
end
view(3)
grid on;
set(gca, 'ZDir', 'reverse'); 
xlabel('North'); ylabel('East'); zlabel('Down');
grid on;
box on;


figure;
hold on;
plot(wind_vel_test, distanceE);
xlabel("Wind Speed (m/s)")
ylabel("Distance Travelled (m)")
title("Distance @ Land vs Cross Wind Speed")
legend(string(altVec) + " m/s", 'Location', 'best')
grid on;
box on;



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

