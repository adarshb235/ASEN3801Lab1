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
x_i = [0; 0; 0];
x_i = [x_i; v_i];
wind_vel = [0; 0; 0];


options = odeset('Events', @hitGround, 'RelTol', 1e-8, 'AbsTol', 1e-8);
[t_final, x_final] = ode45(@(t, x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel), [0 100], x_i, options);

figure;
plot3(x_final(:,1), x_final(:, 2), x_final(:, 3));
set(gca, 'ZDir', 'reverse'); 
xlabel('North'); ylabel('East'); zlabel('Down');
grid on;

%% SECTION INTENDED TO TEST BEHAVIOR IN THE WIND

wind_vel_test = -16:4:16;
wind_vel_tests = [wind_vel_test' zeros(length(wind_vel_test), 1) zeros(length(wind_vel_test), 1)];
wind_vel_tests = transpose(wind_vel_tests);

results = cell(length(wind_vel_test), 1);

figure;
hold on;
grid on;
box on;
ylabel('North Position')
xlabel('East Position')
for i = 1:length(wind_vel_test)
    [t_final_1, x_final_1] = ode45(@(t, x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel_tests(:, i)), [0 100], x_i, options);
    results{i}.time = t_final_1;
    results{i}.windspeed = wind_vel_test(i);
    results{i}.x = x_final_1;
    deflection = (x_final(end, 1) - results{i}.x(end, 1));
    results{i}.deflPer = deflection ./ wind_vel_test(i);
    totalDist = norm(x_final_1(end, 1:3));
    results{i}.dist = abs(totalDist ./ wind_vel_test(i)); 
    names(i) = "X Wind Vel" + wind_vel_test(i);
    plot(x_final_1(:, 2), x_final_1(:, 1));
end
legend(names);
hold off;
grid off;
box off;

figure;
hold on;
for j = 1:length(wind_vel_test)
    plot3(results{j}.x(:, 1), results{j}.x(:, 2), results{j}.x(:, 3))
end
view(3)
set(gca, 'ZDir', 'reverse'); 
xlabel('North'); ylabel('East'); zlabel('Down');
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

