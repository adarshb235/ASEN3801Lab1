% Contributors: Ryan Hajdu, Isaac Kness
% Course number: ASEN 3801
% File name: Lab1.m 
% Created: 01/13/26

clc;
clear;
close all;

%% part 1a
% Creating n.d time step
t = [0 20];
% Creating arbitrary intial vector
X = [0.25; 0.5; 0.75; 1];

% Creating tolerences
options = odeset('RelTol',1*10^-12 ,'AbsTol',1*10^-12);

% Calling ode45
[t, X2] = ode45(@ode, t, X, options);

% Plotting on one figure with 4 subplots
hold on;
subplot(4,1,1)
plot(t,X2(:,1));
title('State Vectors vs Time');
ylabel('w (n.d)');
subplot(4,1,2)
plot(t,X2(:,2));
ylabel('x (n.d)');
subplot(4,1,3)
plot(t,X2(:,3));
ylabel('y (n.d)');
subplot(4,1,4)
plot(t,X2(:,4));
ylabel('z (n.d)');
xlabel('Time (n.d)')
hold off;

%% Part 1b

% Setting each tolerence, time steps, and intial state vector
tol = [1*10^-2 1*10^-4 1*10^-6 1*10^-8 1*10^-10 1*10^-12];
t1 = [0 20];
X3 = [0.25; 0.5; 0.75; 1];
final_states = zeros(4,length(tol));

% Looping through each tolerence and retrieving the state vector for each tolerence
for i = 1:length(tol)
    options1 = odeset('RelTol',tol(i) ,'AbsTol',tol(i));
    [~,X4] = ode45(@ode,t1,X3,options1);
    final_states(:,i) = X4(end,:)';
end

% storeing the final state vector computed with the tightest tolerance 
% 1×10^-12 as the reference solution for error comparisons.
X_ref = final_states(:,end);

%computeing the absolute difference between each final state at the tested 
% tolerances and the reference solution at 1×10^-12, giving the numerical 
% error for each state variable.
diff = abs(final_states(:,1:end-1) - X_ref);

% converting each numeric error value in diff into a scientific-notation 
% string with 12 digits so it can be displayed correctly in a table figure
formatted_data = arrayfun(@(x) sprintf('%.12e', x),diff, 'UniformOutput', false);

% Create table
figure
uitable('Data', formatted_data,'ColumnName', {'1e-2','1e-4','1e-6','1e-8','1e-10'},'RowName', {'|w(t=20)-w_R(t=20)|', '|x(t=20)-x_R(t=20)|', '|y(t=20)-y_R(t=20)|', '|z(t=20)-z_R(t=20)|'},'Units','normalized', 'Position',[0 0 1 1]);
title('Absolute Error at t = 20')

%% Functions

% Creating function to be input into ode45
function vector = ode(t, X)

%
% Inputs: X = w, x, y, and z. t = n.d time steps
%
%
% Outputs: vector = state vector at each time step
% Methodology: Create function to plug into ode45 by using defintions
% given for dynamical systems 

% defining variables within intial state vector 
w = X(1);
x = X(2);
y = X(3);
z = X(4);

% creating dynamical system governing equations
wdot = -9 * w + y;
xdot = 4 * w * x * y - x^2;
ydot = 2 * w - x  - 2 * z;
zdot = x * y - y^2 - 3 * z^3;
vector = [wdot; xdot; ydot; zdot];
end
