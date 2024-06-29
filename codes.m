% System parameters   all of this not only second line
A = 0.8;  % System matrix it is all bullshit
B = 1.0;  % Input matrix
C = 1.0;  % Output matrix
Q = 0.1;  % Process noise covariance
R = 0.1;  % Measurement noise covariance

% Simulation parameters
T = 100;  % Total time steps
x0 = 1.0;  % Initial state

% Moving Target Defense parameters
MTD_period = 20;  % Change system every 20 steps
MTD_amplitude = 0.2;  % Maximum change in A

% Attack parameters
attack_start = 40;
attack_end = 70;
attack_magnitude = 2.0;

% Controller and estimator gains
K = 0.5;  % State feedback gain
L = 0.6;  % Estimator gain

% Initialize arrays
x = zeros(1, T);
x_est = zeros(1, T);
y = zeros(1, T);
u = zeros(1, T);
A_t = zeros(1, T);

x(1) = x0;
x_est(1) = x0;

% Simulation loop
for t = 2:T
    % Moving Target Defense
    if mod(t, MTD_period) == 0
        A_t(t) = A + (2*rand - 1) * MTD_amplitude;
    else
        A_t(t) = A_t(t-1);
    end
    
    % Control input
    u(t) = -K * x_est(t-1);
    
    % FDI Attack
    if t >= attack_start && t < attack_end
        u(t) = u(t) + attack_magnitude;
    end
    
    % System update
    x(t) = A_t(t) * x(t-1) + B * u(t) + sqrt(Q) * randn;
    
    % Measurement
    y(t) = C * x(t) + sqrt(R) * randn;
    
    % State estimation
    x_est(t) = A_t(t) * x_est(t-1) + B * u(t) + L * (y(t) - C * (A_t(t) * x_est(t-1) + B * u(t)));
end

% Plotting results
figure;

subplot(3,1,1);
plot(1:T, x, 'b', 1:T, x_est, 'r--');
legend('True State', 'Estimated State');
title('System State');
xlabel('Time Step');
ylabel('State');

subplot(3,1,2);
plot(1:T, u);
title('Control Input');
xlabel('Time Step');
ylabel('Input');

subplot(3,1,3);
plot(1:T, A_t);
title('System Parameter A (with MTD)');
xlabel('Time Step');
ylabel('A');

% Adjust the layout
set(gcf, 'Position', [100, 100, 800, 600]);