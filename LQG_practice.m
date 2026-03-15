%% Cart-Pole LQG Control Simulation
% Linearized model at upright equilibrium
% State: x = [p, p_dot, theta, theta_dot]
% Control: u = force on cart (N)

%% System Parameters
M = 1.0;   % Cart mass (kg)
m = 0.1;   % Pole mass (kg)
l = 0.5;   % Pole length (m)
g = 9.81;  % Gravitational acceleration (m/s^2)

%% State-Space Matrices (linearized at theta=0)
A = [0,  1,              0,   0;
     0,  0,        -(m*g)/M,  0;
     0,  0,              0,   1;
     0,  0,  (M+m)*g/(M*l),   0];

B = [0; 1/M; 0; -1/(M*l)];

C = [1, 0, 0, 0;   % Measure cart position p
     0, 0, 1, 0];  % Measure pole angle theta

D = zeros(2,1);    % No direct feedthrough

%% LQR Design
% Q: state cost matrix (higher weight on theta for tight angle control)
% R: control cost (smaller = more aggressive input allowed)
Q = diag([1, 1, 10, 1]);
R = 0.1;

K = lqr(A, B, Q, R);          % Solve DARE, get optimal gain K
poles = eig(A - B*K);          % Closed-loop poles (should be stable)

%% Kalman Filter Design
% Qn: process noise covariance
% Rn: measurement noise covariance
Qn = diag([10, 200, 10, 200]);
Rn = diag([0.01, 0.01]);

P = care(A', C', Qn, Rn);     % Solve dual DARE, get error covariance P
L = P * C' / Rn;               % Kalman gain L = P*C'*Rn^{-1}
obs_poles = eig(A - L*C);      % Observer poles (should be faster than controller)

disp('Controller poles:'); disp(poles);
disp('Observer poles:');   disp(obs_poles);

%% Simulation Setup
dt = 0.01;   % Time step (s)
T  = 10;     % Total simulation time (s)
t  = 0:dt:T;
N  = length(t);

% Initial conditions
x     = [0; 0; 0.2; 0];    % True state: pole tilted 0.2 rad
x_hat = zeros(4,1);         % Estimated state: initialized to zero

% Noise intensities
q_noise = 0.001;   % Process noise standard deviation
r_noise = 0.01;    % Measurement noise standard deviation

% Storage
X     = zeros(4,N);   % True state trajectory
X_hat = zeros(4,N);   % Estimated state trajectory
U     = zeros(1,N);   % Control input trajectory

vid = VideoWriter('cart_pole_LQG.avi');
vid.FrameRate = 30;
open(vid);

%% Simulation Loop (Euler integration)
for k = 1:N

    % External disturbance: push pole at t = T/2
    if k == floor(N/2)
        x(3) = x(3) + 1;   % Impulse disturbance: +1 rad (~60 deg)
    end

    % Store current states
    X(:,k)     = x;
    X_hat(:,k) = x_hat;

    % Sensor measurement with noise
    v = r_noise * randn(2,1);
    y = C*x + v;

    % Control law: based on estimated state
    u    = -K * x_hat;
    U(k) = u;

    % Kalman observer update (prediction + correction)
    x_hat = x_hat + (A*x_hat + B*u + L*(y - C*x_hat)) * dt;

    % True system dynamics with process noise
    w = q_noise * randn(4,1);
    x = x + (A*x + B*u + w) * dt;

end

%% Animation
figure;
for k = 1:5:N
    clf;

    % Left: cart-pole animation
    subplot(1,2,1);
    p_k     = X(1,k);
    theta_k = X(3,k);

    rectangle('Position', [p_k-0.2, -0.1, 0.4, 0.2], 'FaceColor', 'b');
    hold on;
    px = p_k + l*sin(theta_k);
    py = l*cos(theta_k);
    plot([p_k, px], [0, py], 'r-', 'LineWidth', 3);
    plot(px, py, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot([-2, 2], [0, 0], 'k-', 'LineWidth', 2);
    axis([-1.5, 1.5, -0.1, 0.8]);
    title(sprintf('t = %.2f s', t(k)));
    xlabel('Position (m)'); ylabel('Height (m)');

    % Right: control input over time
    subplot(1,2,2);
    plot(t(1:k), U(1:k), 'g-', 'LineWidth', 2);
    xlim([0, T]);
    ylim([min(U)-1, max(U)+1]);
    xlabel('Time (s)'); ylabel('u (N)');
    title('Control Force u');
    grid on;

    drawnow;
    frame = getframe(gcf);
    writeVideo(vid, frame);
end

close(vid);
disp('Video saved: cart_pole_LQG.avi');