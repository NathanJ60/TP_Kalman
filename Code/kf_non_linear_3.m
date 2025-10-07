clear all;

% Load 'dt', 'groundtruth', 'labels', 'obsdist', 'obsangl' and 'obsvel'
load nonlinear.mat;
N = size(groundtruth, 1);
M = size(obsdist, 1);
T = size(groundtruth, 2);
livedisplay = false;

% TODO: Initial filter state
X = ...; % (N*1)
P = ...; % (N*N)

% A is not defined here but in the loop as it has to be re-evaluted at each
% iterations now

% TODO: Covariance of the evolution noise.
Q = ...; % (N*N)

% TODO: Velocity observation model and covariance of its noise
Cv = ...; % (Mv*N)
Rv = ...; % (Mv*Mv)

% Cdist and Cangl are not defined here but in the loop as they have to be
% recomputed at each iterations now
% TODO: Covariances of the distance and angle noises.
Rdist = ...; % (M*M)
Rangl = ...; % (M*M)

% Memory for display after filtering
Xs = zeros(N, T);
Ps = zeros(N, N, T);
I = eye(N);

for t = 1:T
    % Update
    C = Gdist(X, beacons);
    K = P * C' * pinv(C * P * C' + Rdist);
    X = X + K * (obsdist(:, t) - gdist(X, beacons));
    P = (I - K * C) * P;

    yangls = gangl(X, beacons);
    for b = 1:M
        yangls(b) = normalize_angle_diff(yangls(b), obsangl(b, t));
    end
    C = Gangl(X, beacons);
    K = P * C' * pinv(C * P * C' + Rangl);
    X = X + K * (obsangl(:, t) - yangls);
    P = (I - K * C) * P;

    K = P * Cv' * pinv(Cv * P * Cv' + Rv);
    X = X + K * (obsvel(:, t) - Cv * X);
    P = (I - K * Cv) * P;

    % Storage
    Xs(:, t) = X;
    X(3) = normalize_angle_diff(X(3), groundtruth(3, t));
    Ps(:, :, t) = P;

    % Prediction
    A = F(X, dt);
    X = f(X, dt);
    P = A * P * A' + Q;

    % Live display
    if livedisplay
      figure(3);
      clf;
      hold on;
      plot(groundtruth(1, 1:T), groundtruth(2, 1:T), 'b', 'DisplayName', 'Estimate', Xs(1, 1:t), Xs(2, 1:t), 'r' , 'DisplayName', 'History', X(1), X(2), 1, 'k', 'DisplayName', 'Groundtruth');
      displayCov(X(1:2), P(1:2,1:2), 0.997, 'b');
      legend;
      axis equal;
      xlabel(num2str(t));
      hold off;
      pause(0.001);
    end
end

figure(1);
title('Trajectory and estimate')
hold on;
plot(Xs(1, 1:T), Xs(2, 1:T), 'b', 'DisplayName', 'Estimate');
plot(groundtruth(1, 1:T), groundtruth(2, 1:T), 'k', 'DisplayName', 'Groundtruth');
legend;
axis equal;
hold off;

% Error display with +/- estimated standard deviations
errors = Xs(:, 1:T) - groundtruth(:, 1:T);
stddevs = squeeze([sqrt(Ps(1, 1, 1:T)); sqrt(Ps(2, 2, 1:T)); sqrt(Ps(3, 3, 1:T)); sqrt(Ps(4, 4, 1:T))]);

figure(2);
title('Estimation errors and 3 std bounds');
for k = 1:N,
    subplot(N, 1, k);
    hold on;
    plot(1:T, errors(k, :), 'b');
    plot(1:T, 3 * stddevs(k, :), 'r');
    plot(1:T, -3 * stddevs(k, :), 'r');
    zoom on;
    ylabel(labels(k));
    hold off;
end

mean_error = mean(errors');
rmse = sqrt(mean(errors'.^2));
consistency = mean(abs(errors') < (3*stddevs'));
uncertainty = mean(stddevs');

disp(['Mean error: ', num2str(mean_error)]);
disp(['Consistency: ', num2str(consistency)]);
disp(['RMSE: ', num2str(rmse)]);
disp(['Uncertainty: ', num2str(uncertainty)]);

% ==== FUNCTION DEFINITIONS ==== %
% If using octave, move these functions at the beginning of this file
function angleout = normalize_angle_diff(anglein, reference)
    % modify the angle so that its difference is as close to
    % the reference as possible (modulo 2pi)
    angleout = anglein + 2 * pi * floor((reference - anglein) / (2 * pi));
    if (reference - angleout) > pi
        angleout += 2 * pi;
    elseif (angleout - reference) > pi
        angleout -= 2 * pi;
    end
end

function X = f(X, dt)
    % TODO Return an updated X (N*1)
    ...
end

function A = F(X, dt)
    % TODO: Return the linearized evolution model (N*N)
    ...
end

function ydists = gdist(X, beacons)
    % TODO: Fill ydists (M*1) so that every row is a observation of distance to beacon b (y_rho_i)
    M = size(beacons, 2);
    ydists = zeros(M, 1);

    for b = 1:M
        ydists(b) = ...;
    end
end

function yangls = gangl(X, beacons)
    % TODO: Fill yangls (M*1) so that every row is a observation of angle to beacon b (y_theta_i)
    M = size(beacons, 2);
    yangls = zeros(M, 1);

    for b = 1:M
        yangls(b) = ...;
    end
end

function Cdists = Gdist(X, beacons)
    % TODO: Fill Gdist (M*N) so that every row is the jacobian of a distance observation to beacon b (G_rho_i)
    M = size(beacons, 2);
    Cdists = zeros(M, size(X, 1));

    for b = 1:M
        Cdists(b, :) = ...;
    end
end

function Cangls = Gangl(X, beacons)
    % TODO: Fill Gangl (M*N) so that every row is the jacobian of an angle observation to beacon b (G_theta_i)
    M = size(beacons, 2);
    Cangls = zeros(M, size(X, 1));

    for b = 1:M
        Cangls(b, :) = ...;
    end
end

% ==== END OF DEFINITIONS ==== %