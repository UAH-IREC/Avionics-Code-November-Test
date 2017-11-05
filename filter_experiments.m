clear, clc

sigma_accel = 0.1; % m/s^2
accel_mean = 1.2; % m/s^2

tstep = 1;
times = 0:tstep:200;
accel = accel_mean * ones([length(times), 1]) + sigma_accel * randn([length(times), 1]);
accel(1:50) = sigma_accel * randn([50, 1]);
velocities = zeros([length(times), 1]);
altitudes = zeros([length(times), 1]);

for i=2:length(altitudes)
    velocities(i) = velocities(i - 1) + tstep * (accel(i) + accel(i - 1)) / 2;
	altitudes(i) = altitudes(i - 1) + tstep * (velocities(i) + velocities(i - 1)) / 2;
end

% Standard deviation of velocity and position measurements from the true values
sigma_vel = 50.0; % m
sigma_accel = 3.0; % m/s

vel_measurements = velocities + sigma_vel * randn([length(velocities), 1]);
accel_measurements = accel + sigma_accel * randn([length(velocities), 1]);
alt_predictions = zeros([length(times), 2]);
vel_predictions = zeros([length(times), 1]);

xk = [0; 0]; % Initial altitude and velocity
Pk = [sigma_vel^2 0; 0 sigma_accel^2]; % Initial covariance matrix

result = zeros([length(times), 2]);
result(1, :) = xk';

Rk = [1 0; 0 1];

Hk = [1 0; 0 1];

for i=1:length(times)
	% Predict
	Fk = [1 tstep; 0 1];
	xkhat = Fk * xk;
	Qk = [1 0; 0 0.1];
	Pk = Fk * Pk * Fk' + Qk;

	% Update
	zk = [vel_measurements(i); accel_measurements(i)];
	K = Pk * Hk' * (Hk * Pk * Hk' + Rk)^-1;
	xk = xkhat + K * (zk - Hk * xkhat);
	Pk = Pk - K * Hk * Pk;

	% Store
	result(i, :) = xk';
    if i > 1
        alt_predictions(i, 1) = alt_predictions(i - 1, 1) + (result(i, 1) + result(i - 1, 1)) * tstep / 2;
        vel_predictions(i) = vel_predictions(i - 1) + (accel_measurements(i) + accel_measurements(i - 1)) * tstep / 2;
        alt_predictions(i, 2) = alt_predictions(i - 1, 2) + (vel_predictions(i) + vel_predictions(i - 1)) * tstep / 2;
    end
end

figure()
plot(times, altitudes, 'r+', times, alt_predictions(:, 1), 'b+', times, alt_predictions(:, 2), 'yo');
legend('True altitudes', 'Kalman altitude predictions', 'Raw integrated altitude');

figure()
plot(times, altitudes - alt_predictions(:, 1), times, altitudes - alt_predictions(:, 2));
legend('With Kalman filter', 'Without Kalman filter');

errors = zeros([length(times), 2]);
errors(:, 1) = altitudes - result(:, 1); % Altitude errors
errors(:, 2) = velocities - result(:, 2); % Velocity errors

avg_err = mean(abs(altitudes - result(:, 1)))
avg_vel_error = mean(abs(velocities - result(:, 2)))

std_dev_pos = std(errors(:, 1))
std_dev_vel = std(errors(:, 2))
std_dev_accel = std(abs(result(:, 2) - accel))

%quit
