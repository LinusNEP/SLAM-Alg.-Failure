%%
% ========================================================================================================================
%                               o2s: SLAM Evaluation in Environment of Different Characteristics 
% ========================================================================================================================

clc
close all
clear
%% CSV file

gtVisLongPass = 'gtVisLongPass.csv'; % GroundTruth
rtabVisLongPass = 'rtabLongPass.csv';
orbslamVisLongPass = 'orbslamLongPass.csv';

%% Read the Messages

gtVisLongPass_data = readtable(gtVisLongPass);
rtabVisLongPass_data = readtable(rtabVisLongPass);
orbslamVisLongPass_data = readtable(orbslamVisLongPass);

% Extract the timestamp column
gt_time_stamp = gtVisLongPass_data{:,1};
rtab_time_stamp = rtabVisLongPass_data{:,1};
orbslam_time_stamp = orbslamVisLongPass_data{:,1};

% convert the timestamps to real time
gt_time = ((gt_time_stamp)- (gt_time_stamp(1)));
rtab_time = ((rtab_time_stamp)- (rtab_time_stamp(1)));
orbslam_time = ((orbslam_time_stamp)- (orbslam_time_stamp(1)));


% Extract the x, y, z position columns
gt_x = gtVisLongPass_data{:,2};
gt_y = gtVisLongPass_data{:,3};
gt_z = gtVisLongPass_data{:,4};

rtab_x = rtabVisLongPass_data{:,2};
rtab_y = rtabVisLongPass_data{:,3};
rtab_z = rtabVisLongPass_data{:,4};

orbslam_x = orbslamVisLongPass_data{:,2};
orbslam_y = orbslamVisLongPass_data{:,3};
orbslam_z = orbslamVisLongPass_data{:,4};


% Extract the qx, qy, qz, qw rotation columns
gt_qz = gtVisLongPass_data{:,7};
rtab_qz = rtabVisLongPass_data{:,7};
orbslam_qz = orbslamVisLongPass_data{:,7};

%%

gt_translation_ts = [gt_time, (gt_x ), (gt_y), gt_z];
rtab_pose_ts = [rtab_time, rtab_x, rtab_y, rtab_z];
orbslam_pose_ts = [orbslam_time, orbslam_x-0.0872665, orbslam_y-0.0872665, orbslam_z-0.0872665];

gt_rotation_ts = [gt_qz];
rtab_orient_ts = [rtab_qz];
orbslam_orient_ts = [orbslam_qz];

%% Plot the trajectories in 2D

plot(gt_translation_ts(:,2),gt_translation_ts(:,3),"r","LineWidth",1.0)
hold on
plot(rtab_pose_ts(:,2),rtab_pose_ts(:,3),"b","LineWidth",1.0)
hold on
plot(orbslam_pose_ts(:,2), orbslam_pose_ts(:,3),"k","LineWidth",1.0)


xlabel('x-pose [m]')
ylabel('y-pose [m]')
%zlabel('z-pose [m]')
legend("GT","RTAB","ORB-SLAM2")


%% Plot the topic varaiables

subplot(2, 2, 1)
plot(gt_time,gt_translation_ts(:,2), gt_time,gt_translation_ts(:,3), gt_time,gt_rotation_ts(:,1))
%title('gt pose')
xlabel('Time [seconds]')
ylabel('gt pose')
legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
%xlim([0 500])
grid on

subplot(2, 2 ,2)
plot(rtab_time,rtab_pose_ts(:,2), rtab_time,rtab_pose_ts(:,3), rtab_time,rtab_orient_ts(:,1))
%title('rtab estimated pose')
xlabel('Time [seconds]')
ylabel('rtab pose')
legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
grid on

subplot(2, 2 ,3)
plot(orbslam_time,orbslam_pose_ts(:,2), orbslam_time,orbslam_pose_ts(:,3), orbslam_time,orbslam_orient_ts(:,1))
%title('orbslamping estimated pose')
xlabel('Time [seconds]')
ylabel('orbslam pose')
legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
grid on


%% Generate table for all the values

%tbl = table(t_amcl_x,t_amcl_y,t_amcl_z, resize_odom_x, resize_odom_y, resize_odom_z);   % Combine them in a table
%


%% Compute the ATE and RPE

% Extract the data and timestamps from the timeseries object
ground_truth_x = gt_translation_ts(:,2);
ground_truth_y = gt_translation_ts(:,3);
gt_estimated_z = gt_rotation_ts(:,1);

% RTAB
rtab_estimated_x = rtab_pose_ts(:,2);
rtab_estimated_y = rtab_pose_ts(:,3);
rtab_estimated_z = rtab_orient_ts(:,1);

% ORB-SLAM 
orbslam_estimated_x = orbslam_pose_ts(:,2);
orbslam_estimated_y = orbslam_pose_ts(:,3);
orbslam_estimated_z = orbslam_orient_ts(:,1);


%% Interpolate the estimated data

% GT
interp_gt_x = imresize(ground_truth_x,[11355,1]);
interp_gt_y = imresize(ground_truth_y,[11355,1]);
interp_gt_theta = imresize(gt_estimated_z,[11355,1]);
interp_gt_time = imresize(gt_time,[11355,1]);

% RTAB
interp_rtab_x = imresize(rtab_estimated_x,[11355,1]);
interp_rtab_y = imresize(rtab_estimated_y,[11355,1]);
interp_rtab_z = imresize(rtab_estimated_z,[11355,1]);
interp_rtab_time = imresize(rtab_time,[11355,1]);

% ORB-SLAM
interp_orbslam_x = imresize(orbslam_estimated_x,[11355,1]);
interp_orbslam_y = imresize(orbslam_estimated_y,[11355,1]);
interp_orbslam_z = imresize(orbslam_estimated_z,[11355,1]);
interp_orbslam_time = imresize(orbslam_time,[11355,1]);


% Compute the error between ground truth and interpolated estimated poses

% RTAB
errors_rtab_x = interp_rtab_x - interp_gt_x;
errors_rtab_y = interp_rtab_y - interp_gt_y;
errors_rtab_z = interp_gt_theta - interp_rtab_z;

% ORB-SLAM
errors_orbslam_x = interp_gt_x - interp_orbslam_x;
errors_orbslam_y = interp_gt_y - interp_orbslam_y;
errors_orbslam_z = interp_gt_theta - interp_orbslam_z;

% Compute the absolute trajectory error
RTAB_ate = sqrt(errors_rtab_x.^2 + errors_rtab_y.^2);
orbslam_ate = sqrt(errors_orbslam_x.^2 + errors_orbslam_y.^2);


% Create an array of ate values for multiple runs of the slam algorithm
ate_values_rtab = [abs(RTAB_ate)/2];
ate_values_orbslam = [abs(orbslam_ate)/2];

% Set the bin edges for the histogram
bins_rtab = linspace(min(ate_values_rtab(:)), max(ate_values_rtab(:)), 40);
bins_orbslam = linspace(min(ate_values_orbslam(:)), max(ate_values_orbslam(:)), 40);

% Plot the histogram
figure;
histogram(ate_values_rtab, bins_rtab, 'Normalization', 'probability', 'FaceColor', 'r','FaceAlpha',1.0);
hold on
histogram(ate_values_orbslam, bins_orbslam, 'Normalization', 'probability', 'FaceColor', 'b', 'EdgeAlpha', 0.1);

legend('RTAB', 'ORB-SLAM');
ax = gca; % Get the current axes
ax.XAxis.LineWidth = 2; % Set the x axis line width
ax.YAxis.LineWidth = 2; % Set the y axis line width
ax.FontSize = 14; % Set the font size for the axis numbers
xlabel('ATE Error (m)');
ylabel('Probability');
%title('ATE Error Histograms');
set(gca, 'FontWeight', 'bold'); % sets the font weight of the current axes
%set(gca, 'XTickLabel', get(gca, 'XTick'), 'FontWeight', 'bold'); % sets the font weight of the x tick labels
%set(gca, 'YTickLabel', get(gca, 'YTick'), 'FontWeight', 'bold'); % sets the font weight of the y tick labels

xline(mean(RTAB_ate),'--r', '\mu','HandleVisibility','off');
xline(mean(orbslam_ate),'--b', '\mu','HandleVisibility','off');
hold off;


% Plot the ATE against timestamp
figure;
plot(abs(interp_gt_time), abs(RTAB_ate), 'r');
hold on
plot(abs(interp_gt_time), abs(orbslam_ate));

xlabel('Time (s)');
ylabel('ATE (m)');
legend('RTAB', 'ORB-SLAM')

yline(mean(RTAB_ate),'--g', 'meanRTAB','HandleVisibility','off');
yline(mean(orbslam_ate),'--m', 'meanORB','HandleVisibility','off');



%% Compute RPE

% Compute the translation error
%RTAB
x_trans_rtab = (interp_gt_x - interp_rtab_x).^2;
y_trans_rtab = (interp_gt_y - interp_rtab_y).^2;
trans_error_rtab = sqrt(x_trans_rtab + y_trans_rtab);

%ORB-SLAM
x_trans_orbslam = (interp_gt_x - interp_orbslam_x).^2;
y_trans_orbslam = (interp_gt_y - interp_orbslam_y).^2;
trans_error_orbslam = sqrt(x_trans_orbslam + y_trans_orbslam);

% Compute the angular error
%RTAB
ang_error_rtab = abs(interp_gt_theta - interp_rtab_z);
ang_error_rtab = min(ang_error_rtab, 2*pi - ang_error_rtab);

%ORB-SLAM
ang_error_orbslam = abs(interp_gt_theta - interp_orbslam_z);
ang_error_orbslam = min(ang_error_orbslam, 2*pi - ang_error_orbslam);

% Compute the relative pose error
%rtab
rpe_trans_rtab = trans_error_rtab ./ sqrt(interp_gt_x.^2 + interp_gt_y.^2);
rpe_ang_rtab = ang_error_rtab ./ (2*pi);

% orbslam
rpe_trans_orbslam = trans_error_orbslam ./ sqrt(interp_gt_x.^2 + interp_gt_y.^2);
rpe_ang_orbslam = ang_error_orbslam ./ (2*pi);

% display the RPE error as scalar
%rtab
rtab_RPE_mean = mean(rpe_trans_rtab);
fprintf('rtab_RPE_mean: %f\n', rtab_RPE_mean);

%orbslam
orbslam_RPE_mean = mean(rpe_trans_orbslam);
fprintf('orbslam_RPE_mean: %f\n', orbslam_RPE_mean);


% Plot the RPE
figure;
subplot(2,1,1);
plot(abs(interp_gt_time), abs(rpe_trans_rtab));
hold on
plot(abs(interp_gt_time), abs(rpe_trans_orbslam));
legend("rtab","orbslam")
xlabel('Time (s)');
ylabel('Translational RPE ');


subplot(2,1,2);
plot(abs(interp_rtab_time),abs(rpe_ang_rtab));
hold on 
plot(abs(interp_rtab_time), abs(rpe_ang_orbslam));
legend("rtab","orbslam")

xlabel('Time (s)');
ylabel('Angular RPE (rad)');


%% Compute RMSE

% Compute the mean squared error
%rtab
x_mse_trans_rtab = (interp_gt_x - interp_rtab_x).^2 ;
y_mse_trans_rtab = (interp_gt_y - interp_rtab_y).^2;
mse_trans_rtab = mean(x_mse_trans_rtab + y_mse_trans_rtab);
mse_ang_rtab = mean(ang_error_rtab.^2);
% Compute the root mean squared error
rmse_trans_rtab = sqrt(mse_trans_rtab);
rmse_ang_rtab = sqrt(mse_ang_rtab);


%orbslam
x_mse_trans_orbslam = (interp_gt_x - interp_orbslam_x).^2 ;
y_mse_trans_orbslam = (interp_gt_y - interp_orbslam_y).^2;
mse_trans_orbslam = mean(x_mse_trans_orbslam + y_mse_trans_orbslam);
mse_ang_orbslam = mean(ang_error_orbslam.^2);
% Compute the root mean squared error
rmse_trans_orbslam = sqrt(mse_trans_orbslam);
rmse_ang_orbslam = sqrt(mse_ang_orbslam);


% Plot the RMSE
%{
figure;
subplot(2,1,1);
plot(rtab_time, log10(rmse_trans));
xlabel('Time (s)');
ylabel('Translational RMSE (m)');
subplot(2,1,2);
plot(rtab_time, log10(rmse_ang));
xlabel('Time (s)');
ylabel('Angular RMSE (rad)');

%}

%% Compute the scale drift

% Compute the cumulative distance traveled by the ground truth trajectory
gt_cumulative_distance = cumsum(sqrt(diff(interp_gt_x).^2 + diff(interp_gt_y).^2));

% Compute the cumulative distance traveled by the rtab trajectory
rtab_cumulative_distance = cumsum(sqrt(diff(interp_rtab_x).^2 + diff(interp_rtab_y).^2));

% Compute the cumulative distance traveled by the orbslam trajectory
orbslam_cumulative_distance = cumsum(sqrt(diff(interp_orbslam_x).^2 + diff(interp_orbslam_y).^2));


% Compute the scale drift in percentage
scale_drift_rtab = (rtab_cumulative_distance ./ gt_cumulative_distance);
scale_drift_orbslam = (orbslam_cumulative_distance ./ gt_cumulative_distance);

%Mean Scale Drift

mean_rtab_scale_drift = std(scale_drift_rtab)
mean_orbslam_scale_drift = std(scale_drift_orbslam)

% Plot the scale drift
drift_time_rtab = imresize(rtab_time,[11354,1]);
drift_time_orbslam = imresize(orbslam_time,[11354,1]);

figure;
plot(drift_time_rtab, scale_drift_rtab);
hold on
plot(drift_time_rtab, scale_drift_orbslam);

xlabel('Time (s)');
ylabel('Scale Drift');
legend('rtab','orbslam')


%% Plot the translation error against the distance traveled by the robot:

% Compute the distance traveled by the robot
distance = cumsum(sqrt(diff(interp_gt_x).^2 + diff(interp_gt_y).^2));

% Compute the translation error
%rtab
error_rtab = sqrt((interp_rtab_x - interp_gt_x).^2 + (interp_rtab_y - interp_gt_y).^2);
% orbslam
error_orbslam = sqrt((interp_orbslam_x - interp_gt_x).^2 + (interp_orbslam_y - interp_gt_y).^2);

% Interpolate the error to match the distance traveled
interp_error_rtab = imresize(error_rtab,[11354,1]);
interp_error_orbslam = imresize(error_orbslam,[11354,1]);


% Plot the translation error against the distance traveled
figure;
plot(distance, interp_error_rtab);
hold on
plot(distance, interp_error_orbslam);

xlabel('Distance traveled (m)');
ylabel('Translation error (m)');
%title('Translation error vs distance traveled');% Compute the distance traveled by the robot
legend('rtab','orbslam')


%% Average translation error in percentage

% Compute the average error
average_error_rtab = mean(interp_error_rtab);
average_error_orbslam = mean(interp_error_orbslam);


% Compute the average error in percentage
average_error_percent_rtab = 100 * average_error_rtab / mean(sqrt(interp_gt_x.^2 + interp_gt_y.^2));
average_error_percent_orbslam = 100 * average_error_orbslam / mean(sqrt(interp_gt_x.^2 + interp_gt_y.^2));
% Display the average error in percentage
disp(['Average error in percentage_rtab: ', num2str(average_error_percent_rtab), '%']);
disp(['Average error in percentage_orbslam: ', num2str(average_error_percent_orbslam), '%']);


%% Computing the time to convergence (TTC) metrics 

% Define the convergence threshold
threshold = 0.1;

% Find the index of the first time the error is below the threshold
rtab_first_convergence_index = find(RTAB_ate < threshold, 1);
orbslam_first_convergence_index = find(orbslam_ate < threshold, 1);

% Compute the time to convergence
rtab_time_to_convergence = interp_rtab_time(rtab_first_convergence_index);
orbslam_time_to_convergence = interp_orbslam_time(orbslam_first_convergence_index);

% Display the time to convergence
disp(['Time to convergence_rtab: ', num2str(rtab_time_to_convergence), 's']);
disp(['Time to convergence_orbslam: ', num2str(orbslam_time_to_convergence), 's']);


% Plot the ATE and TTC
figure;
plot(interp_gt_time, RTAB_ate, 'LineWidth', 1.0);
hold on
plot(interp_gt_time, orbslam_ate, 'LineWidth', 1.0);

hold on;
plot(rtab_time_to_convergence, RTAB_ate(rtab_first_convergence_index), '*', 'MarkerSize', 10);
hold on;
plot(orbslam_time_to_convergence, orbslam_ate(orbslam_first_convergence_index), '<', 'MarkerSize', 10);
xlabel('Time (s)');
ylabel('ATE (m)');
legend({'ATE', 'TTC'});
