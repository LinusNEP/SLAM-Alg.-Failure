%%
% ========================================================================================================================
%                               o2s: SLAM Evaluation in Environment of Different Characteristics 
% ========================================================================================================================

clc
close all
clear
%% ROS bag file

% CPS Laboratory  (Lidar)
gt_ctrlEnv = rosbag('gtctrldEnv.bag'); % GroundTruth
hector_ctrlEnv = rosbag('kartoCtrlEnv.bag');
karto_ctrlEnv = rosbag('kartoCtrlEnv.bag');
gmap_ctrlEnv = rosbag('gmapCtrlEnv.bag');
%cartog_cpsLab = rosbag('cpsCartog.bag');
%rtab_cpsLab = rosbag('cpsRTAB.bag');


%% ROS Topics

% GT
gt_tf = select(gt_ctrlEnv, 'Topic', '/tf');
gt_msgs = readMessages(gt_tf);

%Hector
hector_tf = select(hector_ctrlEnv, 'Topic', '/tf');
hector_msgs = readMessages(hector_tf);

%gmap
gmap_tf = select(karto_ctrlEnv, 'Topic', '/tf');
gmap_msgs = readMessages(gmap_tf);

%karto
karto_tf = select(gmap_ctrlEnv, 'Topic', '/tf');
karto_msgs = readMessages(karto_tf);


%% Specify the frames of interest
frame_id = 'map';
child_frame_id = 'base_link';

% Create empty arrays to store the data
%GT
gt_translation_data = [];
gt_rotation_data = [];
gt_time_data = [];

%Hector
hector_translation_data = [];
hector_rotation_data = [];
hector_time_data = [];

%Gmap
gmap_translation_data = [];
gmap_rotation_data = [];
gmap_time_data = [];

%Karto
karto_translation_data = [];
karto_rotation_data = [];
karto_time_data = [];

%% Get the transformation data
%GT
for i = 1:length(gt_msgs)
    gt_transform = gt_msgs{i}.Transforms;
    for j = 1:numel(gt_transform)
        if strcmp(gt_transform(j).ChildFrameId,'base_link') && strcmp(gt_transform(j).Header.FrameId,'map')
            % Extract the translation and rotation data from the message
            gt_translation = [gt_transform(j).Transform.Translation.X, gt_transform(j).Transform.Translation.Y, gt_transform(j).Transform.Translation.Z];
            gt_rotation = [gt_transform(j).Transform.Rotation.W, gt_transform(j).Transform.Rotation.X, gt_transform(j).Transform.Rotation.Y, gt_transform(j).Transform.Rotation.Z];
            gt_time = double(gt_transform(j).Header.Stamp.Sec) + double(gt_transform(j).Header.Stamp.Nsec)*1e-9;
            % Append the data to the arrays
            gt_translation_data = [gt_translation_data; gt_translation];
            gt_rotation_data = [gt_rotation_data; gt_rotation];
            gt_time_data = [gt_time_data; gt_time];
        end
    end
end


%Hector
for i = 1:length(hector_msgs)
    hector_transform = hector_msgs{i}.Transforms;
    for j = 1:numel(hector_transform)
        if strcmp(hector_transform(j).ChildFrameId,'base_link') && strcmp(hector_transform(j).Header.FrameId,'map')
            % Extract the translation and rotation data from the message
            hector_translation = [hector_transform(j).Transform.Translation.X, hector_transform(j).Transform.Translation.Y, hector_transform(j).Transform.Translation.Z];
            hector_rotation = [hector_transform(j).Transform.Rotation.W, hector_transform(j).Transform.Rotation.X, hector_transform(j).Transform.Rotation.Y, hector_transform(j).Transform.Rotation.Z];
            hector_time = double(hector_transform(j).Header.Stamp.Sec) + double(hector_transform(j).Header.Stamp.Nsec)*1e-9;
            % Append the data to the arrays
            hector_translation_data = [hector_translation_data; hector_translation];
            hector_rotation_data = [hector_rotation_data; hector_rotation];
            hector_time_data = [hector_time_data; hector_time];
        end
    end
end


%Gmap
for i = 1:length(gmap_msgs)
    gmap_transform = gmap_msgs{i}.Transforms;
    for j = 1:numel(gmap_transform)
        if strcmp(gmap_transform(j).ChildFrameId,'base_link') && strcmp(gmap_transform(j).Header.FrameId,'map')
            % Extract the translation and rotation data from the message
            gmap_translation = [gmap_transform(j).Transform.Translation.X, gmap_transform(j).Transform.Translation.Y, gmap_transform(j).Transform.Translation.Z];
            gmap_rotation = [gmap_transform(j).Transform.Rotation.W, gmap_transform(j).Transform.Rotation.X, gmap_transform(j).Transform.Rotation.Y, hector_transform(j).Transform.Rotation.Z];
            gmap_time = double(gmap_transform(j).Header.Stamp.Sec) + double(gmap_transform(j).Header.Stamp.Nsec)*1e-9;
            % Append the data to the arrays
            gmap_translation_data = [gmap_translation_data; gmap_translation];
            gmap_rotation_data = [gmap_rotation_data; gmap_rotation];
            gmap_time_data = [gmap_time_data; gmap_time];
        end
    end
end


%Karto
for i = 1:length(karto_msgs)
    karto_transform = karto_msgs{i}.Transforms;
    for j = 1:numel(karto_transform)
        if strcmp(karto_transform(j).ChildFrameId,'base_link') && strcmp(karto_transform(j).Header.FrameId,'map')
            % Extract the translation and rotation data from the message
            karto_translation = [karto_transform(j).Transform.Translation.X, karto_transform(j).Transform.Translation.Y, karto_transform(j).Transform.Translation.Z];
            karto_rotation = [karto_transform(j).Transform.Rotation.W, karto_transform(j).Transform.Rotation.X, karto_transform(j).Transform.Rotation.Y, hector_transform(j).Transform.Rotation.Z];
            karto_time = double(karto_transform(j).Header.Stamp.Sec) + double(karto_transform(j).Header.Stamp.Nsec)*1e-9;
            % Append the data to the arrays
            karto_translation_data = [karto_translation_data; karto_translation];
            karto_rotation_data = [karto_rotation_data; karto_rotation];
            karto_time_data = [karto_time_data; karto_time];
        end
    end
end



%% Create a timeseries object for the translation data
gt_translation_ts = timeseries(gt_translation_data+ 0.015 ,gt_time_data );
hector_translation_ts = timeseries(hector_translation_data+0.01,hector_time_data);
gmap_translation_ts = timeseries(gmap_translation_data,gmap_time_data);
karto_translation_ts = timeseries(karto_translation_data + 0.002,karto_time_data );


gt_rotation_ts = timeseries(gt_rotation_data ,gt_time_data );
hector_rotation_ts = timeseries(hector_rotation_data,hector_time_data );
gmap_rotation_ts = timeseries(gmap_rotation_data,gmap_time_data);
karto_rotation_ts = timeseries(karto_rotation_data,karto_time_data);



%% Plot the trajectories in 2D
plot(hector_translation_ts.Data(:,1),hector_translation_ts.Data(:,2),"r","LineWidth",1.0)
hold on
plot(gt_translation_ts.Data(:,1),gt_translation_ts.Data(:,2),"b","LineWidth",1.0)
hold on
plot(karto_translation_ts.Data(:,1),karto_translation_ts.Data(:,2),"g","LineWidth",1.0)
hold on
plot(gmap_translation_ts.Data(:,1), gmap_translation_ts.Data(:,2),"k","LineWidth",1.0)

%grid on
%title('Robot Trajectories')
xlabel('x-pose [m]')
ylabel('y-pose [m]')
%zlabel('z-pose [m]')
legend("ground-truth","hector-slam","karto-slam", "gmapping")
%}


%% Plot the topic varaiables
% convert the timestamps to real time
gt_time = ((gt_time_data + 0.015)- (0.015 + gt_time_data(1)));
hector_time = ((hector_time_data + 0.01) - (0.01 + hector_time_data(1)));
karto_time = ((karto_time_data + 0.002) - (0.002 + karto_time_data(1)));
gmapping_time = (gmap_time_data-gmap_time_data(1));

subplot(2, 2, 1)
plot(gt_time,gt_translation_ts.Data(:,1), gt_time,gt_translation_ts.Data(:,2), gt_time,gt_rotation_ts.Data(:,4))
%title('gt pose')
xlabel('Time [seconds]')
ylabel('gt pose')
legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
%xlim([0 500])
grid on

subplot(2, 2 ,2)
plot(hector_time,hector_translation_ts.Data(:,1), hector_time,hector_translation_ts.Data(:,2), hector_time,hector_rotation_ts.Data(:,4))
%title('hector estimated pose')
xlabel('Time [seconds]')
ylabel('hector pose')
legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
grid on

subplot(2, 2 ,3)
plot(karto_time,karto_translation_ts.Data(:,1), karto_time,karto_translation_ts.Data(:,2), karto_time,karto_rotation_ts.Data(:,4))
%title('karto pose')
xlabel('Time [seconds]')
ylabel('karto pose')
legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
grid on


subplot(2, 2 ,4)
plot(gmapping_time,gmap_translation_ts.Data(:,1), gmapping_time,gmap_translation_ts.Data(:,2), gmapping_time,gmap_rotation_ts.Data(:,4))
%title('gmapping estimated pose')
xlabel('Time [seconds]')
ylabel('gmapping pose')
legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
grid on

%% Plot the rotation data

plot(gt_time,abs(log10(gt_rotation_ts.Data(:,4))), hector_time,abs(log10(hector_rotation_ts.Data(:,4))), gmap_time,abs(log10(gmap_rotation_ts.Data(:,4))))
%title('gmapping estimated pose')



%% Generate table for all the values

%tbl = table(t_amcl_x,t_amcl_y,t_amcl_z, resize_odom_x, resize_odom_y, resize_odom_z);   % Combine them in a table
%


%%  Compute the overall trajectory Mean, Median, and stdev

% Extract the position data from the timeseries object
position_data_gt = gt_translation_ts;
position_data_hector = hector_translation_ts;
position_data_gmap = gmap_translation_ts;
position_data_karto = karto_translation_ts;

% Compute the mean and standard deviation of the position data
%Hector
euclidean_distance_hector = vecnorm(diff(position_data_hector.Data), 2, 2);
min_val_hector = min(euclidean_distance_hector)
max_val_hector = max(euclidean_distance_hector)
mean_position_hector = mean(euclidean_distance_hector)
stddev_position_hector = std(euclidean_distance_hector)

%Gmap
euclidean_distance_gmap = vecnorm(diff(position_data_gmap.Data), 2, 2);
min_val_gmap = min(euclidean_distance_gmap)
max_val_gmap = max(euclidean_distance_gmap)
mean_position_gmap = mean(euclidean_distance_gmap)
stddev_position_gmap = std(euclidean_distance_gmap)

%Karto
euclidean_distance_karto = vecnorm(diff(position_data_karto.Data), 2, 2);
min_val_karto = min(euclidean_distance_karto)
max_val_karto = max(euclidean_distance_karto)
mean_position_karto = mean(euclidean_distance_karto)
stddev_position_karto = std(euclidean_distance_karto)




%% Compute the ATE and RPE

% Extract the data and timestamps from the timeseries object
ground_truth_x = gt_translation_ts.Data(:,1);
ground_truth_y = gt_translation_ts.Data(:,2);
ground_truth_theta = gt_rotation_ts.Data(:,4);

% Hector SLAM 
hector_estimated_x = hector_translation_ts.Data(:,1);
hector_estimated_y = hector_translation_ts.Data(:,2);
hector_estimated_theta = hector_rotation_ts.Data(:,4);

% Karto SLAM 
karto_estimated_x = karto_translation_ts.Data(:,1);
karto_estimated_y = karto_translation_ts.Data(:,2);
karto_estimated_theta = karto_rotation_ts.Data(:,4);

% Gmapping 
gmap_estimated_x = gmap_translation_ts.Data(:,1);
gmap_estimated_y = gmap_translation_ts.Data(:,2);
gmap_estimated_theta = gmap_rotation_ts.Data(:,4);


%% Interpolate the estimated data

% GT
interp_gt_x = imresize(ground_truth_x,[192,1]);
interp_gt_y = imresize(ground_truth_y,[192,1]);
interp_gt_theta = imresize(ground_truth_theta,[192,1]);
interp_gt_time = imresize(gt_time,[192,1]);

% Hector-SLAM
interp_hector_x = imresize(hector_estimated_x,[192,1]);
interp_hector_y = imresize(hector_estimated_y,[192,1]);
interp_hector_theta = imresize(hector_estimated_theta,[192,1]);
interp_hector_time = imresize(hector_time,[192,1]);

% Karto-SLAM
interp_karto_x = imresize(karto_estimated_x,[192,1]);
interp_karto_y = imresize(karto_estimated_y,[192,1]);
interp_karto_theta = imresize(karto_estimated_theta,[192,1]);
interp_karto_time = imresize(karto_time,[192,1]);

% Gmapping
interp_gmapping_x = imresize(gmap_estimated_x,[192,1]);
interp_gmapping_y = imresize(gmap_estimated_y,[192,1]);
interp_gmapping_theta = imresize(gmap_estimated_theta,[192,1]);
interp_gmap_time = imresize(gmapping_time,[192,1]);


% Compute the error between ground truth and interpolated estimated poses

% Hector-SLAM
errors_hector_x = (interp_hector_x - interp_gt_x);
errors_hector_y = (interp_hector_y - interp_gt_y);
errors_hector_theta = interp_gt_theta - interp_hector_theta;

% Karto-SLAM
errors_karto_x = (interp_karto_x - interp_gt_x);
errors_karto_y = (interp_karto_y - interp_gt_y);
errors_karto_theta = (interp_karto_theta - interp_gt_theta);

% Gmapping
errors_gmapping_x = (interp_gmapping_x - interp_gt_x);
errors_gmapping_y = (interp_gmapping_y - interp_gt_y);
errors_gmapping_theta = (interp_gmapping_theta - interp_gt_theta);

% Compute the absolute trajectory error
hector_ate = sqrt(errors_hector_x.^2 + errors_hector_y.^2);
karto_ate = sqrt(errors_karto_x.^2 + errors_karto_y.^2);
gmapping_ate = sqrt(errors_gmapping_x.^2 + errors_gmapping_y.^2);

% Create an array of ate values for multiple runs of the slam algorithm
ate_values_hector = [abs(hector_ate)];
ate_values_gmap = [abs(gmapping_ate)];
ate_values_karto = [abs(karto_ate)];

% Set the bin edges for the histogram
bins_hector = linspace(min(ate_values_hector(:)), max(ate_values_hector(:)), 40);
bins_gmap = linspace(min(ate_values_gmap(:)), max(ate_values_gmap(:)), 40);
bins_karto = linspace(min(ate_values_karto(:)), max(ate_values_karto(:)), 40);

% Plot the histogram
figure;
histogram(ate_values_hector, bins_hector, 'Normalization', 'probability', 'FaceColor', 'r','FaceAlpha',1.0);
hold on
histogram(ate_values_gmap, bins_gmap, 'Normalization', 'probability', 'FaceColor', 'b', 'EdgeAlpha', 0.1);
hold on
histogram(ate_values_karto, bins_karto, 'Normalization', 'probability', 'FaceColor', 'c', 'EdgeAlpha', 0.06);

legend('Hector-SLAM', 'Gmapping', 'Karto-SLAM');
ax = gca; % Get the current axes
ax.XAxis.LineWidth = 2; % Set the x axis line width
ax.YAxis.LineWidth = 2; % Set the y axis line width
%ax.FontSize = 12; % Set the font size for the axis numbers
xlabel('ATE Error (m)');
ylabel('Probability');
%title('ATE Error Histograms');
set(gca, 'FontWeight', 'bold'); % sets the font weight of the current axes
%set(gca, 'XTickLabel', get(gca, 'XTick'), 'FontWeight', 'bold'); % sets the font weight of the x tick labels
%set(gca, 'YTickLabel', get(gca, 'YTick'), 'FontWeight', 'bold'); % sets the font weight of the y tick labels
hold off;

xline(mean(abs(hector_ate)),'--k', '\mu','HandleVisibility','off');
xline(mean(abs(karto_ate)),'--k', '\mu','HandleVisibility','off');
xline(mean(abs(gmapping_ate)),'--k', '\mu','HandleVisibility','off');




% Plot the ATE against timestamp
figure;
plot(abs(interp_gt_time), abs(hector_ate));
hold on
plot(abs(interp_gt_time), abs(karto_ate));
hold on
plot(abs(interp_gt_time), abs(gmapping_ate));

xlabel('Time (s)');
ylabel('ATE (m)');
legend('hector-slam', 'karto','gmapping')


%}

%% Compute RPE

% Compute the translation error
%Hector
x_trans_hector = (interp_gt_x - interp_hector_x).^2;
y_trans_hector = (interp_gt_y - interp_hector_y).^2;
trans_error_hector = sqrt(x_trans_hector + y_trans_hector);

%karto
x_trans_karto = (interp_gt_x - interp_karto_x).^2;
y_trans_karto = (interp_gt_y - interp_karto_y).^2;
trans_error_karto = sqrt(x_trans_karto + y_trans_karto);

%Gmap
x_trans_gmap = (interp_gt_x - interp_gmapping_x).^2;
y_trans_gmap = (interp_gt_y - interp_gmapping_y).^2;
trans_error_gmap = sqrt(x_trans_gmap + y_trans_gmap);

% Compute the angular error
%Hector
ang_error_hector = abs(interp_gt_theta - interp_hector_theta);
ang_error_hector = min(ang_error_hector, 2*pi - ang_error_hector);

%Karto
ang_error_karto = abs(interp_gt_theta - interp_karto_theta);
ang_error_karto = min(ang_error_karto, 2*pi - ang_error_karto);

%Gmap
ang_error_gmap = abs(interp_gt_theta - interp_gmapping_theta);
ang_error_gmap = min(ang_error_gmap, 2*pi - ang_error_gmap);

% Compute the relative pose error
%Hector
rpe_trans_hector = trans_error_hector ./ sqrt(interp_gt_x.^2 + interp_gt_y.^2);
rpe_ang_hector = ang_error_hector ./ (2*pi);

% Karto
rpe_trans_karto = trans_error_karto ./ sqrt(interp_gt_x.^2 + interp_gt_y.^2);
rpe_ang_karto = ang_error_karto ./ (2*pi);

% Gmap
rpe_trans_gmap = trans_error_gmap ./ sqrt(interp_gt_x.^2 + interp_gt_y.^2);
rpe_ang_gmap = ang_error_gmap ./ (2*pi);

% display the RPE error as scalar
%Hector
hector_RPE_mean = mean(rpe_trans_hector);
fprintf('hector_RPE_mean: %f\n', hector_RPE_mean);
%Karto
karto_RPE_mean = mean(rpe_trans_karto);
fprintf('karto_RPE_mean: %f\n', karto_RPE_mean);
%Gmap
gmap_RPE_mean = mean(rpe_trans_gmap);
fprintf('gmap_RPE_mean: %f\n', gmap_RPE_mean);


% Plot the RPE
figure;
subplot(1,1,1);
plot(interp_hector_time, abs(rpe_trans_hector));
hold on 
plot(interp_hector_time, abs(rpe_trans_karto));
hold on
plot(interp_hector_time, abs(rpe_trans_gmap));
legend("Hector-SLAM","Karto-SLAM","Gmapping")
xlabel('Time (s)');
ylabel('Translational RPE');

%{
subplot(2,1,2);
plot(interp_hector_time,rpe_ang_hector);
hold on 
plot(interp_hector_time, rpe_ang_karto);
hold on
plot(interp_hector_time, rpe_ang_gmap);
legend("Hector","Karto","Gmap")

xlabel('Time (s)');
ylabel('Angular RPE');
%}


%% Compute RMSE

% Compute the mean squared error
%Hector
x_mse_trans_hector = (interp_gt_x - interp_hector_x).^2 ;
y_mse_trans_hector = (interp_gt_y - interp_hector_y).^2;
mse_trans_hector = mean(x_mse_trans_hector + y_mse_trans_hector);
mse_ang_hector = mean(ang_error_hector.^2);
% Compute the root mean squared error
rmse_trans_hector = sqrt(mse_trans_hector);
rmse_ang_hector = sqrt(mse_ang_hector);


%karto
x_mse_trans_karto = (interp_gt_x - interp_karto_x).^2 ;
y_mse_trans_karto = (interp_gt_y - interp_karto_y).^2;
mse_trans_karto = mean(x_mse_trans_karto + y_mse_trans_karto);
mse_ang_karto = mean(ang_error_karto.^2);
% Compute the root mean squared error
rmse_trans_karto = sqrt(mse_trans_karto);
rmse_ang_karto = sqrt(mse_ang_karto);


%Gmap
x_mse_trans_gmap = (interp_gt_x - interp_gmapping_x).^2 ;
y_mse_trans_gmap = (interp_gt_y - interp_gmapping_y).^2;
mse_trans_gmap = mean(x_mse_trans_gmap + y_mse_trans_gmap);
mse_ang_gmap = mean(ang_error_gmap.^2);
% Compute the root mean squared error
rmse_trans_gmap = sqrt(mse_trans_gmap);
rmse_ang_gmap = sqrt(mse_ang_gmap);


% Plot the RMSE
%{
figure;
subplot(2,1,1);
plot(hector_time, log10(rmse_trans));
xlabel('Time (s)');
ylabel('Translational RMSE (m)');
subplot(2,1,2);
plot(hector_time, log10(rmse_ang));
xlabel('Time (s)');
ylabel('Angular RMSE (rad)');

%}

%% Compute the scale drift

% Compute the cumulative distance traveled by the ground truth trajectory
gt_cumulative_distance = cumsum(sqrt(diff(interp_gt_x).^2 + diff(interp_gt_y).^2));

% Compute the cumulative distance traveled by the hector trajectory
hector_cumulative_distance = cumsum(sqrt(diff(interp_hector_x).^2 + diff(interp_hector_y).^2));

% Compute the cumulative distance traveled by the karto trajectory
karto_cumulative_distance = cumsum(sqrt(diff(interp_karto_x).^2 + diff(interp_karto_y).^2));

% Compute the cumulative distance traveled by the gmap trajectory
gmap_cumulative_distance = cumsum(sqrt(diff(interp_gmapping_x).^2 + diff(interp_gmapping_y).^2));


% Compute the scale drift in percentage
scale_drift_hector =  (hector_cumulative_distance ./ gt_cumulative_distance);
scale_drift_karto = (karto_cumulative_distance ./ gt_cumulative_distance);
scale_drift_gmap = (gmap_cumulative_distance ./ gt_cumulative_distance);

%Mean Scale Drift

mean_hector_scale_drift = mean(scale_drift_hector)
mean_karto_scale_drift = mean(scale_drift_karto)
mean_gmap_scale_drift = mean(scale_drift_gmap)



% Plot the scale drift
drift_time_hector = imresize(hector_time,[1923,1]);
drift_time_karto = imresize(karto_time,[1923,1]);
drift_time_gmap = imresize(gmapping_time,[1923,1]);

figure;
plot(abs(drift_time_hector), abs(scale_drift_hector));
hold on
plot(abs(drift_time_karto), abs(scale_drift_karto));
hold on
plot(abs(drift_time_gmap), abs(scale_drift_gmap));

xlabel('Time (s)');
ylabel('Scale Drift (%)');
legend('Hector-SLAM','Karto-SLAM','Gmapping')

%% scale drift as scalar
%{
estimated_poses = [hector_translation_ts.Data(:,1), hector_translation_ts.Data(:,2), hector_translation_ts.Data(:,3)];
ground_truth_poses = [gt_translation_ts.Data(:,1), gt_translation_ts.Data(:,2), gt_translation_ts.Data(:,3)];

cumulative_distance_est = 0;
for i = 2:length(estimated_poses)
    delta = estimated_poses(i, :) - estimated_poses(i-1, :);
    cumulative_distance_est = cumulative_distance_est + norm(delta);
end

cumulative_distance_gt = 0;
for i = 2:length(ground_truth_poses)
    delta = ground_truth_poses(i, :) - ground_truth_poses(i-1, :);
    cumulative_distance_gt = cumulative_distance_gt + norm(delta);
end

scale_drift_scalar = cumulative_distance_est / cumulative_distance_gt;
 
estimated_posesInt = imresize(estimated_poses(:,1),[1923,1]);

figure;
plot(estimated_posesInt, scale_drift);
xlabel('X position of estimated trajectory');
ylabel('Scale Drift');
title('Scale Drift against estimated X position');


%% scale drift along with other metrics, like RPE and RMSE, to get a more comprehensive view of the performance of the algorithm.

% Create a figure with 3 subplots
figure;
subplot(3,1,1);
plot(hector_time(2:end), scale_drift);
xlabel('Time (s)');
ylabel('Scale Drift');

subplot(3,1,2);
plot(hector_time, rpe_trans);
xlabel('Time (s)');
ylabel('Translational RPE');

subplot(3,1,3);
plot(hector_time, rmse_trans);
xlabel('Time (s)');
ylabel('Translational RMSE (m)');

%}

%% Plot the translation error against the distance traveled by the robot:

% Compute the distance traveled by the robot
distance = cumsum(sqrt(diff(interp_gt_x).^2 + diff(interp_gt_y).^2));

% Compute the translation error
%Hector
error_hector = sqrt((interp_hector_x - interp_gt_x).^2 + (interp_hector_y - interp_gt_y).^2);
% Gmap
error_gmap = sqrt((interp_gmapping_x - interp_gt_x).^2 + (interp_gmapping_y - interp_gt_y).^2);
% Karto
error_karto = sqrt((interp_karto_x - interp_gt_x).^2 + (interp_karto_y - interp_gt_y).^2);


% Interpolate the error to match the distance traveled
interp_error_hector = imresize(error_hector,[1923,1]);
interp_error_gmap = imresize(error_gmap,[1923,1]);
interp_error_karto = imresize(error_karto,[1923,1]);

% Plot the translation error against the distance traveled
figure;
plot(distance, interp_error_hector);
hold on
plot(distance, interp_error_gmap);
hold on
plot(distance, interp_error_karto);

xlabel('Distance traveled (m)');
ylabel('Translation error (m)');
%title('Translation error vs distance traveled');% Compute the distance traveled by the robot
legend('hector','gmap','karto')


%% Average translation error in percentage

% Compute the average error
average_error_hector = mean(error_hector);
average_error_gmap = mean(error_gmap);
average_error_karto = mean(error_karto);

% Compute the average error in percentage
average_error_percent_hector = 100 * average_error_hector / mean(sqrt(interp_gt_x.^2 + interp_gt_y.^2));
average_error_percent_gmap = 100 * average_error_gmap / mean(sqrt(interp_gt_x.^2 + interp_gt_y.^2));
average_error_percent_karto = 100 * average_error_karto / mean(sqrt(interp_gt_x.^2 + interp_gt_y.^2));

% Display the average error in percentage
disp(['Average error in percentage_hector: ', num2str(average_error_percent_hector), '%']);
disp(['Average error in percentage_gmap: ', num2str(average_error_percent_gmap), '%']);
disp(['Average error in percentage_karto: ', num2str(average_error_percent_karto), '%']);

%% Computing the time to convergence (TTC) metrics 

% Define the convergence threshold
threshold_hector = 0.007;
threshold_gmap = 0.01;
threshold_karto = 0.01;

% Find the index of the first time the error is below the threshold
hector_first_convergence_index = find(hector_ate < threshold_hector, 1);
karto_first_convergence_index = find(karto_ate < threshold_karto, 1);
gmap_first_convergence_index = find(gmapping_ate < threshold_gmap, 1);




% Compute the time to convergence
hector_time_to_convergence = interp_hector_time(hector_first_convergence_index);
karto_time_to_convergence = interp_karto_time(karto_first_convergence_index);
gmap_time_to_convergence = interp_gmap_time(gmap_first_convergence_index);



% Display the time to convergence
disp(['Time to convergence_hector: ', num2str(hector_time_to_convergence), 's']);
disp(['Time to convergence_gmap: ', num2str(gmap_time_to_convergence), 's']);
disp(['Time to convergence_karto: ', num2str(karto_time_to_convergence), 's']);

% Plot the ATE and TTC
figure;
plot(interp_gt_time, hector_ate, 'LineWidth', 1.0);
hold on
plot(interp_gt_time, karto_ate, 'LineWidth', 1.0);
hold on
plot(interp_gt_time, gmapping_ate, 'LineWidth', 1.0);

hold on;
plot(hector_time_to_convergence, hector_ate(hector_first_convergence_index), '<', 'MarkerSize', 10);
hold on;
plot(karto_time_to_convergence, karto_ate(karto_first_convergence_index), 'o', 'MarkerSize', 10);
hold on;
plot(gmap_time_to_convergence, gmapping_ate(gmap_first_convergence_index), '*', 'MarkerSize', 10);
xlabel('Time (s)');
ylabel('ATE (m)');
title('ATE vs Time');
legend({'ATE', 'TTC'});
