%%
% =========================================================================
%  Understanding why SLAM algorithms fail in modern indoor environments.
% =========================================================================

%                   Long Passage Environment (Long Pass.)

% ======================= Useful Instruction ==============================
%   This code runs in sections, first you have to use the 'Run' command to
%   load the parameters to workspace, it will return some warnings and
%   errors, never mind. Use the 'Run section' command to analyse the
%   section of your choice. Make sure you click at the section first before
%   using the 'Run section' command.
% =========================================================================

%%  ==================  Load Dataset the workspace  =======================

%   Check if there is existing data loaded in workspace  
if ~isempty(who)
%   If there is data, the you have to to clear the workspace
    clear_data = questdlg('There is existing data in the workspace. Do you want to clear the workspace?', 'Clear Workspace', 'Yes', 'No', 'Yes');
    
    if strcmp(clear_data, 'Yes')
%   Clear the workspace and command prompt, clear all variables
        close all
        clear
        clc
        disp('Workspace cleared')
    else
%   If you doesn't want to clear the workspace, exit the program
        return
    end
end

%%   If there is no data loaded, ask user to load data to workspace
if ~exist('data_loaded', 'var')
%     data_loaded = false;
    load_data = questdlg('Do you want to load data to workspace?', 'Load Data', 'Yes', 'No', 'Yes');

  if strcmp(load_data, 'Yes')

    gt_LongPass = 'gtLongPass.csv'; % GroundTruth
    hector_LongPass = 'hectorLongPass.csv';
    gmap_LongPass = 'gmapLongPass.csv';
    karto_LongPass = 'kartoLongPass.csv';

%   Read and extract the messages 
    gt_LongPass_data = readtable(gt_LongPass);
    hector_LongPass_data = readtable(hector_LongPass);
    gmap_LongPass_data = readtable(gmap_LongPass);
    karto_LongPass_data = readtable(karto_LongPass);

%   Extract the timestamp column
    gt_time_stamp = gt_LongPass_data{:,1};
    hector_time_stamp = hector_LongPass_data{:,1};
    gmap_time_stamp = gmap_LongPass_data{:,1};
    karto_time_stamp = karto_LongPass_data{:,1};

%   Convert the timestamps to real time
    gt_time = ((gt_time_stamp)- (gt_time_stamp(1)));
    hector_time = ((hector_time_stamp)- (hector_time_stamp(1)));
    gmap_time = ((gmap_time_stamp)- (gmap_time_stamp(1)));
    karto_time = ((karto_time_stamp)- (karto_time_stamp(1)));

%   Extract the x, y, z position columns
    gt_x = gt_LongPass_data{:,2};
    gt_y = gt_LongPass_data{:,3};
    gt_z = gt_LongPass_data{:,4};
    
    hector_x = hector_LongPass_data{:,2};
    hector_y = hector_LongPass_data{:,3};
    hector_z = hector_LongPass_data{:,4};

    gmap_x = gmap_LongPass_data{:,2};
    gmap_y = gmap_LongPass_data{:,3};
    gmap_z = gmap_LongPass_data{:,4};

    karto_x = karto_LongPass_data{:,2};
    karto_y = karto_LongPass_data{:,3};
    karto_z = karto_LongPass_data{:,4};

%   Extract the qx, qy, qz, qw rotation columns
    gt_qz = gt_LongPass_data{:,7};
    hector_qz = hector_LongPass_data{:,7};
    gmap_qz = gmap_LongPass_data{:,7};
    karto_qz = karto_LongPass_data{:,7};

%   Arrange the position and rotation data
%   Pose data
    gt_translation_ts = [gt_time, (gt_x ), (gt_y), gt_z];
    hector_translation_ts = [hector_time, hector_x, hector_y, hector_z];
    karto_translation_ts = [karto_time, karto_x , karto_y, karto_z];
    gmap_translation_ts = [gmap_time, gmap_x, gmap_y, gmap_z];
    
%   Angular data 
    gt_rotation_ts = [gt_qz];
    hector_rotation_ts = [hector_qz];
    karto_rotation_ts = [karto_qz];
    gmap_rotation_ts = [gmap_qz];

%   Extract the ground-truth (GT) data
    ground_truth_x = gt_translation_ts(:,2);
    ground_truth_y = gt_translation_ts(:,3);
    ground_truth_theta = gt_rotation_ts(:,1);

%   Hector SLAM 
    hector_estimated_x = hector_translation_ts(:,2);
    hector_estimated_y = hector_translation_ts(:,3);
    hector_estimated_theta = hector_rotation_ts(:,1);

%   Karto SLAM 
    karto_estimated_x = karto_translation_ts(:,2);
    karto_estimated_y = karto_translation_ts(:,3);
    karto_estimated_theta = karto_rotation_ts(:,1);

%   Gmapping 
    gmap_estimated_x = gmap_translation_ts(:,2);
    gmap_estimated_y = gmap_translation_ts(:,3);
    gmap_estimated_theta = gmap_rotation_ts(:,1);


% Interpolate the estimated data
%   target length of the interpolated matrix
    target_length = 9360;

%   GT
    interp_gt_x = interp1(1:length(ground_truth_x), ground_truth_x, linspace(1, length(ground_truth_x), target_length))';
    interp_gt_y = interp1(1:length(ground_truth_y), ground_truth_y, linspace(1, length(ground_truth_y), target_length))';
    interp_gt_theta = interp1(1:length(ground_truth_theta), ground_truth_theta, linspace(1, length(ground_truth_theta), target_length))';
    interp_gt_time = interp1(1:length(gt_time), gt_time, linspace(1, length(gt_time), target_length))';

%   Hector-SLAM
    interp_hector_x = interp1(1:length(hector_estimated_x), hector_estimated_x, linspace(1, length(hector_estimated_x), target_length))';
    interp_hector_y = interp1(1:length(hector_estimated_y), hector_estimated_y, linspace(1, length(hector_estimated_y), target_length))';
    interp_hector_theta = interp1(1:length(hector_estimated_theta), hector_estimated_theta, linspace(1, length(hector_estimated_theta), target_length))';
    interp_hector_time = interp1(1:length(hector_time), hector_time, linspace(1, length(hector_time), target_length))';

%   Karto-SLAM
    interp_karto_x = interp1(1:length(karto_estimated_x), karto_estimated_x, linspace(1, length(karto_estimated_x), target_length))';
    interp_karto_y = interp1(1:length(karto_estimated_y), karto_estimated_y, linspace(1, length(karto_estimated_y), target_length))';
    interp_karto_theta =  interp1(1:length(karto_estimated_theta), karto_estimated_theta, linspace(1, length(karto_estimated_theta), target_length))';
    interp_karto_time = interp1(1:length(karto_time), karto_time, linspace(1, length(karto_time), target_length))';

%   Gmapping
    interp_gmapping_x = interp1(1:length(gmap_estimated_x), gmap_estimated_x, linspace(1, length(gmap_estimated_x), target_length))';
    interp_gmapping_y = interp1(1:length(gmap_estimated_y), gmap_estimated_y, linspace(1, length(gmap_estimated_y), target_length))';
    interp_gmapping_theta = interp1(1:length(gmap_estimated_theta), gmap_estimated_theta, linspace(1, length(gmap_estimated_theta), target_length))';
    interp_gmap_time = interp1(1:length(gmap_time), gmap_time, linspace(1, length(gmap_time), target_length))';
    
% Compute the errors between ground truth and interpolated estimated poses
%   Hector-SLAM
    errors_hector_x = interp_hector_x - interp_gt_x;
    errors_hector_y = interp_hector_y - interp_gt_y;
    errors_hector_theta = interp_gt_theta - interp_hector_theta;

%   Karto-SLAM
    errors_karto_x = interp_gt_x - interp_karto_x;
    errors_karto_y = interp_gt_y - interp_karto_y;
    errors_karto_theta = interp_gt_theta - interp_karto_theta;

%   Gmapping
    errors_gmapping_x = interp_gt_x - interp_gmapping_x;
    errors_gmapping_y = interp_gt_y - interp_gmapping_y;
    errors_gmapping_theta = interp_gt_theta - interp_gmapping_theta;
    
% Compute the translation errors
%   Hector
    x_trans_hector = (interp_gt_x - interp_hector_x).^2;
    y_trans_hector = (interp_gt_y - interp_hector_y).^2;
    trans_error_hector = sqrt(x_trans_hector + y_trans_hector);

%   karto-SLAM
    x_trans_karto = (interp_gt_x - interp_karto_x).^2;
    y_trans_karto = (interp_gt_y - interp_karto_y).^2;
    trans_error_karto = sqrt(x_trans_karto + y_trans_karto);

%   Gmapping
    x_trans_gmap = (interp_gt_x - interp_gmapping_x).^2;
    y_trans_gmap = (interp_gt_y - interp_gmapping_y).^2;
    trans_error_gmap = sqrt(x_trans_gmap + y_trans_gmap);

% Compute the angular errors
%   Hector
    ang_error_hector = abs(interp_gt_theta - interp_hector_theta);
    ang_error_hector = min(ang_error_hector, 2*pi - ang_error_hector);

%   Karto
    ang_error_karto = abs(interp_gt_theta - interp_karto_theta);
    ang_error_karto = min(ang_error_karto, 2*pi - ang_error_karto);

%   Gmapping
    ang_error_gmap = abs(interp_gt_theta - interp_gmapping_theta);
    ang_error_gmap = min(ang_error_gmap, 2*pi - ang_error_gmap);

% Compute the translation errors
%   Hector
    error_hector = sqrt((interp_hector_x - interp_gt_x).^2 + (interp_hector_y - interp_gt_y).^2);
%   Gmapping
    error_gmap = sqrt((interp_gmapping_x - interp_gt_x).^2 + (interp_gmapping_y - interp_gt_y).^2);
%   Karto
    error_karto = sqrt((interp_karto_x - interp_gt_x).^2 + (interp_karto_y - interp_gt_y).^2);

%   Interpolate the errors to match the distance traveled
    interp_error_hector = imresize(error_hector,[9359,1]);
    interp_error_gmap = imresize(error_gmap,[9359,1]);
    interp_error_karto = imresize(error_karto,[9359,1]);
        
%     data_loaded = true;
    disp('Data loaded to workspace')
  end
else
    return
end

%% Sections
sections = {'Show the trajectories', 'Absolute trajectory errors (ATE)', 'Relative pose error (RPE)', 'Root mean square error (RMSE)', 'Scale drift', 'Avg. error and distance travelled', 'Time to convergence', 'Quit'};
[indx, tf] = listdlg('PromptString', 'Select section(s) to run:', 'ListString', sections, 'SelectionMode', 'multiple');
if tf
%   Execute the selected sections
    for i = 1:length(indx)
        switch sections{indx(i)}    
% Choose the section that you want to analyse
%    select_section = questdlg('Which section do you want to analyse?', 'Select Section', 'Plot the trajectories', 'Plot ATE, RPE & RMSE', 'Plot SD & TTC', 'Plot the trajectories');

%% Plot the robot's trajectory data as well as the variables
            case 'Show the trajectories'       
%   Plot the trajectories in 2D
    figure;
    plot(gt_translation_ts(:,2),gt_translation_ts(:,3),"r","LineWidth",1.0)
    hold on
    plot(hector_translation_ts(:,2),hector_translation_ts(:,3),"b","LineWidth",1.0)
    hold on
    plot(karto_translation_ts(:,2),karto_translation_ts(:,3),"g","LineWidth",1.0)
    hold on
    plot(gmap_translation_ts(:,2), gmap_translation_ts(:,3),"k","LineWidth",1.0)
    xlabel('x-pose [m]')
    ylabel('y-pose [m]')
%    zlabel('z-pose [m]')
    legend("GTCPSLab","hectorCPSLab","kartoCPSLab", "gmappingCPSLab")

%   Plot the topic varaiables
    figure;
    subplot(2, 2, 1)
    plot(gt_time,gt_translation_ts(:,2), gt_time,gt_translation_ts(:,3), gt_time,gt_rotation_ts(:,1))
    %title('gt pose')
    xlabel('Time [seconds]')
    ylabel('gt pose')
    legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
%    xlim([0 500])
    grid on

    subplot(2, 2 ,2)
    plot(hector_time,hector_translation_ts(:,2), hector_time,hector_translation_ts(:,3), hector_time,hector_rotation_ts(:,1))
%   title('hector estimated pose')
    xlabel('Time [seconds]')
    ylabel('hector pose')
    legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
    grid on

    subplot(2, 2 ,3)
    plot(karto_time,karto_translation_ts(:,2), karto_time,karto_translation_ts(:,3), karto_time,karto_rotation_ts(:,1))
%   title('karto pose')
    xlabel('Time [seconds]')
    ylabel('karto pose')
    legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
    grid on

    subplot(2, 2 ,4)
    plot(gmap_time,gmap_translation_ts(:,2), gmap_time,gmap_translation_ts(:,3), gmap_time,gmap_rotation_ts(:,1))
%   title('gmapping estimated pose')
    xlabel('Time [seconds]')
    ylabel('gmapping pose')
    legend("x-pose [m]","y-pose [m]", "angular-z [rad]")
    grid on    

    disp('The displayed plots are the robot trajectories')

%% Analyse the absolute trajectory error (ATE)  
            case 'Absolute trajectory errors (ATE)'           
%   Compute the absolute trajectory errors (ATE)
    hector_ate = sqrt(errors_hector_x.^2 + errors_hector_y.^2);
    karto_ate = sqrt(errors_karto_x.^2 + errors_karto_y.^2);
    gmapping_ate = sqrt(errors_gmapping_x.^2 + errors_gmapping_y.^2);

%   Create an array of ate values for multiple runs of the slam algorithm
    ate_values_hector = [hector_ate];
    ate_values_gmap = [gmapping_ate];
    ate_values_karto = [karto_ate];

%   Set the bin edges for the histogram
    bins_hector = linspace(min(ate_values_hector(:)), max(ate_values_hector(:)), 40);
    bins_gmap = linspace(min(ate_values_gmap(:)), max(ate_values_gmap(:)), 40);
    bins_karto = linspace(min(ate_values_karto(:)), max(ate_values_karto(:)), 40);

%   Plot ATE error histogram
    figure;
    histogram(ate_values_hector, bins_hector, 'Normalization', 'probability', 'FaceColor', 'r','FaceAlpha',1.0);
    hold on
    histogram(ate_values_gmap, bins_gmap, 'Normalization', 'probability', 'FaceColor', 'b', 'EdgeAlpha', 0.1);
    hold on
    histogram(ate_values_karto, bins_karto, 'Normalization', 'probability', 'FaceColor', 'c', 'EdgeAlpha', 0.06);
    legend('Hector-SLAM', 'Gmapping', 'Karto-SLAM');
    ax = gca; 
    ax.XAxis.LineWidth = 2;
    ax.YAxis.LineWidth = 2;
    ax.FontSize = 14; 
    xlabel('ATE Error (m)');
    ylabel('Probability');
%   title('ATE Error Histograms');
    set(gca, 'FontWeight', 'bold');	
%   set(gca, 'XTickLabel', get(gca, 'XTick'), 'FontWeight', 'bold');	
%   set(gca, 'YTickLabel', get(gca, 'YTick'), 'FontWeight', 'bold'); 	

    xline(mean(hector_ate),'--r', '\mu','HandleVisibility','off');
    xline(mean(karto_ate),'--c', '\mu','HandleVisibility','off');
    xline(mean(gmapping_ate),'--b', '\mu','HandleVisibility','off');
    
    hold off;

%   Plot the ATE against timestamp
    figure;
    plot(interp_gt_time, hector_ate, 'r');
    hold on
    plot(interp_gt_time, karto_ate, 'b');
    hold on
    plot(interp_gt_time, gmapping_ate,'g');
    xlabel('Time (s)');
    ylabel('ATE (m)');
    legend('hector-slam', 'karto','gmapping')
    yline(mean(hector_ate),'--g', 'meanHector','HandleVisibility','off');
    yline(mean(karto_ate),'--c', 'meanKarto','HandleVisibility','off');
    yline(mean(gmapping_ate),'--m', 'meanGmapping','HandleVisibility','off');
    
    clc
    disp('The displayed plots shows the ATE')
    disp('The mean ATE for Hector-SLAM, Karto-SLAM and Gmapping are:')
    disp([mean(hector_ate) mean(karto_ate) mean(gmapping_ate)])
    disp('The standard deviation of the ATE for Hector-SLAM, Karto-SLAM and Gmapping are:')
    disp([std(hector_ate) std(karto_ate) std(gmapping_ate)])


%% Compute the relative pose errors (RPE)
            case 'Relative pose error (RPE)'    
%   Hector
    rpe_trans_hector = trans_error_hector ./ sqrt(interp_gt_x.^2 + interp_gt_y.^2);
    rpe_ang_hector = ang_error_hector ./ (2*pi);

%   Karto-SLAM
    rpe_trans_karto = trans_error_karto ./ sqrt(interp_gt_x.^2 + interp_gt_y.^2);
    rpe_ang_karto = ang_error_karto ./ (2*pi);

%   Gmapping
    rpe_trans_gmap = trans_error_gmap ./ sqrt(interp_gt_x.^2 + interp_gt_y.^2);
    rpe_ang_gmap = ang_error_gmap ./ (2*pi);

%   Plot the RPE
    figure;
    subplot(2,1,1);
    plot(interp_gt_time, rpe_trans_hector);
    hold on 
    plot(interp_gt_time, rpe_trans_karto);
    hold on
    plot(interp_gt_time, rpe_trans_gmap);
    legend("Hector-SLAM","Karto-SLAM","Gmapping")
    xlabel('Time (s)');
    ylabel('Translational RPE ');

    subplot(2,1,2);
    plot(interp_hector_time,rpe_ang_hector);
    hold on 
    plot(interp_hector_time, rpe_ang_karto);
    hold on
    plot(interp_hector_time, rpe_ang_gmap);
    legend("Hector-SLAM","Karto-SLAM","Gmapping")
    xlabel('Time (s)');
    ylabel('Angular RPE ');

    clc
    disp('The displayed plots shows the relative pose errors (RPE) ')
    disp('The mean RPE-trans for Hector-SLAM, Karto-SLAM and Gmapping are:')
    disp([mean(rpe_trans_hector) mean(rpe_trans_karto) mean(rpe_trans_gmap)])
    disp('The standard deviation of the RPE-trans for Hector-SLAM, Karto-SLAM and Gmapping are:')
    disp([std(rpe_trans_hector) std(rpe_trans_karto) std(rpe_trans_gmap)])
    
    disp('The mean RPE-rot for Hector-SLAM, Karto-SLAM and Gmapping are:')
    disp([mean(rpe_ang_hector) mean(rpe_ang_karto) mean(rpe_ang_gmap)])
    disp('The standard deviation of the RPE-rot for Hector-SLAM, Karto-SLAM and Gmapping are:')
    disp([std(rpe_ang_hector) std(rpe_ang_karto) std(rpe_ang_gmap)])    
    

%% Compute the root mean square error (RMSE)
            case 'Root mean square error (RMSE)'    
%   Hector
    x_mse_trans_hector = (interp_gt_x - interp_hector_x).^2 ;
    y_mse_trans_hector = (interp_gt_y - interp_hector_y).^2;
    mse_trans_hector = mean(x_mse_trans_hector + y_mse_trans_hector);
    mse_ang_hector = mean(ang_error_hector.^2);
%   Compute the root mean squared error
    rmse_trans_hector = sqrt(mse_trans_hector);
    rmse_ang_hector = sqrt(mse_ang_hector);

%   Karto-SLAM
    x_mse_trans_karto = (interp_gt_x - interp_karto_x).^2 ;
    y_mse_trans_karto = (interp_gt_y - interp_karto_y).^2;
    mse_trans_karto = mean(x_mse_trans_karto + y_mse_trans_karto);
    mse_ang_karto = mean(ang_error_karto.^2);
%   Compute the root mean squared error
    rmse_trans_karto = sqrt(mse_trans_karto);
    rmse_ang_karto = sqrt(mse_ang_karto);

%   Gmapping
    x_mse_trans_gmap = (interp_gt_x - interp_gmapping_x).^2 ;
    y_mse_trans_gmap = (interp_gt_y - interp_gmapping_y).^2;
    mse_trans_gmap = mean(x_mse_trans_gmap + y_mse_trans_gmap);
    mse_ang_gmap = mean(ang_error_gmap.^2);
%   Compute the root mean squared error
    rmse_trans_gmap = sqrt(mse_trans_gmap);
    rmse_ang_gmap = sqrt(mse_ang_gmap);
    
    clc   
    disp('The translational RMSE for Hector-SLAM, Karto-SLAM and Gmapping are:')
    disp([(rmse_trans_hector) (rmse_trans_karto) (rmse_trans_gmap)])
    
    disp('The angular RMSE for Hector-SLAM, Karto-SLAM and Gmapping are:')
    disp([(rmse_ang_hector) (rmse_ang_karto) (rmse_ang_gmap)])

%% Compute the average error and the distance traveled 
            case 'Avg. error and distance travelled'        
% Average translation error in percentage
%   Compute the average error
    average_error_hector = mean(error_hector);
    average_error_gmap = mean(error_gmap);
    average_error_karto = mean(error_karto);

%   Compute the average error in percentage
    average_error_percent_hector = 100 * average_error_hector / mean(sqrt(interp_gt_x.^2 + interp_gt_y.^2));
    average_error_percent_gmap = 100 * average_error_gmap / mean(sqrt(interp_gt_x.^2 + interp_gt_y.^2));
    average_error_percent_karto = 100 * average_error_karto / mean(sqrt(interp_gt_x.^2 + interp_gt_y.^2));
                
% Compute the cumulative distance traveled    
    gt_cumulative_distance = cumsum(sqrt(diff(interp_gt_x).^2 + diff(interp_gt_y).^2));
    hector_cumulative_distance = cumsum(sqrt(diff(interp_hector_x).^2 + diff(interp_hector_y).^2));
    karto_cumulative_distance = cumsum(sqrt(diff(interp_karto_x).^2 + diff(interp_karto_y).^2));
    gmap_cumulative_distance = cumsum(sqrt(diff(interp_gmapping_x).^2 + diff(interp_gmapping_y).^2));    
%   Compute the distance traveled by the robot
    distance = cumsum(sqrt(diff(interp_gt_x).^2 + diff(interp_gt_y).^2));
    
%   Display the average error in percentage
    clc
    disp(['Average error in percentage_hector: ', num2str(average_error_percent_hector), '%']);
    disp(['Average error in percentage_gmap: ', num2str(average_error_percent_gmap), '%']);
    disp(['Average error in percentage_karto: ', num2str(average_error_percent_karto), '%']);
        
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
    legend('Hector-SLAM','Gmapping','Karto-SLAM')

    disp('The displayed plots shows distance travelled against the errors')


%% Compute the scale drift    
            case 'Scale drift' 
 % Extract the ground truth and estimated trajectories from the table
    gt_traj = [interp_gt_x, interp_gt_y, interp_gt_theta];
    hector_est_traj = [interp_hector_x, interp_hector_y, interp_hector_theta];
    karto_est_traj = [interp_karto_x, interp_karto_y, interp_karto_theta];
    gmap_est_traj = [interp_gmapping_x, interp_gmapping_y, interp_gmapping_theta];

% Compute the scale deviation between the ground truth and estimated trajectories
    N = size(gt_traj, 1);
    hector_scale_deviation = zeros(N-1, 1);
    karto_scale_deviation = zeros(N-1, 1);
    gmap_scale_deviation = zeros(N-1, 1);
for k = 2:N
    gt_dist = norm(gt_traj(k,:) - gt_traj(k-1,:));
    hector_est_dist = norm(hector_est_traj(k,:) - hector_est_traj(k-1,:));
    karto_est_dist = norm(karto_est_traj(k,:) - karto_est_traj(k-1,:));
    gmap_est_dist = norm(gmap_est_traj(k,:) - gmap_est_traj(k-1,:));
    hector_scale_deviation(k-1) = hector_est_dist/gt_dist;
    karto_scale_deviation(k-1) = karto_est_dist/gt_dist;
    gmap_scale_deviation(k-1) = gmap_est_dist/gt_dist;
end

% Create a time vector for the scale deviation data
target_length_time = 9359;
time = interp1(1:length(gt_time), gt_time, linspace(1, length(gt_time), target_length_time))';

% Plot the scale deviation over time
plot(time, hector_scale_deviation, time, gmap_scale_deviation, time, karto_scale_deviation);
xlabel('Time');
ylabel('Scale deviation');
title('Scale deviation over time');               
                
                
                
% %   Compute the cumulative distance traveled by the ground truth trajectory
%     gt_cumulative_distance = cumsum(sqrt(diff(interp_gt_x).^2 + diff(interp_gt_y).^2));
% 
% %   Compute the cumulative distance traveled by the hector trajectory
%     hector_cumulative_distance = cumsum(sqrt(diff(interp_hector_x).^2 + diff(interp_hector_y).^2));
% 
% %   Compute the cumulative distance traveled by the karto trajectory
%     karto_cumulative_distance = cumsum(sqrt(diff(interp_karto_x).^2 + diff(interp_karto_y).^2));
% 
% %   Compute the cumulative distance traveled by the gmap trajectory
%     gmap_cumulative_distance = cumsum(sqrt(diff(interp_gmapping_x).^2 + diff(interp_gmapping_y).^2));
% 
% % Compute the scale drift
%     scale_drift_hector = (hector_cumulative_distance ./ gt_cumulative_distance);
%     scale_drift_karto = (karto_cumulative_distance ./ gt_cumulative_distance);
%     scale_drift_gmap = (gmap_cumulative_distance ./ gt_cumulative_distance);
% 
% %   Interpolate the scale drift time
%     drift_time_hector = imresize(hector_time,[9359,1]);
%     drift_time_karto = imresize(karto_time,[9359,1]);
%     drift_time_gmap = imresize(gmap_time,[9359,1]);
% 
% %   Plot the scale drift
%     figure;
%     plot(drift_time_hector, scale_drift_hector);
%     hold on
%     plot(drift_time_hector, scale_drift_karto);
%     hold on
%     plot(drift_time_hector, scale_drift_gmap);
%     xlabel('Time (s)');
%     ylabel('Scale Drift');
%     legend('Hector-SLAM','Karto-SLAM','Gmapping')
%     
%     clc
%     disp('The mean scale drift for Hector-SLAM, Karto-SLAM and Gmapping are:')
%     disp([mean(scale_drift_hector) mean(scale_drift_karto) mean(scale_drift_gmap)])
%     disp('The standard deviation of the scale drift for Hector-SLAM, Karto-SLAM and Gmapping are:')
%     disp([std(scale_drift_hector) std(scale_drift_karto) std(scale_drift_gmap)])
     
%% Compute the time to convergence (TTC)
            case 'Time to convergence'
% Compute the time to convergence (TTC) metrics 
%   Define the convergence threshold
    threshold = 0.1;

%   Find the index of the first time the error is below the threshold
    hector_first_convergence_index = find(hector_ate < threshold, 1);
    karto_first_convergence_index = find(karto_ate < threshold, 1);
    gmap_first_convergence_index = find(gmapping_ate < threshold, 1);

%   Compute the time to convergence
    hector_time_to_convergence = interp_hector_time(hector_first_convergence_index);
    karto_time_to_convergence = interp_karto_time(karto_first_convergence_index);
    gmap_time_to_convergence = interp_gmap_time(gmap_first_convergence_index);

%   Display the time to convergence
    disp(['Time to convergence_hector: ', num2str(hector_time_to_convergence), 's']);
    disp(['Time to convergence_karto: ', num2str(karto_time_to_convergence), 's']);
    disp(['Time to convergence_gmap: ', num2str(gmap_time_to_convergence), 's']);


%   Plot the ATE and TTC
    figure;
    plot(interp_gt_time, hector_ate, 'LineWidth', 1.0);
    hold on
    plot(interp_gt_time, karto_ate, 'LineWidth', 1.0);
    hold on
    plot(interp_gt_time, gmapping_ate, 'LineWidth', 1.0);

    hold on;
    plot(hector_time_to_convergence, hector_ate(hector_first_convergence_index), '*', 'MarkerSize', 10);
    hold on;
    plot(karto_time_to_convergence, karto_ate(karto_first_convergence_index), 'o', 'MarkerSize', 10);
    hold on;
    plot(gmap_time_to_convergence, gmapping_ate(gmap_first_convergence_index), '<', 'MarkerSize', 10);
    xlabel('Time (s)');
    ylabel('ATE (m)');
    title('ATE vs Time');
    legend({'ATE', 'TTC'});
    
    disp('The displayed plots shows ATE and TTC')

%% Quit the program
  case 'Quit'
%   Quit the program
        return        
    otherwise
%   User did not select a valid option
        disp('Invalid selection')

        end
    end
end    


