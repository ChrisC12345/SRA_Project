%% --- Post-Process Monte Carlo Data ---
clear; clc;

% 1. Load your saved .mat file
filename = 'mc_results_PID.mat'; % <-- Update this to your exact filename
load(filename);

fprintf('Loaded data from: %s\n', filename);

% 2. Define your outlier threshold
% Looking at your plot, 20 meters is a safe cutoff for a "normal" run
threshold = 2; 

% 3. Extract the raw RMSE data (saved inside your R struct)
raw_rmse = R.rmse;
total_runs = length(raw_rmse);

% 4. Find the indices of the clean, normal runs
valid_idx = ~isnan(raw_rmse) & (raw_rmse < threshold);
num_clean = sum(valid_idx);
num_fails = total_runs - num_clean;

% 5. Recalculate all the means using ONLY the valid runs
clean_mean_rmse = mean(raw_rmse(valid_idx));
clean_var_rmse  = var(raw_rmse(valid_idx));

% Use the raw arrays that were saved to the .mat file to get clean averages
clean_mean_rms_F   = mean(rms_F(valid_idx));
clean_mean_rms_tau = mean(rms_tau(valid_idx));
clean_mean_tv_F    = mean(tv_F(valid_idx));
clean_mean_tv_tau  = mean(tv_tau(valid_idx));

% 6. Print the beautiful, outlier-free results
fprintf('\n=== CLEAN RESULTS (Outliers Excluded) ===\n');
fprintf('  Total Runs           = %d\n', total_runs);
fprintf('  Clean Runs Kept      = %d\n', num_clean);
fprintf('  Extreme Outliers Cut = %d\n', num_fails);
fprintf('  ------------------------------------\n');
fprintf('  Clean Mean RMSE      = %.4f m\n', clean_mean_rmse);
fprintf('  Clean Var RMSE       = %.4e m^2\n', clean_var_rmse);
fprintf('  Clean RMS Thrust     = %.4f N\n', clean_mean_rms_F);
fprintf('  Clean RMS Torque     = %.4f N-m\n', clean_mean_rms_tau);
fprintf('  Clean TV Thrust      = %.4f \n', clean_mean_tv_F);
fprintf('  Clean TV Torque      = %.4f \n', clean_mean_tv_tau);
fprintf('=========================================\n');