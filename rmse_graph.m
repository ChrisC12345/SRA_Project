lqr = load('mc_results_LQR.mat');
smc = load('mc_results_SMC.mat');
hyb = load('mc_results_Hybrid.mat');

r_lqr = lqr.R.rmse;
r_smc = smc.R.rmse;
r_hyb = hyb.R.rmse;

c = [0 0.447 0.741; 0.85 0.325 0.098; 0.929 0.694 0.125; 0.494 0.184 0.556];
names = {'LQR','SMC','Hybrid'};
arms  = {r_lqr, r_smc, r_hyb};

figure; hold on
for i = 1:3
    grp = categorical(repmat(names(i), numel(arms{i}), 1), names);
    b = boxchart(grp, arms{i}(:));
    b.BoxFaceColor = c(i,:);
    b.MarkerColor  = c(i,:);
    plot(i, mean(arms{i}), 'x', 'Color', c(i,:), ...
        'MarkerSize', 12, 'LineWidth', 2)
end
ylabel('Position RMSE (m)'), grid on
title('RMSE distribution by controller')