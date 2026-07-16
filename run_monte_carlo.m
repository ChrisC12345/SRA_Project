function R = run_monte_carlo(arm)
%RUN_MONTE_CARLO  Monte Carlo batch for ONE controller arm (label-mode variant).
%   R = run_monte_carlo("LQR")   % then "SMC", then "PID" / "HYBRID"
%
%   Runs N stochastic-wind trials under the Dryden turbulence model for the
%   chosen arm, extracts per-run position-tracking RMSE, and reports the
%   ensemble VARIANCE of RMSE (the headline metric). Per-arm .mat output.
%
%   Reconcile with your model:
%     * MODEL          top-level .slx name
%     * CTRL_BLOCK     full path to the Variant Subsystem (the "Controller" block)
%     * ARM2LABEL      arm name -> exact Variant control label from the dialog
%     * 'dryden_seed' Dryden block "Noise seeds [ug vg wg pg]" field
%     * logged signals 'pos'+'ref' must each be a 3-element ENU position wire

if nargin < 1 || arm == ""
    error('Specify an arm, e.g. run_monte_carlo("LQR").');
end
arm = string(arm);

%% ---- config ----
MODEL      = 'simulation';
CTRL_BLOCK = 'simulation/Controller';   % <-- full path to the Variant Subsystem
N          = 5;
Tend       = 60;
baseSeed   = 23456;

% Map arm name -> the exact "Variant control label" shown in the block dialog.
% (Labels are punctuation-sensitive: hyphen vs underscore matters.)
ARM2LABEL = containers.Map( ...
    {'LQR',            'SMC',            'PID', 'Hybrid'}, ...
    {'LQR-Controller', 'SMC_Controller', 'PID-Controller', 'LQR-SMC-Hybrid-Controller'});

if ~isKey(ARM2LABEL, char(arm))
    error('Unknown arm "%s". Known: %s', arm, strjoin(keys(ARM2LABEL), ', '));
end
label = ARM2LABEL(char(arm));

%% ---- reproducible per-run Dryden seeds + wind headings ----
% Seeded per-arm so each arm sees the SAME turbulence realizations AND wind
% directions (paired comparison) rather than independent draws.
rng(baseSeed, 'twister');
seeds    = randi([1, 1e6], N, 4);      % [ug vg wg pg] per run
wind_dir = 360*rand(N, 1);            % mean-wind heading [rad], uniform 0..2pi
load_system(MODEL);
disp(seeds(2,:));
disp(wind_dir(2));
in(1:N) = Simulink.SimulationInput(MODEL);
parfor k = 1:N
    in(k) = in(k).setBlockParameter(CTRL_BLOCK, 'LabelModeActiveChoice', label);
    in(k) = in(k).setVariable('dryden_seed', seeds(k,:));
    %disp(seeds(k,:))
    in(k) = in(k).setVariable('wind_dir',    wind_dir(k));
    in(k) = in(k).setModelParameter('StopTime',    num2str(Tend));
    in(k) = in(k).setModelParameter('FastRestart', 'on');
end

out = parsim(in, ...
    'ShowProgress',                  'on', ...
    'TransferBaseWorkspaceVariables','on', ...
    'StopOnError',                   'off');

rmse  = nan(N,1);
rms_u = nan(N,1); % NEW: Control Effort
tv_u  = nan(N,1); % NEW: Chattering
nFail = 0;

for k = 1:N
    if ~isempty(out(k).ErrorMessage)
        nFail = nFail + 1;
        warning('Run %d (%s) failed: %s', k, arm, out(k).ErrorMessage);
        continue;
    end
    try
        rmse(k) = rmseFromOut(out(k));
        [rms_u(k), tv_u(k)] = ctrlMetricsFromOut(out(k)); % NEW
    catch ME
        nFail = nFail + 1;
        warning('Run %d (%s) extraction failed: %s', k, arm, ME.message);
    end
end

R.arm      = arm;
R.label    = label;
R.rmse     = rmse;
R.meanRMSE = mean(rmse, 'omitnan');
R.varRMSE  = var(rmse,  'omitnan');   
R.meanRMS_u = mean(rms_u, 'omitnan'); % NEW
R.meanTV_u  = mean(tv_u, 'omitnan');  % NEW
R.nFail    = nFail;

%% ---- report ----
fprintf('\n=== Monte Carlo: %s (%s), %d runs, Dryden severe ===\n', arm, label, N);
fprintf('  mean(RMSE) = %.4f m\n', R.meanRMSE);
fprintf('  var(RMSE)  = %.4e m^2\n', R.varRMSE);
fprintf('  mean(Ctrl Effort RMS) = %.4f\n', R.meanRMS_u); % NEW
fprintf('  mean(Chattering TV)   = %.4f\n', R.meanTV_u);  % NEW
fprintf('  fails      = %d\n', R.nFail);

fname = sprintf('mc_results_%s.mat', arm);
save(fname, 'R', 'seeds', 'wind_dir', 'N', 'Tend', 'baseSeed', 'rms_u', 'tv_u'); % UPDATED
fprintf('\nSaved -> %s\n', fname);

%% ---- plots: trajectories of all runs + per-run RMSE ----
% Figure 1: position error magnitude vs time, every successful run overlaid
figure('Name', sprintf('MC trajectories - %s', arm));
hold on; grid on;
for k = 1:N
    if ~isempty(out(k).ErrorMessage), continue; end
    try
        [t, emag] = errMagFromOut(out(k));
        plot(t, emag, 'DisplayName', sprintf('run %d', k));
    catch
        % extraction already warned above; skip plot for this run
    end
end
xlabel('Time (s)');
ylabel('||position error|| (m)');
title(sprintf('%s: position error, all runs (N = %d)', arm, N));
legend('Location', 'best');
hold off;

% Figure 2: RMSE per run
ok = ~isnan(rmse);
figure('Name', sprintf('MC RMSE - %s', arm));
hold on; grid on;
stem(find(ok), rmse(ok), 'filled', 'DisplayName', 'per-run RMSE');
yline(R.meanRMSE, '--', sprintf('mean = %.3f m', R.meanRMSE), ...
    'DisplayName', 'mean', 'LabelHorizontalAlignment', 'left');
if any(~ok)
    plot(find(~ok), zeros(nnz(~ok),1), 'rx', 'MarkerSize', 10, ...
        'LineWidth', 1.5, 'DisplayName', 'failed run');
end
xlim([0, N+1]);
xticks(1:N);
xlabel('Run index');
ylabel('Position RMSE (m)');
title(sprintf('%s: per-run RMSE, var = %.3e m^2', arm, R.varRMSE));
legend('Location', 'best');
hold off;
end


function [t, emag] = errMagFromOut(so)
%ERRMAGFROMOUT  Time vector and position-error magnitude for one run.
pos_ts = getElement(so.logsout, 'pos').Values;
ref_ts = getElement(so.logsout, 'ref').Values;

t   = pos_ts.Time(:);
T   = numel(t);
pos = coerceT3(pos_ts.Data, T, 'pos');

rD = ref_ts.Data;
if numel(rD) == 3
    ref = repmat(reshape(rD, 1, 3), T, 1);
else
    tr  = ref_ts.Time(:);
    ref = coerceT3(rD, numel(tr), 'ref');
    if numel(tr) ~= T || any(tr ~= t)
        ref = interp1(tr, ref, t, 'linear', 'extrap');
    end
end

emag = sqrt(sum((pos - ref).^2, 2));   % [T x 1]
end


function r = rmseFromOut(so)
%RMSEFROMOUT  Per-run position-tracking RMSE (trapz-weighted for variable step).
%   Requires two named, logged signals: 'pos' and 'ref', each a 3-element
%   ENU position wire. Any array layout is accepted; ref is resampled onto
%   pos's time grid if their logging rates differ. Guaranteed scalar output.

[t, emag] = errMagFromOut(so);
T  = numel(t);
sq = emag.^2;
if T > 1
    ms = trapz(t, sq) / (t(end) - t(1));
else
    ms = mean(sq);
end
r = sqrt(ms);
assert(isscalar(r) && isfinite(r), ...
    'rmseFromOut:notScalar', 'RMSE came out %s', mat2str(size(r)));
end


function D = coerceT3(D, T, name)
%COERCET3  Coerce logged data to [T x 3] or fail with a diagnostic.
%   Handles [T x 3], [3 x T], [3 x 1 x T], [1 x 3 x T], [T x 1 x 3], etc.
origSize = size(D);
D = squeeze(D);

if ~ismatrix(D)
    error('coerceT3:tooManyDims', ...
        ['Signal ''%s'' logged as %s -> squeezed %s: still >2-D. ' ...
         'The wire carries more than one 3-vector. Check the Mux/Bus ' ...
         'Selector feeding the ''%s'' signal name.'], ...
        name, mat2str(origSize), mat2str(size(D)), name);
end

% Orient so rows = time
if size(D,1) ~= T && size(D,2) == T
    D = D.';
end

if size(D,1) ~= T
    error('coerceT3:timeMismatch', ...
        'Signal ''%s'': data %s does not align with %d time samples.', ...
        name, mat2str(size(D)), T);
end
if size(D,2) ~= 3
    error('coerceT3:notThreeChannels', ...
        ['Signal ''%s'' has %d channels, expected 3 (ENU position). ' ...
         'Logged size was %s. The named wire is carrying the wrong ' ...
         'signals - fix the Mux/Bus Selector so only [x y z] pass through.'], ...
        name, size(D,2), mat2str(origSize));
end
end

function [rms_u, tv_u] = ctrlMetricsFromOut(so)
%CTRLMETRICSFROMOUT Calculates RMS control effort and Total Variation (chattering).
%   Requires a logged signal named 'u' representing actuator commands.

% 1. Extract the control signal
u_ts = getElement(so.logsout, 'u').Values;
t = u_ts.Time(:);
u_data = u_ts.Data;
T = numel(t);

% Safely reshape data to [Time x Channels]
if size(u_data, 1) ~= T && size(u_data, 2) == T
    u_data = u_data.';
elseif size(u_data, 1) ~= T
    u_data = reshape(u_data, [], T).';
end

% 2. RMS Control Effort (Energy)
sq_u = sum(u_data.^2, 2); % Sum power across all actuator channels
if T > 1
    ms = trapz(t, sq_u) / (t(end) - t(1));
else
    ms = mean(sq_u);
end
rms_u = sqrt(ms);

% 3. Total Variation (Chattering / Wear)
% Sum of absolute differences step-to-step, summed across channels
tv_u = sum(sum(abs(diff(u_data, 1, 1))));
end

