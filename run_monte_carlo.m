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
%     * logged signals 'pos'+'ref' (ENU 3-vecs) or a single 'pos_err'

if nargin < 1 || arm == ""
    error('Specify an arm, e.g. run_monte_carlo("LQR").');
end
arm = string(arm);

%% ---- config ----
MODEL      = 'simulation';
CTRL_BLOCK = 'simulation/Controller';   % <-- full path to the Variant Subsystem
N          = 2;
Tend       = 2;
baseSeed   = 12345;

% Map arm name -> the exact "Variant control label" shown in the block dialog.
% (Labels are punctuation-sensitive: hyphen vs underscore matters.)
ARM2LABEL = containers.Map( ...
    {'LQR',            'SMC',            'PID'}, ...
    {'LQR-Controller', 'SMC_Controller', 'PID-Controller'});

if ~isKey(ARM2LABEL, char(arm))
    error('Unknown arm "%s". Known: %s', arm, strjoin(keys(ARM2LABEL), ', '));
end
label = ARM2LABEL(char(arm));

%% ---- reproducible per-run Dryden seeds ----
% Reset per call so every arm sees the SAME turbulence realizations (paired).
rng(baseSeed, 'twister');
seeds = randi([1, 1e6], N, 4);     % [ug vg wg pg] per run

load_system(MODEL);

in(1:N) = Simulink.SimulationInput(MODEL);
for k = 1:N
    in(k) = in(k).setBlockParameter(CTRL_BLOCK, 'LabelModeActiveChoice', label);
    in(k) = in(k).setVariable('dryden_seed', seeds(k,:));
    in(k) = in(k).setModelParameter('StopTime',    num2str(Tend));
    in(k) = in(k).setModelParameter('FastRestart', 'on');
end

out = parsim(in, ...
    'ShowProgress',                  'on', ...
    'TransferBaseWorkspaceVariables','on', ...
    'StopOnError',                   'off');

rmse  = nan(N,1);
nFail = 0;
for k = 1:N
    if ~isempty(out(k).ErrorMessage)
        nFail = nFail + 1;
        warning('Run %d (%s) failed: %s', k, arm, out(k).ErrorMessage);
        continue;
    end
    rmse(k) = rmseFromOut(out(k));
end

R.arm      = arm;
R.label    = label;
R.rmse     = rmse;
R.meanRMSE = mean(rmse, 'omitnan');
R.varRMSE  = var(rmse,  'omitnan');   % <-- headline metric
R.nFail    = nFail;

%% ---- report ----
fprintf('\n=== Monte Carlo: %s (%s), %d runs, Dryden severe ===\n', arm, label, N);
fprintf('  mean(RMSE) = %.4f\n', R.meanRMSE);
fprintf('  var(RMSE)  = %.4e   <-- headline\n', R.varRMSE);
fprintf('  fails      = %d\n', R.nFail);

fname = sprintf('mc_results_%s.mat', arm);
save(fname, 'R', 'seeds', 'N', 'Tend', 'baseSeed');
fprintf('\nSaved -> %s\n', fname);
end


function r = rmseFromOut(so)
%RMSEFROMOUT  Per-run position-tracking RMSE (trapz-weighted for variable step).
%   Requires two named, logged signals in the model:
%     'pos' : actual ENU position, single 3-element wire (Mux'd, not a bus)
%     'ref' : commanded ENU position, single 3-element wire
pos = getElement(so.logsout, 'pos').Values.Data;   % any shape
ref = getElement(so.logsout, 'ref').Values.Data;
t   = getElement(so.logsout, 'pos').Values.Time;
t   = t(:);                                        % force column

pos = shapeTx3(pos, numel(t));                     % -> [T x 3]
if numel(ref) == 3
    ref = reshape(ref, 1, 3);                      % constant setpoint
else
    ref = shapeTx3(ref, numel(t));
end

e  = pos - ref;                 % [1x3] ref broadcasts over rows
sq = sum(e.^2, 2);
if numel(t) > 1
    ms = trapz(t, sq) / (t(end) - t(1));
else
    ms = mean(sq);
end
r = sqrt(ms);
end


function D = shapeTx3(D, T)
%SHAPETX3  Coerce logged 3-vector data of any layout to [T x 3].
D = squeeze(D);                 % [3x1xT] -> [3xT], [1x3xT] -> [3xT]
if isvector(D)
    D = D(:);                   % degenerate single-channel case
elseif size(D,1) ~= T && size(D,2) == T
    D = D.';                    % [3xT] -> [Tx3]
end
if size(D,1) ~= T
    error('shapeTx3:mismatch', ...
        'Logged data has %d rows but time has %d samples.', size(D,1), T);
end
end