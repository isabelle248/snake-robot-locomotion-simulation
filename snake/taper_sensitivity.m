% taper_sensitivity.m -- sensitivity of the forward displacement to the taper coefficient
% of the aquatic gait (the 0.25 in A_s = A*(1 + c*s)/(1 + c)), +-25 %.
% Run from the MAT-DiSMech root folder.
%
% Only the aquatic gait (loco_type = 2) uses the taper, so only two cases are run:
% aquatic gait / land and aquatic gait / water. For each, the baseline gait and the best
% gait from joint_sweep.m (joint_sweep_case3.mat, joint_sweep_case4.mat) are run at
% c = 0.75*0.25, 0.25 and 1.25*0.25.
%
% A NaN result means the gait is invalid (e.g. the simulated body touched itself).

clc; close all;
addpath springs/ util_functions/ contact_functions/ rod_dynamics/ shell_dynamics/ external_forces/ adaptive_stepping/ logging/
addpath(genpath('experiments'));

src = fileread(which('custom_main_snake.m'));
if ~contains(src, 'opts.taper')
    error('Add the opts.taper lines to custom_main_snake.m (and the taper argument to actuate_snake.m) first.');
end

%% Settings
freq  = 1.825;                    % Hz
phase = pi/2;
A_base = 0.1; L_base = 0.4;       % baseline gait
c_nom = 0.25;                     % nominal taper coefficient
rel   = 0.25;                     % +-25 %
c_vals = c_nom*[1 - rel, 1, 1 + rel];

loco = 2;                         % aquatic gait only
envs = [1 2];                     % 1 = land, 2 = water
case_ids = [3 4];                 % joint_sweep case numbers for aquatic gait / land and / water
case_names = {'Aquatic gait / land', 'Aquatic gait / water'};

%% Runs
rows = struct('case', {}, 'gait', {}, 'A', {}, 'lambda', {}, 'c_low', {}, 'c_nom', {}, 'c_high', {}, ...
              'D_low', {}, 'D_nom', {}, 'D_high', {}, 'pct_low', {}, 'pct_high', {}, 'norm_sens', {});
t0 = tic;
for k = 1:2
    envt = envs(k);
    gaits = {'baseline', [A_base L_base]};
    fn = sprintf('joint_sweep_case%d.mat', case_ids(k));
    if isfile(fn)
        S = load(fn, 'A_opt', 'L_opt');
        gaits(end+1,:) = {'best', [S.A_opt S.L_opt]};   %#ok<SAGROW>
    else
        fprintf('%s not found: only the baseline gait is run for %s\n', fn, case_names{k});
    end

    for g = 1:size(gaits,1)
        A0 = gaits{g,2}(1); L0 = gaits{g,2}(2);
        D = nan(1,3);
        for j = 1:3
            opts = struct('taper', c_vals(j));
            try
                D(j) = custom_main_snake(A0, freq, L0, phase, loco, envt, opts);
            catch err
                fprintf('Run failed (%s, c = %g): %s\n', case_names{k}, c_vals(j), err.message);
            end
        end
        r.case = case_names{k}; r.gait = gaits{g,1}; r.A = A0; r.lambda = L0;
        r.c_low = c_vals(1); r.c_nom = c_vals(2); r.c_high = c_vals(3);
        r.D_low = D(1); r.D_nom = D(2); r.D_high = D(3);
        r.pct_low  = 100*(D(1) - D(2))/D(2);
        r.pct_high = 100*(D(3) - D(2))/D(2);
        % normalised sensitivity (central difference): (dD/D)/(dc/c)
        r.norm_sens = ((D(3) - D(1))/D(2)) / ((c_vals(3) - c_vals(1))/c_vals(2));
        rows(end+1) = r; %#ok<SAGROW>
        fprintf('%-20s %-8s (A=%.4f, lambda=%.3f): c=%.4f %.4f m | c=%.4f %.4f m | c=%.4f %.4f m | S = %+.3f\n', ...
            case_names{k}, gaits{g,1}, A0, L0, c_vals(1), D(1), c_vals(2), D(2), c_vals(3), D(3), r.norm_sens);
    end
    fprintf('--- %.1f min so far\n', toc(t0)/60);
end

T = struct2table(rows);
writetable(T, 'taper_sensitivity.csv');
save('taper_sensitivity.mat', 'T', 'c_vals', 'rel', 'freq', 'phase');
disp(T);
