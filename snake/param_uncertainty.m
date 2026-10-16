
% param_uncertainty.m --  Cn, Ct (water drag) and mu_t, mu_n (land friction)
% Run from the MAT-DiSMech root folder.
%
% Cn, Ct    : only used in water -> run for land gait / water and aquatic gait / water
% mu_t, mu_n: only used on land  -> run for land gait / land  and aquatic gait / land
% Each case is run at the best gait whose amplitude, wavelength and frequency are set
% by hand in the table below.
% Total: 4 nominal + 16 perturbed runs.
%
% A NaN result means the run failed or the gait was invalid; it is not a script error.
 
clear; clc; close all;
addpath springs/ util_functions/ contact_functions/ rod_dynamics/ shell_dynamics/ external_forces/ adaptive_stepping/ logging/
addpath(genpath('experiments'));
 
src = fileread(which('custom_main_snake.m'));
if ~contains(src, 'opts.Cn') || ~contains(src, 'opts.Ct')
    error('Add the opts.Cn / opts.Ct override lines to custom_main_snake.m first (see header).');
end
 
%% Settings
phase = pi/2;
rel = 0.25;             
 
% One row per case: [loco_type env_type amplitude wavelength frequency]
%   loco_type: 1 = land gait, 2 = aquatic gait;  env_type: 1 = land, 2 = water
%   wavelength is in body lengths, frequency in Hz
best = [1 1  0.3602  1.2  1.825;      % land gait    / land
        1 2  0.2041  2.0  1.825;      % land gait    / water
        2 1  0.4122  1.2  1.825;      % aquatic gait / land
        2 2  0.2041  2.0  1.825];     % aquatic gait / water
case_names = {'Land gait / land', 'Land gait / water', 'Aquatic gait / land', 'Aquatic gait / water'};
 
% name (opts field), nominal value, environment where it is used (1 = land, 2 = water)
P = struct('name',    {'Cn', 'Ct',  'mu_t', 'mu_n'}, ...
           'nominal', {1.0,  0.02,  0.11,   0.20}, ...
           'env',     {2,    2,     1,      1});
 
%% Runs
rows = struct('case_name', {}, 'A', {}, 'lambda', {}, 'freq', {}, 'param', {}, ...
              'val_low', {}, 'val_nom', {}, 'val_high', {}, ...
              'D_low', {}, 'D_nom', {}, 'D_high', {}, ...
              'pct_low', {}, 'pct_high', {}, 'norm_sens', {});
t0 = tic;
for c = 1:size(best, 1)
    loco = best(c,1); envt = best(c,2);
    A0 = best(c,3); L0 = best(c,4); freq = best(c,5);
 
    D_nom = run_one('', NaN, A0, L0, freq, phase, loco, envt);
    fprintf('%s (A=%.4f, lambda=%.3f, f=%.3f Hz): nominal %.4f m\n', case_names{c}, A0, L0, freq, D_nom);
 
    for p = 1:numel(P)
        if P(p).env ~= envt, continue; end
        v = P(p).nominal*[1 - rel, 1, 1 + rel];
        D_lo = run_one(P(p).name, v(1), A0, L0, freq, phase, loco, envt);
        D_hi = run_one(P(p).name, v(3), A0, L0, freq, phase, loco, envt);
 
        r = struct();
        r.case_name = case_names{c}; r.A = A0; r.lambda = L0; r.freq = freq; r.param = P(p).name;
        r.val_low = v(1); r.val_nom = v(2); r.val_high = v(3);
        r.D_low = D_lo; r.D_nom = D_nom; r.D_high = D_hi;
        r.pct_low  = 100*(D_lo - D_nom)/D_nom;
        r.pct_high = 100*(D_hi - D_nom)/D_nom;
        % normalised sensitivity (central difference): (dD/D)/(dp/p)
        r.norm_sens = ((D_hi - D_lo)/D_nom) / ((v(3) - v(1))/v(2));
        rows(end+1) = r; 
        fprintf('   %-5s %.4g: %.4f m (%+.2f %%) | %.4g: %.4f m (%+.2f %%) | S = %+.3f\n', ...
            P(p).name, v(1), D_lo, r.pct_low, v(3), D_hi, r.pct_high, r.norm_sens);
    end
    fprintf('--- %.1f min so far\n', toc(t0)/60);
end
 
T = struct2table(rows);
writetable(T, 'param_uncertainty.csv');
save('param_uncertainty.mat', 'T', 'P', 'best', 'rel', 'phase');
disp(T);
 
%% Quick plot: normalised sensitivity of each parameter, per case
figure; hold on;
labels = strcat(string(T.case_name), " | ", string(T.param));
barh(categorical(labels, labels), T.norm_sens);
xlabel('Normalised sensitivity (dD/D)/(dp/p)'); grid on;
title(sprintf('Sensitivity to +-%g %% change in each parameter', 100*rel));
 
%% ----------------------------------------------------------------------
function D = run_one(name, val, A0, L0, freq, phase, loco, envt)
% One simulation. name = '' runs the nominal case. NaN is returned if the run fails.
    opts = struct();
    if ~isempty(name), opts.(name) = val; end
    try
        D = custom_main_snake(A0, freq, L0, phase, loco, envt, opts);
    catch err
        fprintf('Run failed (%s = %g): %s\n', name, val, err.message);
        D = NaN;
    end
end
 