%% turret_sysid_analysis.m
% Turret System Identification & PID Auto-Tuning (Position-Based)
%
% Loads CSV data from TurretSysIdCommand and computes feedforward and PID
% position constants ready to paste into Constants.java TurretConstants.
%
% The turret uses PositionVoltage control (50:1 geared Kraken X60).
% This script identifies a velocity-domain plant from voltage step data,
% then multiplies by an integrator (1/s) to get the position plant for
% PID tuning — the same approach used for swerve steer tuning.
%
% All outputs are for a Position-based closed-loop controller:
%   kS: Volts                 (static friction feedforward)
%   kV: Volts / (rot/s)       (velocity feedforward)
%   kA: Volts / (rot/s^2)     (acceleration feedforward)
%   kP: Volts / rotation      (proportional gain)
%   kI: Volts / (rot*sec)     (integral gain)
%   kD: Volts / (rot/sec)     (derivative gain)
%
% Requires: System Identification Toolbox, Control System Toolbox

clear; clc; close all;

%% ===================== USER CONFIGURATION =====================
% Replace these with the actual generated filenames from your RoboRIO:
file_qs    = 'turret_sysid_quasistatic.csv';
file_steps = 'turret_sysid_steps.csv';
file_coast = 'turret_sysid_coast.csv';

% Quasistatic filtering
QS_MIN_VELOCITY_RPS   = 0.1;   % rot/s - Ignore data below this speed (stiction noise)
QS_ACCEL_THRESHOLD    = 0.5;   % rot/s^2 - max accel to count as steady-state
QS_SMOOTH_WINDOW      = 11;    % Moving average window for derivative smoothing

% Existing Constants.java values for comparison
EXISTING_KS = 1.2;
EXISTING_KV = 4.8862;
EXISTING_KA = 0.1000;
EXISTING_KP = 80.0000;
EXISTING_KI = 1.0;
EXISTING_KD = 1.5;

%% ===================== LOAD DATA =====================
function [t, vel, motor_v, iter_Ts, cmd_type, cmd_value] = loadTestData(filename)
    try
        data = readtable(filename);
        t_raw = data.timestamp;
        t = t_raw - t_raw(1);
        vel = data.turret_velocity_rps;
        motor_v = data.motor_voltage;

        if ismember('command_type', data.Properties.VariableNames)
            cmd_type = data.command_type;
            cmd_value = data.command_value;
        else
            cmd_type = zeros(size(t));
            cmd_value = zeros(size(t));
        end

        iter_Ts = median(diff(t));
        fprintf('Loaded %s successfully: %d samples, %.1f seconds.\n', filename, height(data), t(end));
    catch
        fprintf('WARNING: Could not load %s. Skipping this telemetry.\n', filename);
        t = []; vel = []; motor_v = []; iter_Ts = 0.02;
        cmd_type = []; cmd_value = [];
    end
end

[qs_t, qs_vel, qs_v, Ts_qs] = loadTestData(file_qs);
[vs_t, vs_vel, vs_v, Ts_vs, vs_ct, vs_cmd] = loadTestData(file_steps);
[cd_t, cd_vel, cd_v, Ts_cd, cd_ct, cd_cmd] = loadTestData(file_coast);

Ts = max([Ts_qs, Ts_vs, Ts_cd, 0.02]);

%% ===================== FEEDFORWARD: QUASISTATIC RAMP =====================
fprintf('\n=== FEEDFORWARD (Quasistatic Ramp) ===\n');

kS_qs = 0; kV_qs = 0;
if ~isempty(qs_t)
    qs_vel_smooth = movmean(qs_vel, QS_SMOOTH_WINDOW);
    qs_accel = gradient(qs_vel_smooth, Ts);

    valid = qs_vel_smooth > QS_MIN_VELOCITY_RPS & abs(qs_accel) < QS_ACCEL_THRESHOLD;
    qs_vel_f = qs_vel_smooth(valid);
    qs_v_f   = qs_v(valid);
    n_qs     = sum(valid);

    if n_qs < 20
        fprintf('  WARNING: Only %d points below %.1f rot/s^2 accel threshold.\n', n_qs, QS_ACCEL_THRESHOLD);
        fprintf('  Relaxing threshold to 2.0 rot/s^2...\n');
        valid = qs_vel_smooth > QS_MIN_VELOCITY_RPS & abs(qs_accel) < 2.0;
        qs_vel_f = qs_vel_smooth(valid);
        qs_v_f   = qs_v(valid);
        n_qs     = sum(valid);
    end

    if (n_qs > 0)
        X_qs = [ones(n_qs, 1), qs_vel_f];
        ff_coeffs = X_qs \ qs_v_f;

        kS_qs = ff_coeffs(1);
        kV_qs = ff_coeffs(2);

        qs_v_pred = X_qs * ff_coeffs;
        R2_qs = 1 - sum((qs_v_f - qs_v_pred).^2) / sum((qs_v_f - mean(qs_v_f)).^2);

        fprintf('  Steady-state fit: V = %.4f + %.4f * vel  (R^2 = %.4f)\n', kS_qs, kV_qs, R2_qs);
        fprintf('  kS = %.4f V\n', kS_qs);
        fprintf('  kV = %.4f V/(rot/s)\n', kV_qs);
    else
        fprintf('  ERROR: Not enough points to calculate Quasistatic Feedforward.\n');
    end
end

%% ===================== SYSTEM ID: VOLTAGE STEPS =====================
fprintf('\n=== SYSTEM IDENTIFICATION (Voltage Steps) ===\n');

sys_vel_plant = []; kA_model = 0;
if ~isempty(vs_t)
    % Segment by voltage-on periods (command_type == 0 means voltage command)
    hold_mask = vs_ct == 0;
    segments = diff([0; hold_mask; 0]);
    seg_starts = find(segments == 1);
    seg_ends   = find(segments == -1) - 1;
    n_segments = length(seg_starts);

    fprintf('  Found %d voltage step segments (bidirectional)\n', n_segments);

    experiments = {};
    for k = 1:n_segments
        seg_idx = seg_starts(k):seg_ends(k);
        if length(seg_idx) < 10, continue; end

        seg_t   = vs_t(seg_idx);
        seg_vel = vs_vel(seg_idx);
        seg_v   = vs_v(seg_idx);

        seg_t_rel = seg_t - seg_t(1);
        seg_t_uni = (0 : Ts : seg_t_rel(end))';
        if length(seg_t_uni) < 10, continue; end

        seg_vel_uni = interp1(seg_t_rel, seg_vel, seg_t_uni, 'linear', 'extrap');
        seg_v_uni   = interp1(seg_t_rel, seg_v, seg_t_uni, 'linear', 'extrap');

        seg_data = iddata(seg_vel_uni, seg_v_uni, Ts);
        seg_data.InputName  = {'Voltage'};
        seg_data.InputUnit  = {'V'};
        seg_data.OutputName = {'Velocity'};
        seg_data.OutputUnit = {'rot/s'};

        experiments{end+1} = seg_data; %#ok<SAGROW>
    end

    if ~isempty(experiments)
        id_data = merge(experiments{:});

        fprintf('  Estimating transfer functions from Voltage to Velocity...\n');
        sys1 = tfest(id_data, 1);
        sys_vel_plant = sys1;

        K_dc  = dcgain(sys1);
        tau   = -1 / real(pole(sys1));

        kV_model = 1 / K_dc;
        kA_model = tau / K_dc;

        fprintf('\n  1st-order VELOCITY model parameters:\n');
        fprintf('    DC gain K = %.4f (rot/s)/V\n', K_dc);
        fprintf('    Time constant tau = %.4f s\n', tau);
        fprintf('    -> kV = 1/K = %.4f V/(rot/s)\n', kV_model);
        fprintf('    -> kA = tau/K = %.4f V/(rot/s^2)\n', kA_model);
    else
        fprintf('  ERROR: Not enough step segments for transfer function estimation.\n');
    end
end

%% ===================== PID AUTO-TUNING (POSITION) =====================
fprintf('\n=== PID AUTO-TUNING (Position-Based, Multiple Profiles) ===\n');

% Define tuning profiles: [bandwidth_rad_s, phase_margin_deg, label]
profiles = {
    30,  50,  'Fast (competition - fastest settling)';
    25,  55,  'Balanced (recommended starting point)';
    15,  65,  'Conservative (if oscillation occurs)';
};

kP_results = zeros(size(profiles, 1), 1);
kI_results = zeros(size(profiles, 1), 1);
kD_results = zeros(size(profiles, 1), 1);

if ~isempty(sys_vel_plant)
    % Transform Velocity Plant to Position Plant using integrator
    % G_pos(s) = G_vel(s) * (1/s)
    sys_pos_plant = sys_vel_plant * tf(1, [1 0]);

    for p = 1:size(profiles, 1)
        bw  = profiles{p, 1};
        pm  = profiles{p, 2};
        lbl = profiles{p, 3};

        opts = pidtuneOptions('PhaseMargin', pm);
        [C_pid, info_pid] = pidtune(sys_pos_plant, 'PID', bw, opts);

        kP_results(p) = C_pid.Kp;
        kI_results(p) = C_pid.Ki;
        kD_results(p) = C_pid.Kd;

        fprintf('\n  --- %s ---\n', lbl);
        fprintf('    Bandwidth: %d rad/s, Phase Margin target: %d deg\n', bw, pm);
        fprintf('    kP = %.4f V/rot\n', C_pid.Kp);
        fprintf('    kI = %.4f V/(rot*s)\n', C_pid.Ki);
        fprintf('    kD = %.4f V/(rot/s)\n', C_pid.Kd);
        fprintf('    Achieved PM: %.1f deg\n', info_pid.PhaseMargin);
    end
else
    fprintf('  Skipping PID calc due to missing TF model.\n');
end

%% ===================== POSITION STEP RESPONSE SIMULATION =====================
if ~isempty(sys_vel_plant)
    fprintf('\n=== POSITION STEP RESPONSE SIMULATION ===\n');

    step_sizes_deg = [10, 45, 90, 180];
    t_sim = (0 : Ts : 2)';

    figure('Name', 'Turret Position Step Response', 'NumberTitle', 'off');

    for p = 1:size(profiles, 1)
        C_sim = pid(kP_results(p), kI_results(p), kD_results(p));
        cl_sys = feedback(C_sim * sys_pos_plant, 1);

        subplot(1, size(profiles, 1), p);
        hold on; grid on;
        title(profiles{p, 3});
        xlabel('Time (s)');
        ylabel('Position (deg)');

        for s = 1:length(step_sizes_deg)
            step_rot = step_sizes_deg(s) / 360;
            [y, t_out] = step(cl_sys * step_rot, t_sim);
            y_deg = y * 360;

            plot(t_out, y_deg, 'LineWidth', 1.5, 'DisplayName', sprintf('%d deg', step_sizes_deg(s)));

            % Compute settling time (within 2% of final value)
            final_val = step_sizes_deg(s);
            settled = abs(y_deg - final_val) < 0.02 * final_val;
            settle_idx = find(settled, 1, 'first');
            if ~isempty(settle_idx)
                settle_time = t_out(settle_idx);
            else
                settle_time = NaN;
            end

            overshoot = (max(y_deg) - final_val) / final_val * 100;

            fprintf('  [%s] %3d deg step: settle=%.3fs, overshoot=%.1f%%\n', ...
                profiles{p, 3}(1:4), step_sizes_deg(s), settle_time, overshoot);
        end
        legend('Location', 'southeast');
    end
    sgtitle('Turret Step Response - All Profiles');
end

%% ===================== FINAL RESULTS =====================
% Use balanced profile (index 2) as primary recommendation
kS_final = kS_qs;
kV_final = kV_qs;
kA_final = kA_model;
kP_final = kP_results(2);  % Balanced
kI_final = kI_results(2);
kD_final = kD_results(2);

fprintf('\n');
fprintf('================================================================\n');
fprintf('   FINAL GAINS - Constants.java TurretConstants (Balanced)      \n');
fprintf('================================================================\n');
fprintf('   Feedforward                                                  \n');
fprintf('     S (kS) = %8.4f   Volts              (was %.4f)\n', kS_final, EXISTING_KS);
fprintf('     V (kV) = %8.4f   Volts / (rot/s)    (was %.4f)\n', kV_final, EXISTING_KV);
fprintf('     A (kA) = %8.4f   Volts / (rot/s^2)  (was %.4f)\n', kA_final, EXISTING_KA);
fprintf('   PID (Position-Based)                                          \n');
fprintf('     P (kP) = %8.4f   Volts / rot        (was %.4f)\n', kP_final, EXISTING_KP);
fprintf('     I (kI) = %8.4f   Volts / (rot*sec)  (was %.4f)\n', kI_final, EXISTING_KI);
fprintf('     D (kD) = %8.4f   Volts / (rot/sec)  (was %.4f)\n', kD_final, EXISTING_KD);
fprintf('================================================================\n');

fprintf('\n// --- Copy into Constants.java TurretConstants ---\n');
fprintf('public static final double S = %.4f;\n', kS_final);
fprintf('public static final double V = %.4f;\n', kV_final);
fprintf('public static final double A = %.4f;\n', kA_final);
fprintf('public static final double P = %.4f;\n', kP_final);
fprintf('public static final double I = %.4f;\n', kI_final);
fprintf('public static final double D = %.4f;\n', kD_final);

fprintf('\n// --- Alternative: Fast profile ---\n');
fprintf('// public static final double P = %.4f;\n', kP_results(1));
fprintf('// public static final double I = %.4f;\n', kI_results(1));
fprintf('// public static final double D = %.4f;\n', kD_results(1));

fprintf('\n// --- Alternative: Conservative profile ---\n');
fprintf('// public static final double P = %.4f;\n', kP_results(3));
fprintf('// public static final double I = %.4f;\n', kI_results(3));
fprintf('// public static final double D = %.4f;\n', kD_results(3));
