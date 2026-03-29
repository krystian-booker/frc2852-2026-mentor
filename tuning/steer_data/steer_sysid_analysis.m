%% steer_sysid_analysis.m
% Swerve Steer (Azimuth) System Identification & PID Auto-Tuning
%
% Loads CSV data from SteerSysIdCommand and computes feedforward and PID
% position constants ready to paste into TunerConstants.java Steer Gains.
%
% All outputs are for a Position-based closed-loop steer drive:
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
file_qs    = 'steer_sysid_quasistatic.csv';
file_steps = 'steer_sysid_steps.csv';
file_coast = 'steer_sysid_coast.csv';

% Quasistatic filtering
QS_MIN_VELOCITY_RPS   = 0.1;   % rot/s - Ignore data below this speed (stiction noise)
QS_ACCEL_THRESHOLD    = 0.5;   % rot/s^2 - max accel to count as steady-state
QS_SMOOTH_WINDOW      = 11;    % Moving average window for derivative smoothing

% PID tuning targets (Position)
PID_BANDWIDTH         = 30;    % rad/s - closed-loop bandwidth target (Steer needs to be fast!)
PID_PHASE_MARGIN      = 60;    % degrees - phase margin target

%% ===================== LOAD DATA =====================
function [t, vel, motor_v, iter_Ts, cmd_type, cmd_value] = loadTestData(filename)
    try
        data = readtable(filename);
        t_raw = data.timestamp;
        t = t_raw - t_raw(1);
        vel = data.steer_velocity_rps;
        motor_v = data.steer_voltage;
        
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
        fprintf('  Relaxing threshold...\n');
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
    hold_mask = vs_ct == 0;
    segments = diff([0; hold_mask; 0]);
    seg_starts = find(segments == 1);
    seg_ends   = find(segments == -1) - 1;
    n_segments = length(seg_starts);

    fprintf('  Found %d voltage step segments\n', n_segments);

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
        fprintf('  ERROR: Not enough step points to calculate TF.\n');
    end
end

%% ===================== PID AUTO-TUNING (POSITION) =====================
fprintf('\n=== PID AUTO-TUNING (For Steer Position Output) ===\n');

kP_pid = 0; kI_pid = 0; kD_pid = 0;
if ~isempty(sys_vel_plant)
    % Transform Velocity Plant to Position Plant using integrator G_pos(s) = G_vel(s) * (1/s)
    sys_pos_plant = sys_vel_plant * tf(1, [1 0]);
    
    opts = pidtuneOptions('PhaseMargin', PID_PHASE_MARGIN);
    [C_pid, info_pid] = pidtune(sys_pos_plant, 'PID', PID_BANDWIDTH, opts);
    kP_pid = C_pid.Kp;
    kI_pid = C_pid.Ki;
    kD_pid = C_pid.Kd;

    fprintf('  POSITION PID result:\n');
    fprintf('    kP = %.4f V/rot\n', kP_pid);
    fprintf('    kI = %.4f V/(rot*s)\n', kI_pid);
    fprintf('    kD = %.4f V/(rot/s)\n', kD_pid);
    fprintf('    Achieved PM: %.1f deg\n', info_pid.PhaseMargin);
else
    fprintf('  Skipping PID calc due to missing TF model.\n');
end

%% ===================== FINAL RESULTS =====================
kS_final = kS_qs;
kV_final = kV_qs;
kA_final = kA_model;
kP_final = kP_pid;
kI_final = kI_pid;
kD_final = kD_pid;

fprintf('\n');
fprintf('================================================================\n');
fprintf('   FINAL GAINS - CTRE TunerConstants (Steer Gains)             \n');
fprintf('================================================================\n');
fprintf('   Feedforward                                                  \n');
fprintf('     S (kS) = %8.4f   Volts\n', kS_final);
fprintf('     V (kV) = %8.4f   Volts / (rot/s)\n', kV_final);
fprintf('     A (kA) = %8.4f   Volts / (rot/s^2)\n', kA_final);
fprintf('   PID (Position-Based)                                          \n');
fprintf('     P (kP) = %8.4f   Volts / rot\n', kP_final);
fprintf('     I (kI) = %8.4f   Volts / (rot*sec)\n', kI_final);
fprintf('     D (kD) = %8.4f   Volts / (rot/sec)\n', kD_final);
fprintf('================================================================\n');

fprintf('\n// --- Copy into TunerConstants.java steerGains ---\n');
fprintf('private static final Slot0Configs steerGains = new Slot0Configs()\n');
fprintf('    .withKP(%.4f).withKI(%.4f).withKD(%.4f)\n', kP_final, kI_final, kD_final);
fprintf('    .withKS(%.4f).withKV(%.4f).withKA(%.4f)\n', kS_final, kV_final, kA_final);
fprintf('    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);\n');
