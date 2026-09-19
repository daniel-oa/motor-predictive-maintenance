clc; clear; % clear command window and workspace

% import dataset
data = readtable('motor_data2.csv','VariableNamingRule','preserve');

% extract required signals
Ia = data.("Ia (A)");
Ib = data.("Ib (A)");
Ic = data.("Ic (A)");
Irms = data.("Irms (A)");
Vx = data.("Vibration_X(g)");
Vy = data.("Vibration_Y(g)");
Vz = data.("Vibration_Z(g)");
T_motor = data.("Motor Temperature(℃)");
T_amb = data.("Ambient Temperature (℃)");
Torque = data.("Torque (Nm)");

state = string(data.State); % converts state column to string

% feature calculation
% Vibration indicator: 3-axis vibration RMS
Vib_rms = sqrt((Vx.^2 + Vy.^2 + Vz.^2));

% Temperature rise indicator
DeltaT = T_motor - T_amb;

% identify healthy samples
healthy_idx = strcmpi(state, "Healthy"); 
if ~any(healthy_idx)
    error('No healthy samples were found in the State column.');
end

% initialize normalized indicators
Vib_norm = zeros(height(data),1);
Inorm = zeros(height(data),1);
Tnorm = zeros(height(data),1);

% compute healthy baseline separately for each torque level
unique_torque = unique(Torque);
for k = 1:length(unique_torque) % loops through each torque level
    this_torque = unique_torque(k);
    torque_idx = (Torque == this_torque);
    healthy_torque_idx = torque_idx & healthy_idx;

    if ~any(healthy_torque_idx)
        error('No healthy samples found for torque level %.4f Nm.', this_torque);
    end

    % healthy baselines
    V_base = mean(Vib_rms(healthy_torque_idx), 'omitnan');
    I_base = mean(Irms(healthy_torque_idx), 'omitnan');
    T_base = mean(DeltaT(healthy_torque_idx), 'omitnan');

    % protect against zero baseline values
    V_base = max(V_base, eps);
    I_base = max(I_base, eps);
    T_base = max(T_base, eps);

    % deviation from healthy baseline at the same torque level
    Vib_norm(torque_idx) = abs(Vib_rms(torque_idx) - V_base) ./ V_base;
    Inorm(torque_idx) = abs(Irms(torque_idx) - I_base) ./ I_base;
    Tnorm(torque_idx) = abs(DeltaT(torque_idx) - T_base) ./ T_base;
end

% cap extreme normalized deviations% limits extreme outliers from dominating the HI
cap_val = 5;
Vib_norm = min(Vib_norm, cap_val);
Inorm = min(Inorm, cap_val);
Tnorm = min(Tnorm, cap_val);

% health index calculation
% weights
w_v = 0.40; % vibration
w_i = 0.35; % current 
w_t = 0.25; % temperature
% raw degradation score
D = w_v .* Vib_norm + w_i .* Inorm + w_t .* Tnorm;
% convert to bounded health index
HI = 1 ./ (1 + D);

% add features to dataset
data.Vib_rms = Vib_rms;
data.TempRise = DeltaT;
data.Vib_norm = Vib_norm;
data.Inorm = Inorm;
data.Tnorm = Tnorm;
data.HI = HI;

% save updated dataset
writetable(data, 'motor_data_complete.csv');