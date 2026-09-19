clc; clear;

% parameters
ts = 0.02;
n = 3000;             
trq_lvls = 1:20;
amb_temp = 25;
kx = 1.0;
ky = 0.7;
kz = 0.4;
noise = 0.03;

all_tables = {};
global_serial = 1;

fault_masks = uint8([0 1 2 4 3 6 5 7]);

% Fault injection magnitudes
% Pure faults — strong single-signal elevation
fault_current_gain = 2.5;   % Irms ~2.5x healthy
fault_vib_gain     = 3.0;   % Vib  ~3.0x healthy
fault_temp_gain    = 3.5;   % Temp ~3.5x healthy

% Compound faults — two signals elevated, each at 2.0x
% clearly above healthy but below pure fault level on each signal
fault_cv_current_gain = 2.0;   % VibrationCurrent  — current component
fault_cv_vib_gain     = 2.0;   % VibrationCurrent  — vibration component
fault_ct_current_gain = 2.0;   % CurrentTemperature — current component
fault_ct_temp_gain    = 2.0;   % CurrentTemperature — temp component
fault_vt_vib_gain     = 2.0;   % VibrationTemperature — vib component
fault_vt_temp_gain    = 2.0;   % VibrationTemperature — temp component

% MultiParameter — all three signals elevated at 1.8x
% lower than any pure fault, unique in having all three elevated
fault_mp_current_gain = 1.8;
fault_mp_vib_gain     = 1.8;
fault_mp_temp_gain    = 1.8;

for k = 1:length(trq_lvls)
    load = trq_lvls(k) * ones(n,1);
    time = (0:n-1) * ts;
    ldtrq = timeseries(load, time);

    for m = 1:length(fault_masks)
        fault_mask = fault_masks(m);
        fault_enable = (fault_mask ~= 0);

        switch double(fault_mask)
            case 0
                anomaly_type = "NoFault";
                state = "Healthy";
                current_gain = 1.0; vib_gain = 1.0; temp_gain = 1.0;
            case 1
                anomaly_type = "CurrentAnomaly";
                state = "Faulty";
                current_gain = fault_current_gain; vib_gain = 1.0; temp_gain = 1.0;
            case 2
                anomaly_type = "VibrationAnomaly";
                state = "Faulty";
                current_gain = 1.0; vib_gain = fault_vib_gain; temp_gain = 1.0;
            case 4
                anomaly_type = "TemperatureAnomaly";
                state = "Faulty";
                current_gain = 1.0; vib_gain = 1.0; temp_gain = fault_temp_gain;
            case 3
                anomaly_type = "VibrationCurrentAnomaly";
                state = "Faulty";
                current_gain = fault_cv_current_gain; vib_gain = fault_cv_vib_gain; temp_gain = 1.0;
            case 6
                anomaly_type = "VibrationTemperatureAnomaly";
                state = "Faulty";
                current_gain = 1.0; vib_gain = fault_vt_vib_gain; temp_gain = fault_vt_temp_gain;
            case 5
                anomaly_type = "CurrentTemperatureAnomaly";
                state = "Faulty";
                current_gain = fault_ct_current_gain; vib_gain = 1.0; temp_gain = fault_ct_temp_gain;
            case 7
                anomaly_type = "MultiParameterAnomaly";
                state = "Faulty";
                current_gain = fault_mp_current_gain; vib_gain = fault_mp_vib_gain; temp_gain = fault_mp_temp_gain;
            otherwise
                anomaly_type = "UnknownAnomaly";
                state = "Faulty";
                current_gain = 1.0; vib_gain = 1.0; temp_gain = 1.0;
        end

        fault_phase = uint8(0);   % no phase rotation 

        out = sim("motor_model.slx", "StopTime", num2str(time(end)));

        x      = out.motor_data;
        t_no   = length(x.Time);
        torque = x.Data(:,1);
        ia     = x.Data(:,2);
        ib     = x.Data(:,3);
        ic     = x.Data(:,4);
        irms   = x.Data(:,5);
        speed  = x.Data(:,6);
        v_base = detrend(x.Data(:,7));
        amb_temp_sig = x.Data(:,8);
        motor_temp   = x.Data(:,9);

        noise_level = noise * rms(v_base);
        vib_x = (kx * vib_gain * v_base) + (noise_level * randn(size(v_base)));
        vib_y = (ky * vib_gain * v_base) + (noise_level * randn(size(v_base)));
        vib_z = (kz * vib_gain * v_base) + (noise_level * randn(size(v_base)));

        serial_no = (global_serial : global_serial + (t_no - 1))';
        global_serial = global_serial + t_no;

        T = table(serial_no, torque, ia, ib, ic, irms, speed, vib_x, ...
        vib_y, vib_z, amb_temp_sig, motor_temp, x.Time, ...
        'VariableNames', {'Serial No','Torque (Nm)','Ia (A)', ...
        'Ib (A)','Ic (A)','Irms (A)','Speed (RPM)', ...
        'Vibration_X(g)','Vibration_Y(g)','Vibration_Z(g)', ...
        'Ambient Temperature (℃)','Motor Temperature(℃)', ...
        'Time (s)'});

        T.State       = repmat(state,        height(T), 1);
        T.AnomalyType = repmat(anomaly_type, height(T), 1);
                all_tables{end+1} = T;
    end
end

all_tables = vertcat(all_tables{:});
disp(all_tables)
writetable(all_tables, 'motor_data2.csv');