function P = aero_params()
%AERO_PARAMS  Todos los parametros y opciones en un solo lugar.

% --- Geometria / inercia ---
J_brazo    = 76.087e-6;   % kg*m^2
J_eje      = 13.282e-6;   % kg*m^2
m_counterW = 12e-3;       % kg
d_counterW = 6.8e-2;      % m
m_motorAss = 12e-3;       % kg
d_motor    = 159e-3;      % m

P.J   = J_brazo + J_eje + m_counterW*d_counterW^2 + m_motorAss*d_motor^2;  % Steiner
P.mgl = (m_motorAss*d_motor + m_counterW*d_counterW)*9.81;

% --- Dinamica ---
P.b       = 6.7e-4;     % N*m*s/rad (valor que mejoro el ajuste; el log. decrement daba 9.18e-4)
P.epsilon = 0.005;    % rad/s, suavizado de tanh(w/eps). Lo usan Simulink Y la identificacion

% --- Perfil de entrada (el mismo que se mando al experimento) ---
k = [3:3:80, 80, 78:-3:3];            % % PWM
P.voltageProfile = 5*(k/100);
P.sampleTime     = 2;                 % s por escalon

% --- Datos ---
P.csvFile     = 'perfil4_01_2026-11-02_18-29-07.csv';
P.timeScale   = 1;                    % factor a segundos (1e-6 si t viene en us)
P.pwmPeak     = 80;                   % % PWM del tope (separa subida / bajada)

% --- LUT ---
P.V_lo   = 1.8;       % V: bajo esto, ajuste parametrico
P.V_hi   = 3.1;       % V: sobre esto, solo rama de subida + Tc
P.V_max  = 5;         % V (100 % PWM)
P.dV_lut = 0.05;      % V, resolucion de la LUT

% --- Identificacion global ---
P.dt_id = 0.05;       % s, malla de integracion
P.maxEvals = 600;

% --- Simulink ---
P.modelName = 'AeropendulumNonLinearModel';
P.plot = true;
end
