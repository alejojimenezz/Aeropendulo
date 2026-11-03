%% ========================================================================
%  AEROPENDULUM - 1) TRATAMIENTO DE DATOS
%  ------------------------------------------------------------------------
%  Linearize around theta_0 and torque_0 -> we get the following model.
%
%  Este script NO grafica. Hace:
%    parametros -> carga del CSV -> torque -> LUT -> ajuste/extrapolacion
%    -> alineacion de tiempos -> guarda todo en datos_procesados.mat
%
%  Orden de uso:
%    1) tratamiento_datos.m      (este)
%    2) correr la simulacion en Simulink
%    3) plots_y_actualizacion.m
% ========================================================================

%% 1. Parametros del modelo
s = tf('s');

J_brazo    = 76.087e-6;   % kg·m²
J_eje      = 13.282e-6;   % kg·m²
m_counterW = 12e-3;       % kg     (12 g)
d_counterW = 6.8e-2;      % m
m_motorAss = 12e-3;       % kg     (12 g)
d_motor    = 159e-3;      % m

% Inercia total (Steiner: J = J_cm + m·d²)
J = J_brazo + J_eje + m_counterW*d_counterW^2 + m_motorAss*d_motor^2;  % kg·m²
%J = J + 1e-4;

% b (from log decrement)
%b = 9.184e-4 * 3.44; % Nms/rad
b = 500e-4; % Nms/rad

m1 = m_motorAss; % kg
m2 = m_counterW; % kg
l1 = d_motor;    % m
l2 = d_counterW; % m

mgl = (m1*l1 + m2*l2)*9.81;

% Voltage Repeating sequence (for simulation)
kv = 0:2:100;
voltageProfile = 5.*(kv/100);

%% 2. Carga de datos
%data = readtable('Rampa de a 5.csv');
data = readtable('escalones de a 2 v2.csv');
head(data)

t       = data{:, 1};
pwm     = data{:, 2};
angle   = data{:, 3};
voltage = pwm*(5/100);

% Torque (cuasi-estatico)
torque = mgl*sin(angle*(pi/180));

%% 3. Recorte del tramo util
initial = 5000;    % muestra inicial
final   = 20000;   % (end - final) = muestra final

cleanVoltage = round(voltage(initial:end-final), 2);
cleanTorque  = torque(initial:end-final);

%% 4. LUT Voltage -> Torque (estado estable)
% Ultimo dato de cada escalon (justo antes de que cambie el voltaje)
idx_end = [find(diff(cleanVoltage) ~= 0); numel(cleanVoltage)];
V_end = cleanVoltage(idx_end);
T_end = cleanTorque(idx_end);

% Breakpoints unicos y crecientes (si un nivel se repite, promedia)
[V_meas, ~, ic] = unique(V_end);
T_meas = accumarray(ic, T_end, [], @mean);

% Quitar offset: 0 V -> 0 N·m
T_meas = T_meas - T_meas(V_meas == 0);
T_meas = max(T_meas, 0);

%% 5. Ajuste / extrapolacion de la LUT (Voltage -> Torque)
% Modelo: T(V) = a * max(V - V0, 0)^n   (zona muerta + ley de potencia)
Vl = V_meas(:);  Tl = T_meas(:);

% Puntos para el ajuste (ajusta la mascara si quieres excluir saturacion, p.ej. Vl <= 3.5)
mask = Vl > 0;
Vf = Vl(mask);  Tf = Tl(mask);

model = @(p,v) p(1) .* max(v - p(2), 0).^p(3);   % p = [a V0 n]

% Costo con restricciones por penalizacion: a>0, 0<=V0<Vmax, 1<=n<=3
cost = @(p) sum((model(p,Vf) - Tf).^2) + ...
            1*(p(1)<=0 || p(2)<0 || p(2)>=Vf(end) || p(3)<1 || p(3)>3);

p0   = [Tf(end)/Vf(end)^2, 0, 2];
opts = optimset('TolX',1e-12,'TolFun',1e-16,'MaxFunEvals',2e4,'MaxIter',2e4);
p    = fminsearch(cost, p0, opts);

% Calidad del ajuste
Tfit = model(p, Vf);
R2   = 1 - sum((Tf - Tfit).^2) / sum((Tf - mean(Tf)).^2);
fprintf('T(V) = %.4e * (V - %.3f)^%.3f   |  R^2 = %.4f\n', p(1), p(2), p(3), R2);

% Alternativa sin zona muerta (cuadratica por el origen): T = c1*V^2 + c2*V
% c = [Vf.^2, Vf] \ Tf;   model_q = @(v) c(1)*v.^2 + c(2)*v;

%% 6. Extender la LUT hacia voltajes mayores
V_ext_max = 5;                                   % V (100 % PWM)
dV        = 0.1;
V_ext     = (Vl(end) + dV : dV : V_ext_max)';

% Offset para que la extrapolacion empate con el ultimo punto de la LUT
offset = Tl(end) - model(p, Vl(end));
T_ext  = model(p, V_ext) + offset;

% LUT final (la que usa Simulink)
V_lut = [Vl; V_ext];
T_lut = [Tl; T_ext];

% Funcion lista para usar: LUT dentro del rango medido, ajuste fuera de el
T_of_V = @(v) (v <= Vl(end)) .* interp1(Vl, Tl, min(v, Vl(end)), 'linear') + ...
              (v >  Vl(end)) .* (model(p, v) + offset);

%% 7. Alineacion de tiempos experimental (para comparar con la simulacion)
t_start = 3335;
t_exp = double(t(t_start:end));        % a double, y columna
t_exp = (t_exp - t_exp(1)) * 1e-6;     % us -> s, empezando en 0

%% 8. Guardar resultados
save('datos_procesados.mat', ...
    's', 'J_brazo', 'J_eje', 'm_counterW', 'd_counterW', 'm_motorAss', 'd_motor', ...
    'J', 'b', 'm1', 'm2', 'l1', 'l2', 'mgl', 'voltageProfile', ...
    't', 'pwm', 'angle', 'voltage', 'torque', ...
    'initial', 'final', 'cleanVoltage', 'cleanTorque', ...
    'V_meas', 'T_meas', 'Vl', 'Tl', 'p', 'R2', 'model', 'offset', 'V_ext_max', ...
    'V_lut', 'T_lut', 'T_of_V', ...
    't_start', 't_exp');

fprintf('Datos guardados en datos_procesados.mat\n');
