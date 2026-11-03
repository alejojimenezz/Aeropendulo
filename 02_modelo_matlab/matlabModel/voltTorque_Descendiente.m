% Aeropendulum Model
% Linearize aroung theta_0 and torque_0
% we get the following model

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
%b = 7.59e-4;


m1 = m_motorAss; % kg
m2 = m_counterW; % kg
l1 = d_motor; % m
l2 = d_counterW; % m

mgl = (m1*l1 + m2*l2)*9.81;

% Voltage Repeating sequence (for simulation)
k = 58:-2:0;
voltageProfileDesc = 5.*(k/100);
sampleTime = 15.6340;
%% Get data
%data = readtable('Rampa de a 5.csv');

data = readtable('escalones de a 2 descendente.csv');
head(data)

t = data{:, 1};
pwm = data{:, 2};
angle = data{:, 3};
voltage = pwm*(5/100);

% Find torque (quasistatic)
torque = mgl*sin(angle*(pi/180));


%% Tratamiento de datos

initial = 13788+1; % starting t
final = 20000; % (end - final) = final time

figure;
plot(t(initial:end-final),angle(initial:end-final), 'o-');
xlabel('t');
ylabel('Angle');
grid on;

%% t pwm
figure;
plot(t(initial:end-final),pwm(initial:end-final), 'o-');
xlabel('t');
ylabel('PWM');
grid on;
% starts at 58%

%%

% t - voltage
figure;
plot(t(initial:end-final),voltage(initial:end-final))

% t - torque
figure;
plot(t(initial:end-final),torque(initial:end-final))


% volt - torque
figure;
plot(voltage(initial:end-final), torque(initial:end-final), 'o');



%% LUT 
% Voltage - Torque (LUT)
cleanVoltage = round(voltage(initial:end-final), 2);
cleanTorque  = torque(initial:end-final);

% Último dato de cada escalón (justo antes de que cambie el voltaje)
idx_end = [find(diff(cleanVoltage) ~= 0); numel(cleanVoltage)];
V_end = cleanVoltage(idx_end);
T_end = cleanTorque(idx_end);

% Breakpoints únicos y crecientes (si un nivel se repite, promedia)
[V_lut, ~, k] = unique(V_end);
T_lut = accumarray(k, T_end, [], @mean);

% Quitar offset: 0 V -> 0 N·m
% Remove offset: reference torque at 0 V
idx0 = abs(V_lut) < 1e-9;
if any(idx0)
    T_lut = T_lut - T_lut(find(idx0,1));
else
    warning('No 0 V step found in the trimmed data. Using the lowest-voltage step as baseline and adding (0 V, 0 N·m).');
    T_lut = T_lut - T_lut(1);
    V_lut = [0; V_lut];
    T_lut = [0; T_lut];
end
T_lut = max(T_lut, 0);


% Verificación
figure;
plot(cleanVoltage, cleanTorque, '.', 'Color', [0.8 0.8 0.8]); hold on
plot(V_lut, T_lut, 'ro-', 'LineWidth', 1.5);
xlabel('Voltage [V]'); ylabel('Torque [N·m]');
legend('Datos limpios', 'LUT (estado estable)', 'Location', 'northwest');
grid on;

%%--------------------
%% Ajuste / extrapolación de la LUT (Voltage -> Torque)
% Modelo: T(V) = a * max(V - V0, 0)^n   (zona muerta + ley de potencia)
Vl = V_lut(:);  Tl = T_lut(:);

% Puntos para el ajuste (ajusta la máscara si quieres excluir saturación, p.ej. Vl <= 3.5)
mask = Vl > 0;
Vf = Vl(mask);  Tf = Tl(mask);

model = @(p,v) p(1) .* max(v - p(2), 0).^p(3);   % p = [a V0 n]

% Costo con restricciones por penalización: a>0, 0<=V0<Vmax, 1<=n<=3
cost = @(p) sum((model(p,Vf) - Tf).^2) + ...
            1*(p(1)<=0 || p(2)<0 || p(2)>=Vf(end) || p(3)<1 || p(3)>3);

p0   = [Tf(end)/Vf(end)^2, 0, 2];
opts = optimset('TolX',1e-12,'TolFun',1e-16,'MaxFunEvals',2e4,'MaxIter',2e4);
p    = fminsearch(cost, p0, opts);

% Calidad del ajuste
Tfit = model(p, Vf);
R2   = 1 - sum((Tf - Tfit).^2) / sum((Tf - mean(Tf)).^2);
fprintf('T(V) = %.4e * (V - %.3f)^%.3f   |  R^2 = %.4f\n', p(1), p(2), p(3), R2);

% Alternativa sin zona muerta (cuadrática por el origen): T = c1*V^2 + c2*V
% c = [Vf.^2, Vf] \ Tf;   model_q = @(v) c(1)*v.^2 + c(2)*v;

%% Extender la LUT hacia voltajes mayores
V_ext_max = 5;                                   % V (100 % PWM)
dV        = 0.1;
V_ext     = (V_lut(end) + dV : dV : V_ext_max)';

% Offset para que la extrapolación empate con el último punto de la LUT
offset = Tl(end) - model(p, Vl(end));
T_ext  = model(p, V_ext) + offset;

V_lut_ext = [Vl; V_ext];
T_lut_ext = [Tl; T_ext];

V_lut = V_lut_ext
T_lut = T_lut_ext

% Función lista para usar: LUT dentro del rango medido, ajuste fuera de él
T_of_V = @(v) (v <= Vl(end)) .* interp1(Vl, Tl, min(v, Vl(end)), 'linear') + ...
              (v >  Vl(end)) .* (model(p, v) + offset);

%% Verificación
figure;
plot(cleanVoltage, cleanTorque, '.', 'Color', [0.8 0.8 0.8]); hold on
plot(V_lut, T_lut, 'ro', 'LineWidth', 1.5);
vv = linspace(0, V_ext_max, 300);
plot(vv, T_of_V(vv), 'b-', 'LineWidth', 1.5);
xline(Vl(end), '--k');
yline(mgl, ':r');                                % torque máx. medible con sin(theta)
xlabel('Voltage [V]'); ylabel('Torque [N·m]');
legend('Datos limpios','LUT','LUT + ajuste','Fin de LUT','mgl (sin\theta = 1)', ...
       'Location','northwest');
grid on;
%%--------------------
%% Calibrate experimental and simulation data (Correr simu primero)
t_start = initial;

t_exp = (t(initial:(end-final))-t(initial)) * 1e-6;     % us -> s, empezando en 0


% t_exp - pwm_exp
figure;
plot(t_exp,pwm(initial:(end-final)))

hold on 

% t_mod - pwm_mod (simulink)
t_mod_pwm = out.mod_pwm.time;
pwm_mod = out.mod_pwm.signals.values*100/5;


% plot
stairs(t_mod_pwm,pwm_mod);
legend('exp', 'simu')

%% ang(exp) vs ang(mod) (Correr simu primero)

figure;
start = 7528-initial;
%final = 5000;

t_mod_angle = out.mod_angle.time;
angle_mod = out.mod_angle.signals.values;

plot(t_exp,angle(initial:(end-final)))
hold on;

plot(t_mod_angle,angle_mod);

legend('exp', 'simu')
xlabel('t[s]'); ylabel('Angle [deg]');

hold off;


%% Plot t - pwm & angle (Check imported data)
figure;
plot(t, pwm, 'o-');
hold on
xlabel('t');
ylabel('PWM');
grid on;
hold off

figure;
plot(t, angle, 'o-');
xlabel('t');
ylabel('Angle');
grid on;
