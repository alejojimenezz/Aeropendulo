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
b = 9.184e-4 * 1; % Nms/rad
%b = 500e-4; % Nms/rad
b = 1e-2; % Nms/rad


m1 = m_motorAss; % kg
m2 = m_counterW; % kg
l1 = d_motor; % m
l2 = d_counterW; % m

mgl = (m1*l1 + m2*l2)*9.81;

% Voltage Repeating sequence (for simulation)
k=5:5:100;
voltageProfile = 5.*(k/100);
sampleTime = 2;

k = [3:3:80,80,78:-3:3]
voltageProfile = 5.*(k/100);
sampleTime = 2;

%% Get data
%data = readtable('Rampa de a 5.csv');

%data = readtable('perfil4_02_2026-11-02_18-33-10.csv');
data = readtable('perfil4_01_2026-11-02_18-29-07.csv');
head(data)

t = data{:, 1};
pwm = data{:, 3};
angle = data{:, 4};

angle(angle>180)= 0; % clean angle

voltage = pwm*(5/100);

% Find torque (quasistatic)
torque = mgl*sin(angle*(pi/180));


%% Tratamiento de datos

initial = 1; % starting t
final = 0; % (end - final) = final time

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
beforeDescent = find(pwm >= 80, 1, 'last');
cleanVoltage = round(voltage(initial:beforeDescent), 2);
cleanTorque  = torque(initial:beforeDescent);

% Último dato de cada escalón (justo antes de que cambie el voltaje)
idx_end = [find(diff(cleanVoltage) ~= 0); numel(cleanVoltage)];
V_end = cleanVoltage(idx_end);
T_end = cleanTorque(idx_end);

% Breakpoints únicos y crecientes (si un nivel se repite, promedia)
[V_lut, ~, k] = unique(V_end);
T_lut = accumarray(k, T_end, [], @mean);

% Quitar offset: 0 V -> 0 N·m


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
t_start = 3335;
t_exp = double(t(t_start:end));  % a double, y columna
t_exp = (t_exp - t_exp(1)) * 1e-6;     % us -> s, empezando en 0


% t_exp - pwm_exp
figure;
plot(t,pwm)
%plot(t_exp(1:end-final),pwm(t_start:end-final))

hold on 

% t_mod - pwm_mod (simulink)
t_mod_pwm = out.mod_pwm.time;
pwm_mod = out.mod_pwm.signals.values*100/5;

% plot
stairs(t_mod_pwm,pwm_mod);
legend('exp', 'simu')

%% ang(exp) vs ang(mod) (Correr simu primero)

figure;
start = 7528-3335;
%final = 5000;

t_mod_angle = out.mod_angle.time;
angle_mod = out.mod_angle.signals.values;



plot(t,angle)
%plot(t_exp((start+1):end-final),angle((3335 + start):end-final))
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


iPeak  = find(pwm >= 80, 1, 'last');
[Vu,Tu] = lastOfEachStep(voltage(1:iPeak),      torque(1:iPeak));
[Vd,Td] = lastOfEachStep(voltage(iPeak+1:end),  torque(iPeak+1:end));

figure; plot(Vu,Tu,'ro-',Vd,Td,'bs-'); grid on
legend('subida','bajada'); xlabel('V'); ylabel('Torque [N·m]')

Vc  = intersect(Vu,Vd);
dT  = interp1(Vu,Tu,Vc) - interp1(Vd,Td,Vc);
figure; plot(Vc,dT,'o-'); ylabel('T_{sub} - T_{baj} [N·m]')

function [V,T] = lastOfEachStep(v,t0)
    v = round(v,2);
    idx = [find(diff(v)~=0); numel(v)];
    V = v(idx);  T = t0(idx);
end

Vc    = intersect(Vu,Vd);
Tmot  = 0.5*(interp1(Vu,Tu,Vc) + interp1(Vd,Td,Vc));   % torque del motor sin fricción seca
band  = interp1(Vu,Tu,Vc) - interp1(Vd,Td,Vc);
mid   = Vc > 1.8 & Vc < 3.1;                           % zona plateau
Tc    = -mean(band(mid))/2                              % ≈ 3.4e-3 N·m

epsilon = 0.005;

%% ===== LUT corregida: Voltaje -> Torque del motor (sin fricción seca) =====
% Relación entre ramas:  Tu = Tmot - Tc  (subida)   |   Td = Tmot + Tc  (bajada)
V_lo      = 1.8;    % por debajo: ajuste paramétrico (brazo apenas se mueve)
V_hi      = 3.1;    % por encima: solo rama de subida + Tc (la banda se cierra por artefacto)
V_max_sim = 5;      % V (100 % PWM)
dV_lut    = 0.05;

Vu = Vu(:); Tu = Tu(:); Vd = Vd(:); Td = Td(:);

% 1) Tabla confiable de Tmot
Vc     = intersect(Vu, Vd);
Tu_c   = interp1(Vu, Tu, Vc);
Td_c   = interp1(Vd, Td, Vc);
Tm_mid = 0.5*(Tu_c + Td_c);                 % zona media: promedio de ambas ramas

im = Vc >= V_lo & Vc <= V_hi;               % zona media
ih = Vu >  V_hi;                            % zona alta: rama de subida

Vtab = [Vc(im);     Vu(ih)];
Ttab = [Tm_mid(im); Tu(ih) + Tc];           % Tmot = Tu + Tc
[Vtab, ks] = sort(Vtab);  Ttab = Ttab(ks);

% 2) Ajuste paramétrico sobre la tabla corregida: T = a*max(V-V0,0)^n
model = @(q,v) q(1).*max(v - q(2), 0).^q(3);
cost  = @(q) sum((model(q,Vtab) - Ttab).^2) + ...
             1*(q(1)<=0 || q(2)<0 || q(2)>=Vtab(1) || q(3)<1 || q(3)>3);
q0    = [Ttab(end)/Vtab(end)^2, 0, 2];
opts  = optimset('TolX',1e-12,'TolFun',1e-16,'MaxFunEvals',2e4,'MaxIter',2e4);
pf    = fminsearch(cost, q0, opts);

Tfit = model(pf, Vtab);
R2f  = 1 - sum((Ttab - Tfit).^2)/sum((Ttab - mean(Ttab)).^2);
fprintf('Tmot(V) = %.4e*(V-%.3f)^%.3f | R^2 = %.4f\n', pf(1), pf(2), pf(3), R2f);

% 3) Armar la LUT final de 0 a V_max_sim
V_lut = (0:dV_lut:V_max_sim)';
T_lut = interp1(Vtab, Ttab, V_lut, 'linear');          % zona medida

lo = V_lut < Vtab(1);                                   % bajo voltaje: ajuste escalado, anclado en (0,0)
T_lut(lo) = model(pf, V_lut(lo)) * Ttab(1)/model(pf, Vtab(1));

hi = V_lut > Vtab(end);                                 % extrapolación: ajuste con offset de empalme
T_lut(hi) = model(pf, V_lut(hi)) + (Ttab(end) - model(pf, Vtab(end)));

T_lut = cummax(T_lut);                                  % monótona creciente (ruido de escalones)

% 4) Verificación
figure; hold on; grid on
plot(Vu, Tu, 'ro-', 'DisplayName','Subida medida (Tu)');
plot(Vd, Td, 'bs-', 'DisplayName','Bajada medida (Td)');
plot(Vtab, Ttab, 'ks', 'MarkerFaceColor','k', 'DisplayName','Tabla Tmot');
plot(V_lut, T_lut,      'k-',  'LineWidth',1.8, 'DisplayName','LUT Tmot');
plot(V_lut, T_lut - Tc, 'r--', 'LineWidth',1.2, 'DisplayName','LUT - Tc (debe caer sobre Tu)');
plot(V_lut, T_lut + Tc, 'b--', 'LineWidth',1.2, 'DisplayName','LUT + Tc (debe caer sobre Td)');
yline(mgl, ':k', 'mgl', 'HandleVisibility','off');
xline([V_lo V_hi], ':', 'HandleVisibility','off');
xlabel('Voltage [V]'); ylabel('Torque [N·m]'); legend('Location','northwest');

% Ángulo de equilibrio estático esperado en la subida (compáralo con el real)
th_up = asind(min(max((T_lut - Tc)/mgl, 0), 1));
fprintf('Equilibrio subida a 3.9 V: %.1f deg (real ~83.5 deg)\n', interp1(V_lut, th_up, 3.9));
%%
bs = [3 5 7 10]*9.184e-4*[1];   % ajusta el rango: ~3e-3 a 1e-2
bs = [3e-3 5e-3 7e-3 1e-2];

figure; plot(t, angle, 'k', 'LineWidth', 1.5); hold on
for i = 1:numel(bs)
    b = bs(i);                                   % variable que lee Simulink
    out = sim('AeropendulumNonLinearModel');
    plot(out.mod_angle.time, out.mod_angle.signals.values, ...
         'DisplayName', sprintf('b = %.1e', b));
end
legend; xlabel('t [s]'); ylabel('Angle [deg]'); grid on

%% ===== Identificación global con b =====
% (J y mgl ya están en el workspace)

% --- tiempo en segundos, desde 0 y sin repetidos ---
t_s = double(t);                            % si t es duration: t_s = seconds(t);
t_s = t_s - t_s(1);
if t_s(end) > 1e4, t_s = t_s*1e-6; end      % venía en µs
[t_s, iu] = unique(t_s);
volt_s = voltage(iu);   ang_s = angle(iu);

dt = 0.05;
tg = (0:dt:t_s(end))';
Vg = interp1(t_s, volt_s, tg, 'previous');
ag = interp1(t_s, ang_s,  tg);
th0 = deg2rad(ag(1));

% --- parámetros: q = [Tc, a, V0, n, b] ---
b0   = 1e-2;                                % el que ya te funcionó
q0   = [Tc, pf(1), pf(2), pf(3), b0];
q0(2:3) = max(q0(2:3), 1e-3);               % evita log(0) si V0 = 0
cost = @(z) sqrt(mean((simAero(q0.*exp(z), tg, Vg, J, mgl, th0) - ag).^2));

z = fminsearch(cost, zeros(1,5), ...
    optimset('Display','iter','TolX',1e-4,'MaxFunEvals',600));
q = q0.*exp(z);

fprintf('Tc=%.3e  a=%.3e  V0=%.3f  n=%.3f  b=%.3e | RMSE=%.2f deg\n', ...
        q(1), q(2), q(3), q(4), q(5), cost(z));

% --- comparación ---
figure; plot(tg, ag, 'b', tg, simAero(q,tg,Vg,J,mgl,th0), 'r'); grid on
legend('exp','modelo ajustado'); xlabel('t [s]'); ylabel('Angle [deg]');

% --- valores para Simulink ---
Tc    = q(1);
b     = q(5);
V_lut = (0:0.05:5)';
T_lut = q(2)*max(V_lut - q(3), 0).^q(4);

%% ===== Funciones locales: SIEMPRE al final del archivo =====
function th = simAero(q, tg, Vg, J, mgl, th0)
    Tc = q(1); a = q(2); V0 = q(3); n = q(4); b = q(5); eps_ = 1e-3;
    th = zeros(size(tg));   x = [th0; 0];   th(1) = rad2deg(x(1));

    ic = unique([1; find(diff(Vg) ~= 0) + 1; numel(tg)]);   % índices donde cambia V
    for k = 1:numel(ic)-1
        i1 = ic(k);  i2 = ic(k+1);
        T  = a*max(Vg(i1) - V0, 0)^n;
        f  = @(~,s) [ s(2); (T - mgl*sin(s(1)) - b*s(2) - Tc*tanh(s(2)/eps_))/J ];
        [~,xs] = ode15s(f, tg(i1:i2), x, odeset('MaxStep',0.05));
        if i2 - i1 == 1, xs = xs([1 end],:); end
        th(i1:i2) = rad2deg(xs(:,1));
        x = xs(end,:)';
    end
end