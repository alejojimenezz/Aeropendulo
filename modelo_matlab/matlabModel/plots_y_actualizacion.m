%% ========================================================================
%  AEROPENDULUM - 2) PLOTS Y ACTUALIZACION
%  ------------------------------------------------------------------------
%  Carga datos_procesados.mat (esto tambien ACTUALIZA el workspace con
%  J, b, mgl, V_lut, T_lut, voltageProfile, etc. para Simulink) y grafica.
%
%  Secciones 2 y 3: solo datos procesados (no requieren simulacion).
%  Secciones 4 y 5: comparan contra Simulink -> correr la simulacion primero.
% ========================================================================

%% 1. Cargar / actualizar workspace
load('datos_procesados.mat');

%% 2. Tratamiento de datos: señales recortadas
% t - voltage
figure;
plot(t(initial:end-final), voltage(initial:end-final))

% t - torque
figure;
plot(t(initial:end-final), torque(initial:end-final))

% volt - torque
figure;
plot(voltage(initial:end-final), torque(initial:end-final), 'o');

%% 3. LUT y ajuste
% Verificacion LUT medida (estado estable)
figure;
plot(cleanVoltage, cleanTorque, '.', 'Color', [0.8 0.8 0.8]); hold on
plot(V_meas, T_meas, 'ro-', 'LineWidth', 1.5);
xlabel('Voltage [V]'); ylabel('Torque [N·m]');
legend('Datos limpios', 'LUT (estado estable)', 'Location', 'northwest');
grid on;

% Verificacion LUT extendida + ajuste
figure;
plot(cleanVoltage, cleanTorque, '.', 'Color', [0.8 0.8 0.8]); hold on
plot(V_lut, T_lut, 'ro', 'LineWidth', 1.5);
vv = linspace(0, V_ext_max, 300);
plot(vv, T_of_V(vv), 'b-', 'LineWidth', 1.5);
xline(Vl(end), '--k');
yline(mgl, ':r');                                % torque max. medible con sin(theta)
xlabel('Voltage [V]'); ylabel('Torque [N·m]');
legend('Datos limpios','LUT','LUT + ajuste','Fin de LUT','mgl (sin\theta = 1)', ...
       'Location','northwest');
grid on;

%% 4. PWM exp vs simu (Correr simu primero)
if ~exist('out','var')
    error('No existe "out". Corre la simulacion en Simulink primero.');
end

% t_exp - pwm_exp
figure;
plot(t_exp(1:end-final), pwm(t_start:end-final))

hold on

% t_mod - pwm_mod (simulink)
t_mod_pwm = out.mod_pwm.time;
pwm_mod   = out.mod_pwm.signals.values*100/5;

stairs(t_mod_pwm, pwm_mod);
legend('exp', 'simu')

%% 5. Angulo exp vs simu (Correr simu primero)
if ~exist('out','var')
    error('No existe "out". Corre la simulacion en Simulink primero.');
end

start     = 7528 - 3335;   % desfase entre inicio exp y simulacion [muestras]
final_ang = 5000;          % muestras recortadas al final

t_mod_angle = out.mod_angle.time;
angle_mod   = out.mod_angle.signals.values;

figure;
plot(t_exp((start+1):end-final_ang), angle((t_start + start):end-final_ang))
hold on;

plot(t_mod_angle, angle_mod);

legend('exp', 'simu')
xlabel('t[s]'); ylabel('Angle [deg]');

hold off;

%% 6. Datos importados (revision rapida)
figure;
plot(t, pwm, 'o-');
xlabel('t');
ylabel('PWM');
grid on;

figure;
plot(t, angle, 'o-');
xlabel('t');
ylabel('Angle');
grid on;
