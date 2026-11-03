%PID Controller Design
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

%% 2. Define the plant transfer function
syms kp ki kd s

Gl = (1/J)/(s^2 + (b/J)*s);
C  = kp + ki/s + kd*s;
L  = Gl*C;
G_cl = simplify(L/(1 + L))   % equivale a feedback(L,1)

%% 3. otra
s  = tf('s');
Gl = (1/J)/(s^2 + (b/J)*s);

kp = realp('kp', 1);
ki = realp('ki', 1);
kd = realp('kd', 1);

C = kp + ki/s + kd*s;
G_cl = feedback(Gl*C, 1)     % modelo genss


%% 4. Tune PID parameters
tunedController = tunablePID('PIDController', 'PID');
closedLoopModel = feedback(Gl*tunedController, 1);
tunedController = systune(closedLoopModel);