%% MAIN  Aeropendulo: caracterizacion, LUT e identificacion
% Archivos: aero_params, aero_loadData, aero_stepLevels, aero_estimateFriction,
%           aero_buildLUT, simAero, aero_identify, aero_exportToBase, aero_runSim
clear; close all; clc

%% 1) Parametros y datos
P = aero_params();
D = aero_loadData(P);

%% 2) Friccion seca (histeresis subida/bajada)
F = aero_estimateFriction(D, P);

%% 3) LUT Voltaje -> Torque del motor
L = aero_buildLUT(F, P);
aero_exportToBase(P, F.Tc, L.V_lut, L.T_lut);      % Simulink ya puede correr

%% 4) Simulacion vs experimento (opcional: barrido de b)
aero_runSim(D, P);                                  % un solo b
aero_runSim(D, P, [3e-3 5e-3 7e-3 1e-2]);         % barrido

%% 5) Identificacion global [Tc, a, V0, n, b]  (tarda varios minutos)
R = aero_identify(D, P, F.Tc, L.pf, P.b);

%% 6) Aplicar resultado y re-simular
q = R.q;
P.b = q(5);
V_id = (0:P.dV_lut:P.V_max)';
T_id = q(2)*max(V_id - q(3), 0).^q(4);
aero_exportToBase(P, q(1), V_id, T_id);
aero_runSim(D, P);
