function out = aero_runSim(D, P, bs)
%AERO_RUNSIM  Corre Simulink (una o varias veces, barriendo b) y compara con el experimento.
%   aero_runSim(D,P)            -> una corrida con el b actual
%   aero_runSim(D,P,[3e-3 1e-2])-> barrido de b
if nargin < 3, bs = evalin('base','b'); end
b_old = evalin('base','b');
figure; plot(D.t, D.angle, 'w', 'LineWidth', 1.5, 'DisplayName','exp'); hold on
for i = 1:numel(bs)
    assignin('base','b',bs(i));
    out = sim(P.modelName);
    plot(out.mod_angle.time, out.mod_angle.signals.values, 'DisplayName', sprintf('b = %.1e', bs(i)));
end
assignin('base','b',b_old);                    % restaura
legend; xlabel('t [s]'); ylabel('Angle [deg]'); grid on
end
