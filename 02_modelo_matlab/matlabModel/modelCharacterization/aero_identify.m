function R = aero_identify(D, P, Tc0, pf0, b0)
%AERO_IDENTIFY  Ajuste global de [Tc, a, V0, n, b] contra la trayectoria completa.
tg  = (0:P.dt_id:D.t(end))';
Vg  = interp1(D.t, D.voltage, tg, 'previous');
ag  = interp1(D.t, D.angle,   tg);
th0 = deg2rad(ag(1));

q0 = [Tc0, pf0(1), max(pf0(2),1e-3), pf0(3), b0];
cost = @(z) sqrt(mean((simAero(q0.*exp(z), tg, Vg, th0, P) - ag).^2));   % z = log(q/q0)

z = fminsearch(cost, zeros(1,5), ...
    optimset('Display','iter','TolX',1e-4,'MaxFunEvals',P.maxEvals));
q = q0.*exp(z);
R = struct('q',q,'rmse',cost(z),'tg',tg,'ag',ag,'Vg',Vg,'th0',th0);
fprintf('Tc=%.3e  a=%.3e  V0=%.3f  n=%.3f  b=%.3e | RMSE=%.2f deg\n', q, R.rmse);

if P.plot
    figure; plot(tg,ag,'b',tg,simAero(q,tg,Vg,th0,P),'r'); grid on
    legend('exp','modelo ajustado'); xlabel('t [s]'); ylabel('Angle [deg]');
end
end
