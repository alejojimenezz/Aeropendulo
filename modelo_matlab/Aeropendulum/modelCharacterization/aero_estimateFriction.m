function F = aero_estimateFriction(D, P)
%AERO_ESTIMATEFRICTION  Separa subida/bajada, mide la banda de histeresis y estima Tc.
%   Subida: Tu = Tmot - Tc     Bajada: Td = Tmot + Tc
iPeak = find(D.pwm >= P.pwmPeak, 1, 'last');
[Vu,Tu] = aero_stepLevels(D.voltage(1:iPeak),     D.torque(1:iPeak));
[Vd,Td] = aero_stepLevels(D.voltage(iPeak+1:end), D.torque(iPeak+1:end));

Vc   = intersect(Vu, Vd);
Tu_c = interp1(Vu, Tu, Vc);
Td_c = interp1(Vd, Td, Vc);
band = Tu_c - Td_c;                          % negativo: -2*Tc
mid  = Vc > P.V_lo & Vc < P.V_hi;            % zona plateau de la banda

F = struct('Vu',Vu,'Tu',Tu,'Vd',Vd,'Td',Td,'Vc',Vc, ...
           'Tmot',0.5*(Tu_c+Td_c),'band',band,'mid',mid, ...
           'Tc',-mean(band(mid))/2);
fprintf('Tc = %.3e N*m  (%.1f %% de mgl)\n', F.Tc, 100*F.Tc/P.mgl);

if P.plot
    figure; plot(Vu,Tu,'ro-',Vd,Td,'bs-'); grid on
    legend('subida','bajada'); xlabel('V'); ylabel('Torque [N*m]');
    figure; plot(Vc,band,'o-'); hold on; xline([P.V_lo P.V_hi],':'); grid on
    xlabel('V'); ylabel('T_{sub} - T_{baj} [N*m]');
end
end
