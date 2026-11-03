function D = aero_loadData(P)
%AERO_LOADDATA  Lee el CSV y devuelve tiempo [s], pwm, angulo, voltaje y torque cuasi-estatico.
data = readtable(P.csvFile);
t   = data{:,1};
if isduration(t), t = seconds(t); end
t   = double(t)*P.timeScale;   t = t - t(1);
pwm = data{:,3};
ang = data{:,4};
ang(ang > 180) = 0;                         % limpiar lecturas invalidas

[t, iu] = unique(t);
D.t       = t;
D.pwm     = pwm(iu);
D.angle   = ang(iu);
D.voltage = D.pwm*(5/100);
D.torque  = P.mgl*sind(D.angle);            % T = mgl*sin(theta) en estado estable

if P.plot
    figure; yyaxis left;  plot(D.t, D.voltage); ylabel('Voltaje [V]');
    yyaxis right; plot(D.t, D.angle); ylabel('Angulo [deg]');
    xlabel('t [s]'); grid on; title('Datos importados');
end
end
