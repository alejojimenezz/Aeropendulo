function L = aero_buildLUT(F, P)
%AERO_BUILDLUT  LUT Voltaje -> Torque del motor (sin friccion seca), lista para Simulink.
Tc = F.Tc;  Vu = F.Vu;  Tu = F.Tu;  Vc = F.Vc;

% 1) Tabla confiable de Tmot
im = Vc >= P.V_lo & Vc <= P.V_hi;            % zona media: promedio de ambas ramas
ih = Vu >  P.V_hi;                           % zona alta: rama de subida + Tc
Vtab = [Vc(im);      Vu(ih)];
Ttab = [F.Tmot(im);  Tu(ih) + Tc];
[Vtab, ks] = sort(Vtab);  Ttab = Ttab(ks);

% 2) Ajuste parametrico T = a*max(V-V0,0)^n
model = @(q,v) q(1).*max(v - q(2), 0).^q(3);
cost  = @(q) sum((model(q,Vtab) - Ttab).^2) + ...
             1*(q(1)<=0 || q(2)<0 || q(2)>=Vtab(1) || q(3)<1 || q(3)>3);
q0    = [Ttab(end)/Vtab(end)^2, 0, 2];
opts  = optimset('TolX',1e-12,'TolFun',1e-16,'MaxFunEvals',2e4,'MaxIter',2e4);
pf    = fminsearch(cost, q0, opts);
R2    = 1 - sum((Ttab - model(pf,Vtab)).^2)/sum((Ttab - mean(Ttab)).^2);
fprintf('Tmot(V) = %.4e*(V-%.3f)^%.3f | R^2 = %.4f\n', pf(1), pf(2), pf(3), R2);

% 3) LUT final 0 -> V_max
V_lut = (0:P.dV_lut:P.V_max)';
T_lut = interp1(Vtab, Ttab, V_lut, 'linear');
lo = V_lut < Vtab(1);                        % bajo voltaje: ajuste escalado, anclado en (0,0)
T_lut(lo) = model(pf, V_lut(lo)) * Ttab(1)/model(pf, Vtab(1));
hi = V_lut > Vtab(end);                      % alto voltaje: extrapolacion con empalme
T_lut(hi) = model(pf, V_lut(hi)) + (Ttab(end) - model(pf, Vtab(end)));
T_lut = cummax(T_lut);                       % monotona

% 4) Saneo (Simulink exige breakpoints estrictamente crecientes, double, sin NaN)
V_lut = double(V_lut(:));  T_lut = double(T_lut(:));
ok = isfinite(V_lut) & isfinite(T_lut);  V_lut = V_lut(ok);  T_lut = T_lut(ok);
[V_lut, iu] = unique(V_lut);  T_lut = T_lut(iu);
assert(all(diff(V_lut) > 0) && V_lut(1) == 0, 'LUT invalida');

L = struct('V_lut',V_lut,'T_lut',T_lut,'pf',pf,'R2',R2,'Vtab',Vtab,'Ttab',Ttab);

th_up = asind(min(max((T_lut - Tc)/P.mgl, 0), 1));
fprintf('Equilibrio de subida a 3.9 V: %.1f deg (real ~83.5)\n', interp1(V_lut, th_up, 3.9));

if P.plot
    figure; hold on; grid on
    plot(F.Vu,F.Tu,'ro-','DisplayName','Subida (Tu)');
    plot(F.Vd,F.Td,'bs-','DisplayName','Bajada (Td)');
    plot(Vtab,Ttab,'ks','MarkerFaceColor','k','DisplayName','Tabla Tmot');
    plot(V_lut,T_lut,    'k-', 'LineWidth',1.8,'DisplayName','LUT Tmot');
    plot(V_lut,T_lut-Tc, 'r--','DisplayName','LUT - Tc (sobre Tu)');
    plot(V_lut,T_lut+Tc, 'b--','DisplayName','LUT + Tc (sobre Td)');
    yline(P.mgl,':k','mgl','HandleVisibility','off');
    xline([P.V_lo P.V_hi],':','HandleVisibility','off');
    xlabel('Voltaje [V]'); ylabel('Torque [N*m]'); legend('Location','northwest');
end
end
