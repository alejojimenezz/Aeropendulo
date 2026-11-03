function aero_exportToBase(P, Tc, V_lut, T_lut)
%AERO_EXPORTTOBASE  Publica en el workspace base lo que lee Simulink.
assignin('base','J',P.J);              assignin('base','b',P.b);
assignin('base','mgl',P.mgl);          assignin('base','epsilon',P.epsilon);
assignin('base','Tc',Tc);
assignin('base','V_lut',V_lut);        assignin('base','T_lut',T_lut);
assignin('base','voltageProfile',P.voltageProfile);
assignin('base','sampleTime',P.sampleTime);
end
