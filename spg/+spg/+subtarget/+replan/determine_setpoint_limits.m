function subtarget = determine_setpoint_limits(d, subtarget)

vmax = [d.par.vmax_move*[1 1]/sqrt(2) d.par.vmax_rotate];
amax = [d.par.amax_move*[1 1]/sqrt(2) d.par.amax_rotate];
dmax = [d.par.dmax_move*[1 1]/sqrt(2) d.par.dmax_rotate];

%clip desired subtarget velocity to maximum velocity
if norm(subtarget.v(1:2))>norm(vmax(1:2))
    subtarget.v(1:2) = subtarget.v(1:2)/norm(subtarget.v(1:2))*norm(vmax(1:2));
end

[subtarget.segment, subtarget.vmax, subtarget.amax, subtarget.dmax] = spg.setpoint.balance_xy(subtarget.segment, d.setpoint.p, d.setpoint.v, subtarget.p, subtarget.v, vmax, amax, dmax);
subtarget.eta = max(subtarget.segment(3).t(1:2));

end