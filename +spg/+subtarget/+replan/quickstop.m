function subtarget = quickstop(d, subtarget)
% 	SMOOTHSTOP_NONE = 0, SMOOTHSTOP_SMOOTH_XYPHI = 1, SMOOTHSTOP_SMOOTH_XY = 2, SMOOTHSTOP_SMOOTH_PHI = 3

subtarget.p = d.setpoint.p;
% in case of SMOOTHSTOP_SMOOTH_XY keep the target.o
if d.input.robot.quickstop_trigger == 2
    subtarget.p(3) = d.target.p(3);
end
subtarget.v = [0 0 0];
max_downscale = .01;
sc = max(max_downscale,abs(d.setpoint.v(1:2))/norm(d.setpoint.v(1:2)));
subtarget.vmax = [d.par.vmax_move*sc d.par.vmax_rotate];
subtarget.amax = [d.par.amax_quickstop*sc d.par.dmax_rotate];
subtarget.dmax = [d.par.dmax_move d.par.dmax_move d.par.dmax_rotate];
subtarget.p(1:2) = subtarget.p(1:2)+d.setpoint.v(1:2).*abs(d.setpoint.v(1:2))./subtarget.dmax(1:2)/2;
subtarget.eta = 0;
subtarget.age = 0;
