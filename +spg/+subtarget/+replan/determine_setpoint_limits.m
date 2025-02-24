function subtarget = determine_setpoint_limits(d, subtarget)

angle_diff = mod(d.setpoint.p(3)-subtarget.p(3)+pi,2*pi)-pi;
sc = d.par.scale_rotate;
possess_ball = any(d.input.robot.skillID==1:4);
large_angle = abs(angle_diff)>d.par.scale_angle;
low_velocity = norm(d.setpoint.v(1:2))<1;

if possess_ball && large_angle && low_velocity
    vmax = [d.par.vmax_move*[sc sc]/sqrt(2) d.par.vmax_rotate];
    amax = [d.par.amax_move*[sc sc]/sqrt(2) d.par.amax_rotate];
    dmax = [d.par.dmax_move*[sc sc]/sqrt(2) d.par.dmax_rotate];
else
    vmax = [d.par.vmax_move*[1 1]/sqrt(2) d.par.vmax_rotate];
    amax = [d.par.amax_move*[1 1]/sqrt(2) d.par.amax_rotate];
    dmax = [d.par.dmax_move*[1 1]/sqrt(2) d.par.dmax_rotate];
end


%clip desired subtarget velocity to maximum velocity
if norm(subtarget.v(1:2))>norm(vmax(1:2)) && (d.input.robot.skillID ~= 5)
    subtarget.v(1:2) = subtarget.v(1:2)/norm(subtarget.v(1:2))*norm(vmax(1:2));
end

[subtarget.segment, subtarget.vmax, subtarget.amax, subtarget.dmax] = spg.setpoint.balance_xy(subtarget.segment, d.setpoint.p, d.setpoint.v, subtarget.p, subtarget.v, vmax, amax, dmax);
subtarget.eta = max(subtarget.segment(3).t(1:2));
