function angle = dribble(d)

if norm(d.setpoint.v(1:2))>1e-6
    v = d.setpoint.v(1:2);
    angle = atan2(-v(1),v(2));
else
    angle = d.subtarget.p(3);
end
