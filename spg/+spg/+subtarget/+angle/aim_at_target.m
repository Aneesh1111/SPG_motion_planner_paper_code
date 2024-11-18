function angle = aim_at_target(d)

v = d.target.p(1:2)-d.setpoint.p(1:2);
if nnz(v)
    angle = atan2(-v(1),v(2));
else
    angle = d.subtarget.p(3);
end
