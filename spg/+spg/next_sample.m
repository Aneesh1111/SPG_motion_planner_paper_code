function d = next_sample(d)

d = spg.setpoint.state_correction(d);
d = spg.target.set(d);
d = spg.subtarget.set(d);
d = spg.setpoint.set(d);

end