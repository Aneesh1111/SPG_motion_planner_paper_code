function d = next_sample_setpoint(d)

d = spg.setpoint.state_correction(d);
d = spg.setpoint.set(d);

end