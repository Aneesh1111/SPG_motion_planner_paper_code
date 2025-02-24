function d = state_correction(d)

if d.input.robot.reset_trigger > 0.5
    d.setpoint.p = d.input.robot.p;
    d.setpoint.v = d.input.robot.v; 
end
