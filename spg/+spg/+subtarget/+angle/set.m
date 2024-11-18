function angle = set(d)

%check skillID   0: move, 1: dribble, 2: aim, 3: kick, 4: shield, 5: intercept
switch d.input.robot.skillID
    case 1 %dribble
        angle = spg.subtarget.angle.dribble(d);
        
    otherwise %0 move, 2 aim, 3 kick, 4 shield
        angle = d.target.p(3);
%         angle = obj.subtarget.p(3); %keep same angle
end

%wrap around current setpoint
angle = spg.setpoint.wrap(angle, d.setpoint.p(3));
