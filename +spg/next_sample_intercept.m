function d = next_sample_intercept(d)

%enforce struct names in c
coder.cstructname(d, 'SPGStateStruct');
coder.cstructname(d.subtarget, 'SPGSubtarget');
coder.cstructname(d.aux.segment, 'SPGSegment')

% for sampling intercept code
for i=1:d.par.nintercept_positions
    % overrule some inputs to get desired behavior to compute the ETAs of the desired points
    d.input.robot.quickstop_trigger     = 0.0;
    d.input.robot.target                = d.intercept_positions_etas.sample(i).p;
    d.input.robot.target_vel            = [0,0,0];
    d.input.robot.human_dribble_flag    = 0;
    d.input.robot.skillID               = 0; % move
    d.input.robot.reset_trigger         = 0;
    %initialize target output
    d.target.p      = d.input.robot.target;
    d.target.v      = d.input.robot.target_vel;
    d.target.eta    = 0;
    % update targets
    d           = spg.target.set(d);
    d           = spg.subtarget.set(d);
    d           = spg.setpoint.set(d);
    
    % calculate eta to subtarget
    subtarget_segments  = d.subtarget.segment;
    subtarget_segments  = spg.setpoint.get_segments(subtarget_segments, d.setpoint.p, d.setpoint.v, d.subtarget.p, d.subtarget.v, d.subtarget.vmax, d.subtarget.amax, [d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate]);
    subtarget_eta       = max(subtarget_segments(3).t(1:2));
    
    % calculate eta from subtarget to target
    if ( abs(d.subtarget.p(1) - d.target.p(1))>1e-2 || abs(d.subtarget.p(2) - d.target.p(2))>1e-2 )  % if subtarget is within 1cm to the target then assume subtarget position is end target position
        subtarget_segments = spg.setpoint.get_segments(subtarget_segments, d.subtarget.p, d.subtarget.v, d.target.p, d.target.v, d.par.vmax_move, d.par.amax_move, [d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate]);
        subtarget2target_eta = max(subtarget_segments(3).t(1:2));
    else
        subtarget2target_eta = 0;
    end
    
    % update eta
    d.intercept_positions_etas.sample(i).eta = subtarget_eta + subtarget2target_eta;
end

end