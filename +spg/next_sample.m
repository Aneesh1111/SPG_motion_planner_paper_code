function d = next_sample(d)

%enforce struct names in c
coder.cstructname(d, 'SPGStateStruct');
coder.cstructname(d.subtarget, 'SPGSubtarget');
coder.cstructname(d.aux.segment, 'SPGSegment')

e = d;

d = spg.setpoint.state_correction(d);
d = spg.target.set(d);
d = spg.subtarget.set(d);
d = spg.setpoint.set(d);

% set target eta - use eta to subtarget, then assume a straight line
% towards end target to estimate time (subtarget.eta + straight line time)

% calculate eta to subtarget
subtarget_segments = d.subtarget.segment;
subtarget_segments = spg.setpoint.get_segments(subtarget_segments, d.setpoint.p, d.setpoint.v, d.subtarget.p, d.subtarget.v, d.subtarget.vmax, d.subtarget.amax, [d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate]);
subtarget_eta = max(subtarget_segments(3).t(1:2));

% calculate eta from subtarget to target
if ( abs(d.subtarget.p(1) - d.target.p(1))>1e-2 || abs(d.subtarget.p(2) - d.target.p(2))>1e-2 )  % if subtarget is within 1cm to the target then assume subtarget position is end target position
    subtarget_segments = spg.setpoint.get_segments(subtarget_segments, d.subtarget.p, d.subtarget.v, d.target.p, d.target.v, d.par.vmax_move, d.par.amax_move, [d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate]);
    subtarget2target_eta = max(subtarget_segments(3).t(1:2));
else
    subtarget2target_eta = 0;
end

% calculate total eta from robot_position to target
d.target.eta = subtarget_eta + subtarget2target_eta;

end