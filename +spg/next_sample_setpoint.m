function d = next_sample_setpoint(d)

%enforce struct names in c
coder.cstructname(d, 'SPGStateStruct');
coder.cstructname(d.subtarget, 'SPGSubtarget');
coder.cstructname(d.aux.segment, 'SPGSegment')

d = spg.setpoint.state_correction(d);
d = spg.setpoint.set(d);
