function [P, V, A, tseg] = traj_segment(segment, time)

%t is the end time of the segment
%v is the velocity at time t
%a is the acceleration in the interval [t-Ts t]
%p is the position at time t

[t,p,v,a] = combine_segment_data(segment);

%aux variables
t_rel = bsxfun(@minus, time, t);
vt = bsxfun(@times, v, t_rel);
at = bsxfun(@times, a, t_rel);

%compute setpoints
P = bsxfun(@plus, p, vt+.5*at.*t_rel); 
V = bsxfun(@plus, v, at);
A = repmat(a,length(time),1);

tseg = bsxfun(@gt, time, t);
