function d = traj_predict(d, segment)

%equidistant time vector
n = size(d.traj.p,1);
nvec = (1:n)';
Ts = d.par.Ts_predict;
t = nvec*Ts;

%preparations
ndof = length(segment(1).p);
nseg = length(segment);

%get time response for all segments
[P,V,A,tseg] = traj_segment(segment, t);

%determine active segment at each time instance
segment_id = sum(reshape(tseg,[n ndof nseg]),3);

%determine correct segment sample selection
ind = bsxfun(@plus, nvec, (0:ndof-1)*n); %initialize first segment
ind = ind + segment_id*n*ndof; %change if in other segments

%set output
d.traj.t(nvec) = t;
d.traj.p(nvec,:) = P(ind);
d.traj.v(nvec,:) = V(ind);
d.traj.a(nvec,:) = A(ind);
d.traj.segment_id(nvec,:) = segment_id;

%% assume that during dribble the moving orientation is already reached

if d.input.robot.skillID==1 %dribble
    sel = sum(d.traj.v.^2,2)>1e-12;
    d.traj.p(sel,3) = atan2(-d.traj.v(sel,1), d.traj.v(sel,2));
end
