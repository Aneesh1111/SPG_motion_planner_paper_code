function traject = traj1(traject, segment, Ts)

%equidistant time vector
n = 1;
nvec = 1;
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
traject.t(nvec) = t;
traject.p(nvec,:) = P(ind);
traject.v(nvec,:) = V(ind);
traject.a(nvec,:) = A(ind);
traject.segment_id(nvec,:) = segment_id;
