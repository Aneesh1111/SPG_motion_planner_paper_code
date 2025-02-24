function [t,p,v,a] = combine_segment_data(segment)

nseg = length(segment);
ndim = length(segment(1).p);
t = zeros(1,nseg*ndim);
p = t;
v = t;
a = t;
ind = 1:ndim;
for i=1:nseg
    t(ind) = segment(i).t;
    p(ind) = segment(i).p;
    v(ind) = segment(i).v;
    a(ind) = segment(i).a;
    ind = ind+ndim;
end
