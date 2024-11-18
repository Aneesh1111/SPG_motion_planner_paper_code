function y=sign(u)

y = ones(size(u));
y(u<0) = -1;