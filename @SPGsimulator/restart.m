function d = restart(s)

s.data_seed = str2double(s.h.data_seed.String);
d = setData(s);
rng(0)
s.h.action = "run";
