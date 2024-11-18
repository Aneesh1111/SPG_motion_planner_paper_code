function segment = get_segments(segment, p0, v0, pe, ve, vm, am, dm)

%correct for rotation
pe(3) = spg.setpoint.wrap(pe(3), p0(3));

%determine maximum speed
[v1, tmax] = get_max_speed(segment(1), p0, v0, pe, ve, vm, am, dm);

%move towards maximum speed
segment(1) = move_to_vel(segment(1), p0, v0, 0, v1, am, dm);

%move at max constant speed
segment(2) = move_at_constant_vel(segment(2), segment(1).p, segment(1).v, segment(1).t, tmax);

%move towards end speed/position
segment(3) = move_to_vel(segment(3), segment(2).p, segment(2).v, segment(2).t, ve, am, dm);

%move at max constant speed
segment(4) = move_at_constant_vel(segment(4), segment(3).p, segment(3).v, segment(3).t, [1e10 1e10 1e10]);

% %move to standstill
% segment(4) = move_to_vel(segment(4), segment(3).p, segment(3).v, segment(3).t, [0 0 0], am);
% 
% %standstill
% segment(5) = segment(4);
% segment(5).a = [0 0 0];
% segment(5).dt = [1e10 1e10 1e10];
% segment(5).t = segment(4).t+segment(5).dt;


function [v1, tmax] = get_max_speed(segment, p0, v0, pe, ve, vm, am, dm)

%accelerate to end velocity to determine correct direction
segment = move_to_vel(segment, p0, v0, 0, ve, am, dm);
direction = functions.sign(pe-segment.p);

%determine maximum speed needed when not using velocity constraint
a = direction.*am;
d = direction.*dm;

%speed to swap acceleration
acc_ratio = a./d;
numerator = v0.^2 + acc_ratio.*ve.^2 + 2.*a.*(pe-p0);
denom = 1 + acc_ratio;
% tmp = max(0,.5*v0.^2+.5*ve.^2+a.*(pe-p0));
tmp = max(0,numerator./denom);
v = sqrt(tmp); 

%determine maximum velocity while satisfying velocity limit
v1 = direction.*min(v,vm);

%determine extra time needed at maximum speed (only needed if maximum speed is exceeded)
% tmax = max(0,(v.^2-vm.^2)./dm./vm);
tmax = max(0,(v.^2-vm.^2)./2./dm./vm + (v.^2-vm.^2)./2./am./vm);

%given the case that the intial velocity is higher than the maximum velocity
sel = abs(v0)>vm;
if nnz(sel)
    tmax(sel) = (pe(sel)-segment.p(sel))./vm(sel).*direction(sel);
end

%time to brake!
sel = abs(pe-segment.p)<1e-8;
tmax(sel) = 0;
v1(sel) = v0(sel);

function segment = move_at_constant_vel(segment, p0, v0, t0, dt)

segment.dt = max(0,dt);
segment.a = zeros(size(dt));
segment.t = t0+segment.dt;
segment.p = p0+v0.*segment.dt;
segment.v = v0;

function segment = move_to_vel(segment, p0, v0, t0, ve, am, dm)
% use the correct value for acceleration/deceleration
if norm(ve) > norm(v0)
    acc = am;
else
    acc = dm;
end
segment.dt = abs(ve-v0)./acc;
segment.a = functions.sign(ve-v0).*acc;
segment.t = t0+segment.dt;
segment.p = p0+v0.*segment.dt+.5*segment.a.*segment.dt.^2;
segment.v = ve;
