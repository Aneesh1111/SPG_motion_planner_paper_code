function [segment, vmax, amax, dmax] = balance_xy(segment, p0, v0, pe, ve, vm, am, dm)

vmax_move = norm(vm(1:2));
amax_move = norm(am(1:2));
dmax_move = norm(dm(1:2));

%balance xy-acceleration
if all(abs(pe(1:2)-p0(1:2))>1e-8)
    a = 45;
    stepsize = a/2;
    niter = 12;
    for i=1:niter
        max_downscale = .01;
        A = max(max_downscale,[cosd(a) sind(a)]);
        amax = [amax_move*A am(3)];
        vmax = [vmax_move*A vm(3)];
        dmax = [dmax_move*A dm(3)];
        segment = spg.setpoint.get_segments(segment, p0, v0, pe, ve, vmax, amax, dmax);

        if segment(3).t(2)>segment(3).t(1)
            a = a+stepsize;
        else
            a = a-stepsize;
        end
        stepsize = stepsize/2;
    end
else
    amax = am;
    vmax = vm;
    dmax = dm;
end
