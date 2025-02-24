classdef SPGsimulator < handle
    properties
        h
        d
        d0
        sim
        log = nan(1,3)
        phi = linspace(0,2*pi,16)'
        data_seed = 0
        update_step = 1
        mex_strategy = 0; %0: no, 1: yes, 2: only verify but don't use
        position
        velocity
        segment
        time
    end
    
    methods
        function s = SPGsimulator(d)
            s.d0 = d;
            s.d = d;
            s.h = initFigure(s, d);
        end
    end
end
