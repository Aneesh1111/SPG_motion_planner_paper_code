classdef test_superclass < matlab.unittest.TestCase
    properties
        d
        s
        npredict = 20;
        nobstacles = 78;
        obstacles = struct('p', zeros(78,2),'v', zeros(78,2),'active', zeros(78,1))
        nintercept_positions = 15;
        t
        pose = [0 -4 0];
        v_initial = [0 0 0];
        target = [0 4 0];
        skillID = 0;
        target_vel = [0 0 0];
        ball_pos = [0 0];
        ball_vel = [0 0];
        obst_vel1 = 0;    
    end
    
    methods(TestMethodSetup)
        function createFigure(testCase)
            testCase.d = spg.init(testCase.pose, testCase.v_initial, testCase.nobstacles, testCase.npredict, testCase.ball_pos, testCase.ball_vel, testCase.nintercept_positions);
            testCase.s = SPGsimulator(testCase.d);
            n = 1000;
            testCase.t = (0:n-1)'*testCase.d.par.Ts;
            drawnow
        end
    end
    
    methods(TestMethodTeardown)
        function closeFigure(testCase)
            if ishandle(testCase.s.h.fig)
                close(testCase.s.h.fig)
            end
        end
    end
    
    methods
        function runSim(testCase)
            CPPA = 0;
            testCase.s.d.setpoint.p = testCase.pose;
            setInputData(testCase.s, testCase.t, testCase.pose, testCase.v_initial, testCase.target, testCase.skillID, CPPA, testCase.obstacles.p, testCase.obstacles.v, testCase.obstacles.active, testCase.target_vel);
            run(testCase.s)
        end
    end
end

