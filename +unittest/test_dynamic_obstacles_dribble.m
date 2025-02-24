classdef test_dynamic_obstacles_dribble < unittest.test_superclass
     
    methods(TestMethodSetup)
        function prep(testCase)
            testCase.skillID = 0;
        end
    end
   
    methods (Test)
        function near_field_edge(testCase)
            testCase.target = [-3.9 -2.5 0];
            testCase.obstacles.p(1:2,:) = [-3 -2;-2 -4];
            testCase.obstacles.v(1:2,:) = [0 -1;.2 1];
            testCase.obstacles.active(1:2) = true;
            
            obstacles_time_sequence(testCase)
            runSim(testCase)
        end
        
        function crossing_obstacles(testCase)
            testCase.obstacles.p(1:3,:) = [-3 0;3 2;-.3 -2];
            testCase.obstacles.v(1:3,:) = [1 0;-1 0;1 0];
            testCase.obstacles.active(1:3) = true;
            
            obstacles_time_sequence(testCase)
            runSim(testCase)
        end
        
        
        function many_crossing_obstacles(testCase)
            n = 10;
            p = [-2*ones(n/2,1) linspace(-4,3,n/2)'];
            testCase.obstacles.p(1:2:n,:) = p;
            testCase.obstacles.v(1:2:n,1) = .5;
            testCase.obstacles.p(2:2:n,:) = p.*[-1 1]+[0 (p(2,2)-p(1,2))*.5];
            testCase.obstacles.v(2:2:n,1) = -.5;
            testCase.obstacles.active(1:n) = true;
            
            obstacles_time_sequence(testCase)
            runSim(testCase)
        end
    end
    
    methods
        function obstacles_time_sequence(testCase)
            n = length(testCase.t);
            testCase.obstacles.v = repmat(testCase.obstacles.v,[1 1 n]);
            testCase.obstacles.p = testCase.obstacles.p + cumsum(testCase.obstacles.v*testCase.d.par.Ts,3);
        end
    end
end
