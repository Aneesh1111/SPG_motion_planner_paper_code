classdef test_static_obstacles_dribble < unittest.test_superclass
    
    methods(TestMethodSetup)
        function prep(testCase)
            testCase.skillID = 0;
        end
    end
    
    methods (Test)
        function near_field_edge(testCase)
            testCase.target = [-3.9 -2.5 0];
            testCase.pose = [-2 -4 -pi*.6];
            testCase.obstacles.p(1,:) = [0 -3.3];
            testCase.obstacles.active(1) = true;
            
            runSim(testCase)
        end
        
        function near_obstacle(testCase)
            testCase.target = [-1 -2.5 0];
            testCase.obstacles.p(1,:) = [0 -3.3];
            testCase.obstacles.active(1) = true;
            
            runSim(testCase)
        end
        
        function random(testCase)
            m = 5;
%             rng(1)
            testCase.obstacles.p(1:m,:) = randn(m,2);
            testCase.obstacles.active(1:m) = true;
            
            runSim(testCase)
        end
        
        function corner(testCase)
            testCase.pose = [testCase.d.par.field_size/2-.3 pi*.4];
            testCase.target = testCase.pose-[2 2 0];
            testCase.obstacles.p(1:3,:) = testCase.pose(1:2)-[1.5 .8;1 1;.8 1.5];
            testCase.obstacles.active(1:3) = true;
            
            runSim(testCase)
        end
    end
end
