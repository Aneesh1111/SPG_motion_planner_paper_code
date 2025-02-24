classdef test_static_obstacles_move < unittest.test_superclass
    
    methods (Test)
        function obstacle_wall(testCase)
            
            m = 5;
            testCase.obstacles.p(1:m,:) = [linspace(-1,1,m); zeros(1,m)]';
            testCase.obstacles.active(1:m) = true;
            
            runSim(testCase);

        end
        
        function obstacle_two_walls(testCase)
            m1 = 5;
            m2 = 3;
            testCase.obstacles.p(1:m1+m2,:) = [linspace(-1,1,m1) linspace(-4,-2,m2); -1*ones(1,m1) 2*ones(1,m2)]';
            testCase.obstacles.active(1:m1+m2) = true;
            
            runSim(testCase);
        end
        
        function near_field_edge(testCase)
            
%             for i=1:3
            testCase.target = [-4.1 -2.5 0];
            testCase.obstacles.p(1,:) = [0 -3.3];
            testCase.obstacles.active(1) = true;
            
            runSim(testCase);
            
%             testCase.s.h.action = "restart";
%             
%             end
        end
        
        function random(testCase)
            m = 10;
            rng(0)
            testCase.obstacles.p(1:m,:) = randn(m,2);
            testCase.obstacles.active(1:m) = true;
            
            runSim(testCase);
        end
        
        function random_scrum(testCase)
            m = 10;
            rng(1)
            testCase.obstacles.p(1:m,:) = randn(m,2);
            testCase.obstacles.active(1:m) = true;
            testCase.pose = [0 0 0];
            
            runSim(testCase);
        end
        
        function keeper(testCase)
            testCase.pose = [-1 -.5*testCase.d.par.field_size(2)-.1 .5];
            testCase.s.d.par.amax_move = 4;
            testCase.target = testCase.pose+[2 0 -1];
            
            runSim(testCase);
        end
        
    end
end
