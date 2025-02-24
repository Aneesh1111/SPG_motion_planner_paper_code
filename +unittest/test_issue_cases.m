classdef test_issue_cases < unittest.test_superclass
    methods (Test)
        function case1(testCase)
            testCase.pose = [-3 5 0];
            testCase.target = testCase.pose+[.0 -4 0];
            m = 1;
            n = 1000;
            testCase.obstacles.p(1,:) = testCase.pose(1:2)+[.0 -2];
            testCase.obstacles.active(1) = true;
            testCase.obstacles.active(1:m) = true;
            testCase.obstacles.p = repmat(testCase.obstacles.p(1:m,:,:),[1 1 n])+randn(m,2,n)*.1;
            
            runSim(testCase);
        end
        
        function case_local_min(testCase)            
            m = 7;
            theta = -pi/2:pi/(m-1):pi/2;
            radius = 2;
            testCase.obstacles.p(1:m,:) = [radius.*sin(theta); radius.*cos(theta)]';
            testCase.obstacles.active(1:m) = true;
            
            runSim(testCase);
        end
        
        function case_robot_drives_out_of_field(testCase)            
            m = 12;
            testCase.obstacles.p(1:m,:) = [linspace(-4,4,m); zeros(1,m)]';
            testCase.obstacles.active(1:m) = true;
            
            runSim(testCase);
        end
        
        function case_robot_trajectory_planned_outside_field(testCase)
            %obstacles
            m = 10;
            n = 500;
            rng(0)
            testCase.obstacles.p(1:m,:) = randn(m,2);
            testCase.obstacles.active(1:m) = true;
            testCase.obstacles.p = repmat(testCase.obstacles.p(1:m,:,:),[1 1 n])+randn(m,2,n)*.0;
            
            testCase.pose = [-3 0 0];
            
            %target
            a = [testCase.d.par.field_size 2*pi];
            m = 4;
            rng(0)
            targets = [3.7191   -4.1086    2.9568; 2.5178    4.8695   -2.3437; 2.5178    4.8695   -2.3437; 2.5178    4.8695   -2.3437];
            
            testCase.target = [];
            for i=1:m
                testCase.target = cat(3, testCase.target, repmat(targets(i,:),[1 1 n/m]));
            end
            testCase.target = testCase.target + randn(size(testCase.target ))*.03;
            
            runSim(testCase);
        end
        
        function case_target_with_velocity(testCase)
%             m = 5;
%             testCase.obstacles.p(1:m,:) = [linspace(-1,1,m); zeros(1,m)]';
%             testCase.obstacles.active(1:m) = true;
            
            testCase.pose = [-3 0 0];
            testCase.target = [0 6 0];
            
            testCase.target_vel = [1 1 0];
            
            runSim(testCase);
        end
        
        function case_overshoot(testCase)
            m = 5;
            testCase.obstacles.p(1:m,:) = [linspace(-1,1,m); zeros(1,m)]';
            testCase.obstacles.active(1:m) = true;
            
            testCase.pose = [3 0 0];
            testCase.target = [-3 0 0];
            
            testCase.target_vel = [0 0 0];
            
            runSim(testCase);
        end
        
        function test_collision_error_works(testCase)
            testCase.pose = [-3 5 0];
            testCase.target = testCase.pose+[.0 -4 0];
            m = 1;
            testCase.obstacles.p(1,:) = testCase.pose(1:2)+[.0 -0.1];
            testCase.obstacles.active(1) = true;
            
            runSim(testCase);
        end

    end
end
