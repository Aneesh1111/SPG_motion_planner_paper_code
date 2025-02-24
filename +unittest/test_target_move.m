classdef test_target_move < unittest.test_superclass
    
    methods (Test)
        function straight(testCase)
            testCase.target = repmat([0 4 0],[1 1 1000]);
            testCase.target(1,2,150:end) = -4;
            
            runSim(testCase);
        end
        
        function varying(testCase)
            %obstacles
            m = 10;
            n = 1000;
            rng(0)
            testCase.obstacles.p(1:m,:) = randn(m,2);
            testCase.obstacles.active(1:m) = true;
            testCase.obstacles.p = repmat(testCase.obstacles.p(1:m,:,:),[1 1 n])+randn(m,2,n)*.0;
            
            %target
            a = [testCase.d.par.field_size 2*pi];
            m = 4;
            rng(0)
            targets = rand(3,m)'.*a-a*.5;
            
            testCase.target = [];
            for i=1:m
                testCase.target = cat(3, testCase.target, repmat(targets(i,:),[1 1 n/m]));
            end
            testCase.target = testCase.target + randn(size(testCase.target ))*.03;
            
            runSim(testCase);
        end
    end
end
