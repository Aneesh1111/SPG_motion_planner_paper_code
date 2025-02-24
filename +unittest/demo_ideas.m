classdef demo_ideas < unittest.test_superclass

    methods(TestMethodSetup)
        function prep(testCase)
            testCase.skillID = 0;
        end
    end

    methods (Test)

        %drive through moving obstacle: without collision/replanning/speed loss
        function demo1(testCase)
            testCase.pose = [0 -4 0];
            n = 6;
            direction = 2*mod((1 : n)',2)-1;
            ind = (1:n)';
            testCase.obstacles.p(1 : n, :) = [-ind.*direction*.5 linspace(-2, 2, n)'];
            testCase.obstacles.v(1:n,:) = direction*[1.5 0];
            testCase.obstacles.active(1:n) = true;

            obstacles_time_sequence(testCase)
            runSim(testCase)
        end

        %crossing paths: no collisions, high speed, only minor D-tour if needed
        function demo2(testCase)

            n = length(testCase.t);
            Ts = testCase.t(2)-testCase.t(1);
            v = 2;
            distance = 8;
            periodObstacles = round(distance*2/v/Ts);
            n = 4*periodObstacles;
            testCase.t = (0:n-1)'*Ts;
            periodTarget = 9.4/Ts;

            testCase.obstacles.v = zeros(1, 2, n);
            testCase.obstacles.v(1, 1, :) = ((mod(0 : n-1, periodObstacles) < periodObstacles/2)*2-1)*v;
            testCase.obstacles.p = cumsum(testCase.obstacles.v, 3)*Ts+[-distance/2 0];
            testCase.obstacles.active(1) = true;

            testCase.target = zeros(1, 3, n);
            testCase.target(1, 2, :) = ((mod(0 : n-1, periodTarget) < periodTarget/2)*2-1)*4;

            runSim(testCase)
        end

        %intercept ball in a desired direction: to be extended with a moving target, needs some interception ETA work)
        function demo3(testCase)
            vTarget = [0 0 0];
            testCase.target = [-3 0 -pi*3/4]+permute(testCase.t, [2 3 1]).*vTarget;
            testCase.s.d.input.robot.cpb_poi_xy = testCase.target(1 : 2);
            testCase.target_vel = [.5 1 0];
            testCase.skillID = 1;

            runSim(testCase)
        end

        %intercept ball through obstacle path
        function demo4(testCase)
            testCase.target = [-2 2 -pi*3/4];
            testCase.target_vel = [1 1 0];
            testCase.s.d.input.robot.cpb_poi_xy = testCase.target(1 : 2);

            Ts = testCase.t(2)-testCase.t(1);
            testCase.obstacles.v = permute(testCase.t, [2 3 1]).*[2 0];
            testCase.obstacles.p = cumsum(testCase.obstacles.v, 3)*Ts+testCase.target(1 : 2)+[-2 -1];
            testCase.obstacles.active(1) = true;

            testCase.skillID = 1;

            runSim(testCase)
        end
    end

    methods
        function obstacles_time_sequence(testCase)
            n = length(testCase.t);
            testCase.obstacles.v = repmat(testCase.obstacles.v, [1 1 n]);
            testCase.obstacles.p = testCase.obstacles.p + cumsum(testCase.obstacles.v*testCase.d.par.Ts, 3);
        end
    end
end
