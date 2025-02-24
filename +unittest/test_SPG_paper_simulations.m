classdef test_SPG_paper_simulations < unittest.test_superclass
    
    methods(TestMethodSetup)
        function prep(testCase)
            testCase.skillID = 0;
        end
    end
    
    methods (Test)
                
        function static(testCase)
            m = 5;
            max = 2;
            min = -2;
            testCase.obstacles.p(1:m,:) = (max-min).*rand(m,2) + min;
            testCase.obstacles.active(1:m) = true;
            
            runSim(testCase)
            
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end
        
        function static_jitter(testCase)
            m = 5;
            max = 2;
            min = -2;
            testCase.obstacles.p(1:m,:) = (max-min).*rand(m,2) + min;
            testCase.obstacles.active(1:m) = true;
            n = 1000;
            testCase.obstacles.p = repmat(testCase.obstacles.p(1:m,:,:),[1 1 n])+randn(m,2,n)*.1;
            
            runSim(testCase);
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end

        function dynamic1(testCase)
            m = 5;
            max_pos = 2;
            min_pos = -2;
            testCase.obstacles.p(1:m,:) = (max_pos-min_pos).*rand(m,2) + min_pos;
            max_vel = 1;
            min_vel = -1;
            testCase.obstacles.v(1:m,:) = (max_vel-min_vel).*rand(m,2) + min_vel;
            testCase.obstacles.active(1:m) = true;
            
            obstacles_time_sequence(testCase)
            runSim(testCase)
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end
        
        function dynamic1_jitter(testCase)
            m = 5;
            max_pos = 2;
            min_pos = -2;
            testCase.obstacles.p(1:m,:) = (max_pos-min_pos).*rand(m,2) + min_pos;
            max_vel = 1;
            min_vel = -1;
            testCase.obstacles.v(1:m,:) = (max_vel-min_vel).*rand(m,2) + min_vel;
            testCase.obstacles.active(1:m) = true;
            
            obstacles_time_sequence_jitter(testCase)
            runSim(testCase)
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end
        
        function dynamic2_old(testCase)
            m = 5;
            max_pos = 3;
            min_pos = -3;
            testCase.obstacles.p(1:m,2) = [2;0;-2;1;-1];%(max_pos-min_pos).*rand(m,1) + min_pos;
            testCase.obstacles.p(1:m,1) = [3;3;3;-3;-3];
            max_vel = 1.5;
            min_vel = 0.5;
            testCase.obstacles.v(1,1) = -1;
            testCase.obstacles.v(2,1) = -1;
            testCase.obstacles.v(3,1) = -1;
            testCase.obstacles.v(4,1) = 1;
            testCase.obstacles.v(5,1) = 1;
            testCase.obstacles.v(1:3,2) = zeros(3,1);
            testCase.obstacles.v(4:5,2) = zeros(2,1);
            testCase.obstacles.active(1:m) = true;
            
            obstacles_time_sequence(testCase)
            runSim(testCase)
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end
        
        function dynamic2(testCase)
%             m = 5;
%             max_pos = 3;
%             min_pos = -3;
%             testCase.obstacles.p(1:m,2) = [2;0;-2;1;-1];%(max_pos-min_pos).*rand(m,1) + min_pos;
%             testCase.obstacles.p(1:m,1) = [3;3;3;-3;-3];
%             max_vel = 1.5;
%             min_vel = 0.5;
%             
%             load("obst_sim_velocities.mat", "obst_sim_velocities")
%             testCase.obstacles.v(1:5,1) = obst_sim_velocities;
%             testCase.obstacles.v(1:3,2) = zeros(3,1);
%             testCase.obstacles.v(4:5,2) = zeros(2,1);
%             testCase.obstacles.active(1:m) = true;
%             
%             obstacles_time_sequence(testCase)
%             runSim(testCase)
%             % add info
%             collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
%             assignin('base', 'collision_dist', collision_dist);
%             obstacle_pos = testCase.s.sim.error.obstP;
%             assignin('base', 'obstacle_pos', obstacle_pos);
%             obstacle_vel = testCase.s.sim.error.obstV;
%             assignin('base', 'obstacle_vel', obstacle_vel);
%             path = testCase.s.sim.error.path;
%             assignin('base', 'path', path);
%             vel = testCase.s.sim.error.vel;
%             assignin('base', 'vel', vel);
            m = 1;
            testCase.obstacles.p(1,2) = 0;
            testCase.obstacles.p(1,1) = 3;
            
            load("obst_sim_velocities.mat", "obst_sim_velocities")
            testCase.obstacles.v(1,2) = 0;
            testCase.obstacles.v(1,1) = obst_sim_velocities(2);
            testCase.obstacles.active(1) = true;
            
            obstacles_time_sequence(testCase)
            runSim(testCase)
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
            time = testCase.s.sim.i * 0.02;
            assignin('base', 'time', time);

        end

        
        function dynamic2_jitter(testCase)
            m = 5;
            max_pos = 3;
            min_pos = -3;
            testCase.obstacles.p(1:m,2) = (max_pos-min_pos).*rand(m,1) + min_pos;
            testCase.obstacles.p(1:m,1) = [3;3;3;-3;-3];
            max_vel = 1.5;
            min_vel = 0.5;
            testCase.obstacles.v(1:3,1) = -1*((max_vel-min_vel).*rand(3,1) + min_vel);
            testCase.obstacles.v(1:3,2) = zeros(3,1);
            testCase.obstacles.v(4:5,2) = zeros(2,1);
            testCase.obstacles.v(4:5,1) = (max_vel-min_vel).*rand(2,1) + min_vel;
            testCase.obstacles.active(1:m) = true;
            
            obstacles_time_sequence_jitter(testCase)
            runSim(testCase)
        end
        
        function local_minima(testCase)    
%             testCase.pose = [0 1 0];
            m = 5;
            theta = -pi/3:pi/6:pi/3;
            radius = 2;
            testCase.obstacles.p(1:m,:) = [radius.*sin(theta); radius.*cos(theta)]';
            testCase.obstacles.p(1:m,2) = testCase.obstacles.p(1:m,2) - 5.*ones(m,1);
            testCase.obstacles.active(1:m) = true;
            
            runSim(testCase);
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end
        
        function local_minima_jitter(testCase)    
%             testCase.pose = [0 1 0];
            m = 5;
            theta = -pi/3:pi/6:pi/3;
            radius = 2;
            testCase.obstacles.p(1:m,:) = [radius.*sin(theta); radius.*cos(theta)]';
            testCase.obstacles.p(1:m,2) = testCase.obstacles.p(1:m,2) - 5.*ones(m,1);
            testCase.obstacles.active(1:m) = true;
            n = 1000;
            testCase.obstacles.p = repmat(testCase.obstacles.p(1:m,:,:),[1 1 n])+randn(m,2,n)*.1;
            
            runSim(testCase);
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end
        
        function very_dynamic1(testCase)
            m = 5;
            max_pos = 2;
            min_pos = -2;
            testCase.obstacles.p(1:m,:) = (max_pos-min_pos).*rand(m,2) + min_pos;
            max_vel = 1;
            min_vel = -1;
            testCase.obstacles.v(1:m,:) = (max_vel-min_vel).*rand(m,2) + min_vel;
            testCase.obstacles.active(1:m) = true;
            
            obstacles_acceleration(testCase)
            runSim(testCase)
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end
        
        function very_dynamic1_jitter(testCase)
            m = 5;
            max_pos = 2;
            min_pos = -2;
            testCase.obstacles.p(1:m,:) = (max_pos-min_pos).*rand(m,2) + min_pos;
            max_vel = 1;
            min_vel = -1;
            testCase.obstacles.v(1:m,:) = (max_vel-min_vel).*rand(m,2) + min_vel;
            testCase.obstacles.active(1:m) = true;
            
            obstacles_acceleration_jitter(testCase)
            runSim(testCase)
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end
        
        function very_dynamic2_old(testCase)
            m = 5;
            max_pos = 3;
            min_pos = -3;
            testCase.obstacles.p(1:m,2) = (max_pos-min_pos).*rand(m,1) + min_pos;
            testCase.obstacles.p(1:m,1) = [3;3;3;-3;-3];
            max_vel = 3;
            min_vel = -3;
            testCase.obstacles.v(1:3,1) = -1*((max_vel-min_vel).*rand(3,1) + min_vel);
            testCase.obstacles.v(1:3,2) = zeros(3,1);
            testCase.obstacles.v(4:5,2) = zeros(2,1);
            testCase.obstacles.v(4:5,1) = (max_vel-min_vel).*rand(2,1) + min_vel;
            testCase.obstacles.active(1:m) = true;
            
            obstacles_acceleration2_old(testCase)
            runSim(testCase)
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end
        
        function very_dynamic2(testCase)
            m = 5;
            testCase.obstacles.p(1:m,2) = [2;0;-2;1;-1];
            testCase.obstacles.p(1:m,1) = [3;3;3;-3;-3];
            load("obst_sim_velocities.mat", "obst_sim_velocities")
            testCase.obstacles.v(1:5,1) = obst_sim_velocities;
            testCase.obstacles.v(1:3,2) = zeros(3,1);
            testCase.obstacles.v(4:5,2) = zeros(2,1);
            testCase.obstacles.active(1:m) = true;
            
            obstacles_acceleration2(testCase)
            runSim(testCase)
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end

        
        function very_dynamic2_jitter(testCase)
            m = 5;
            max_pos = 3;
            min_pos = -3;
            testCase.obstacles.p(1:m,2) = (max_pos-min_pos).*rand(m,1) + min_pos;
            testCase.obstacles.p(1:m,1) = [3;3;3;-3;-3];
            max_vel = 1.5;
            min_vel = 0.5;
            testCase.obstacles.v(1:3,1) = -1*((max_vel-min_vel).*rand(3,1) + min_vel);
            testCase.obstacles.v(1:3,2) = zeros(3,1);
            testCase.obstacles.v(4:5,2) = zeros(2,1);
            testCase.obstacles.v(4:5,1) = (max_vel-min_vel).*rand(2,1) + min_vel;
            testCase.obstacles.active(1:m) = true;
            
            obstacles_acceleration2_jitter(testCase)
            runSim(testCase)
            % add info
            collision_dist = testCase.s.sim.error.smallest_robot2obstacle_dist;
            assignin('base', 'collision_dist', collision_dist);
            obstacle_pos = testCase.s.sim.error.obstP;
            assignin('base', 'obstacle_pos', obstacle_pos);
            obstacle_vel = testCase.s.sim.error.obstV;
            assignin('base', 'obstacle_vel', obstacle_vel);
            path = testCase.s.sim.error.path;
            assignin('base', 'path', path);
            vel = testCase.s.sim.error.vel;
            assignin('base', 'vel', vel);
        end


    end
    
    methods
        
        function obstacles_time_sequence(testCase)
            n = length(testCase.t);
            testCase.obstacles.v = repmat(testCase.obstacles.v,[1 1 n]);
            testCase.obstacles.p = testCase.obstacles.p + cumsum(testCase.obstacles.v*testCase.d.par.Ts,3);
        end
        
        function obstacles_acceleration(testCase)
            n = length(testCase.t);
            max_acc = 8;
            min_acc = -8;
            testCase.obstacles.a = zeros(size(testCase.obstacles.p));
            testCase.obstacles.v = repmat(testCase.obstacles.v,[1 1 n]);
            
            testCase.obstacles.a = (max_acc-min_acc).*rand(size(testCase.obstacles.v,1), size(testCase.obstacles.v,2), n) + min_acc;
            testCase.obstacles.v = testCase.obstacles.v + cumsum(testCase.obstacles.a*testCase.d.par.Ts, 3);
            testCase.obstacles.p = testCase.obstacles.p + cumsum(testCase.obstacles.v*testCase.d.par.Ts, 3);
        end
        
        function obstacles_acceleration2(testCase)
            n = length(testCase.t);
            load("obst_sim_acc.mat", "obst_sim_acc")
            testCase.obstacles.a = zeros(size(testCase.obstacles.p,1),size(testCase.obstacles.p,2),n);
            testCase.obstacles.v = repmat(testCase.obstacles.v,[1 1 n]);
            
            testCase.obstacles.a(1:5,1,:) = obst_sim_acc.*ones(5, 1, n);
            testCase.obstacles.a(1:3,2,:) = zeros(3,1,n);
            testCase.obstacles.a(4:5,2,:) = zeros(2,1,n);
            testCase.obstacles.v = testCase.obstacles.v + cumsum(testCase.obstacles.a*testCase.d.par.Ts, 3);
            testCase.obstacles.v = max(min(testCase.obstacles.v, 4), -4);
            disp(testCase.obstacles.v)
            testCase.obstacles.p = testCase.obstacles.p + cumsum(testCase.obstacles.v*testCase.d.par.Ts, 3);
        end
        
        function obstacles_acceleration2_old(testCase)
            n = length(testCase.t);
            max_acc = 3;
            min_acc = -3;
            testCase.obstacles.a = zeros(size(testCase.obstacles.p,1),size(testCase.obstacles.p,2),n);
            testCase.obstacles.v = repmat(testCase.obstacles.v,[1 1 n]);
            
            testCase.obstacles.a(1:3,1,:) = -1.*((max_acc-min_acc).*rand(3, 1, n) + min_acc);
            testCase.obstacles.a(1:3,2,:) = zeros(3,1,n);
            testCase.obstacles.a(4:5,2,:) = zeros(2,1,n);
            testCase.obstacles.a(4:5,1,:) = (max_acc-min_acc).*rand(2, 1, n) + min_acc;
            testCase.obstacles.v = testCase.obstacles.v + cumsum(testCase.obstacles.a*testCase.d.par.Ts, 3);
            testCase.obstacles.p = testCase.obstacles.p + cumsum(testCase.obstacles.v*testCase.d.par.Ts, 3);
        end

        
        function obstacles_time_sequence_jitter(testCase)
            n = length(testCase.t);
            testCase.obstacles.v = repmat(testCase.obstacles.v,[1 1 n]);
            nn = 1000;
            testCase.obstacles.p = testCase.obstacles.p + cumsum(testCase.obstacles.v*testCase.d.par.Ts,3) + randn(size(testCase.obstacles.v,1),2,nn)*.1;
        end
        
        function obstacles_acceleration_jitter(testCase)
            n = length(testCase.t);
            nn = 1000;
            max_acc = 8;
            min_acc = -8;
            testCase.obstacles.a = zeros(size(testCase.obstacles.p));
            testCase.obstacles.v = repmat(testCase.obstacles.v,[1 1 n]);
            
            testCase.obstacles.a = (max_acc-min_acc).*rand(size(testCase.obstacles.v,1), size(testCase.obstacles.v,2), n) + min_acc;
            testCase.obstacles.v = testCase.obstacles.v + cumsum(testCase.obstacles.a*testCase.d.par.Ts, 3);
            testCase.obstacles.p = testCase.obstacles.p + cumsum(testCase.obstacles.v*testCase.d.par.Ts, 3) + randn(size(testCase.obstacles.v,1),2,nn)*.1;
        end
        
        function obstacles_acceleration2_jitter(testCase)
            n = length(testCase.t);
            nn = 1000;
            max_acc = 3;
            min_acc = -3;
            testCase.obstacles.a = zeros(size(testCase.obstacles.p,1),size(testCase.obstacles.p,2),n);
            testCase.obstacles.v = repmat(testCase.obstacles.v,[1 1 n]);
            
            testCase.obstacles.a(1:3,1,:) = -1.*((max_acc-min_acc).*rand(3, 1, n) + min_acc);
            testCase.obstacles.a(1:3,2,:) = zeros(3,1,n);
            testCase.obstacles.a(4:5,2,:) = zeros(2,1,n);
            testCase.obstacles.a(4:5,1,:) = (max_acc-min_acc).*rand(2, 1, n) + min_acc;
            testCase.obstacles.v = testCase.obstacles.v + cumsum(testCase.obstacles.a*testCase.d.par.Ts, 3);
            testCase.obstacles.p = testCase.obstacles.p + cumsum(testCase.obstacles.v*testCase.d.par.Ts, 3) + randn(size(testCase.obstacles.v,1),2,nn)*.1;
        end

    end

end

