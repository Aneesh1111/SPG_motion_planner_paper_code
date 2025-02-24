function update(s, d, i)

%compute subtarget trajectory
[subtarget, p_robot, p_obs] = spg.subtarget.check_collisionfree(d, d.subtarget, 0);
% d.aux.segment = spg.setpoint.get_segments(d.aux.segment, d.setpoint.p, d.setpoint.v, d.subtarget.p, [0 0 0], d.subtarget.vmax, d.subtarget.amax, d.par.dmax_move);
% d = spg.setpoint.traj_predict(d, d.aux.segment);
s.h.traj_subtarget.XData = p_robot(:,1);
s.h.traj_subtarget.YData = p_robot(:,2);
p_obs_x = permute(cat(3,p_obs(:,1,:),nan(size(p_obs,1),1)),[3 1 2]);
p_obs_y = permute(cat(3,p_obs(:,2,:),nan(size(p_obs,1),1)),[3 1 2]);
s.h.traj_obs.XData = p_obs_x(:);
s.h.traj_obs.YData = p_obs_y(:);

%compute target trajectory
subtarget_target = spg.subtarget.replan.to_target(d);
d.aux.segment = spg.setpoint.get_segments(d.aux.segment, d.setpoint.p, d.setpoint.v, subtarget_target.p, subtarget_target.v, subtarget_target.vmax, subtarget_target.amax, [d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate]);
d = spg.setpoint.traj_predict(d, d.aux.segment);
s.h.traj_target.XData = d.traj.p(:,1);
s.h.traj_target.YData = d.traj.p(:,2);
traject_to_subtarget = d.traj.p;

%update graphics
if ~mod(i,s.update_step)
    if s.d0.par.nobstacles < 1
%         s.h.obstacles.XData = 0;
%         s.h.obstacles.YData = ;
%         s.h.obstacles.ZData(:,d.input.obstacles.active) = 0;
%         s.h.obstacles.ZData(:,~d.input.obstacles.active) = nan;
    else
        s.h.obstacles.XData = bsxfun(@plus,d.input.obstacles.p(:,1)',s.h.shape_obstacle(:,1));
        s.h.obstacles.YData = bsxfun(@plus,d.input.obstacles.p(:,2)',s.h.shape_obstacle(:,2));
        s.h.obstacles.ZData(:,d.input.obstacles.active) = 0;
        s.h.obstacles.ZData(:,~d.input.obstacles.active) = nan;
    end

    set_shape(s.h.setpoint, d.setpoint.p, s.h.shape_robot);
    set_shape(s.h.collision, d.setpoint.p, s.h.shape_collision);
    set_shape(s.h.subtarget, d.subtarget.p, s.h.shape_robot);
    set_shape(s.h.target_shifted, d.target.p, s.h.shape_robot);
    set_shape(s.h.target, d.input.robot.target, s.h.shape_robot);

    if(d.input.SubtargetAvoidPolygon.valid)
        s.h.SubtargetAvoidPolygon.XData = d.input.SubtargetAvoidPolygon.polygon(1,:);
        s.h.SubtargetAvoidPolygon.YData = d.input.SubtargetAvoidPolygon.polygon(2,:);
        s.h.SubtargetAvoidPolygon.ZData = zeros(1, size(d.input.SubtargetAvoidPolygon.polygon,2));
    else
        s.h.SubtargetAvoidPolygon.XData = NaN;
        s.h.SubtargetAvoidPolygon.YData = NaN;
        s.h.SubtargetAvoidPolygon.ZData = NaN;
    end

    s.h.info.String = sprintf('time: %1.2f s\nsample: %d\n\nsubtarget:\n - p = [%s]\n - v = [%s]\n - vmax = [%s]\n - amax = [%s]\n - action = %1.0f\n - eta = [%s] s\n - segment id = [%s]\n\nsetpoint:\n - p = [%s]\n - v = [%s]\n - speed = %1.2f m/s',...
        i*d.par.Ts, i, num2str(d.subtarget.p,'%1.2f '), num2str(d.subtarget.v,'%1.2f '),num2str(d.subtarget.vmax,'%1.2f '),num2str(d.subtarget.amax,'%1.2f '),d.subtarget.action,num2str(d.subtarget.segment(3).t,'%1.2f '),num2str(d.subtarget.segment_id,'%1.0f '),...
        num2str(d.setpoint.p,'%1.2f '),num2str(d.setpoint.v,'%1.2f '), norm(d.setpoint.v(1:2)));

    drawnow
end

function set_shape(h, p, shape)

R = functions.rot(p(3));
xy = bsxfun(@plus,p(1:2)',R*shape');
h.XData = xy(1,:);
h.YData = xy(2,:);
h.ZData = zeros(1,size(xy,2));

% function update(s, d, i)
% 
% compute subtarget trajectory
% [subtarget, p_robot, p_obs] = spg.subtarget.check_collisionfree(d, d.subtarget, 0);
% d.aux.segment = spg.setpoint.get_segments(d.aux.segment, d.setpoint.p, d.setpoint.v, d.subtarget.p, [0 0 0], d.subtarget.vmax, d.subtarget.amax, d.par.dmax_move);
% d = spg.setpoint.traj_predict(d, d.aux.segment);
% s.h.traj_subtarget.XData = p_robot(:,1);
% s.h.traj_subtarget.YData = p_robot(:,2);
% p_obs_x = permute(cat(3,p_obs(:,1,:),nan(size(p_obs,1),1)),[3 1 2]);
% p_obs_y = permute(cat(3,p_obs(:,2,:),nan(size(p_obs,1),1)),[3 1 2]);
% s.h.traj_obs.XData = p_obs_x(:);
% s.h.traj_obs.YData = p_obs_y(:);
% 
% 
% compute target trajectory
% subtarget_target = spg.subtarget.replan.to_target(d);
% d.aux.segment = spg.setpoint.get_segments(d.aux.segment, d.setpoint.p, d.setpoint.v, subtarget_target.p, subtarget_target.v, subtarget_target.vmax, subtarget_target.amax, [d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate]);
% d = spg.setpoint.traj_predict(d, d.aux.segment);
% s.h.traj_target.XData = d.traj.p(:,1);
% s.h.traj_target.YData = d.traj.p(:,2);
% traject_to_subtarget = d.traj.p;
% 
% update graphics
% if ~mod(i,s.update_step)
%     if s.d0.par.nobstacles < 1
%         s.h.obstacles.XData = 0;
%         s.h.obstacles.YData = ;
%         s.h.obstacles.ZData(:,d.input.obstacles.active) = 0;
%         s.h.obstacles.ZData(:,~d.input.obstacles.active) = nan;
%     else
%         s.h.obstacles.XData = bsxfun(@plus,d.input.obstacles.p(:,1)',s.h.shape_obstacle(:,1));
%         s.h.obstacles.YData = bsxfun(@plus,d.input.obstacles.p(:,2)',s.h.shape_obstacle(:,2));
%         s.h.obstacles.ZData(:,d.input.obstacles.active) = 0;
%         s.h.obstacles.ZData(:,~d.input.obstacles.active) = nan;
%     end
%     
%     s.h.setpoint.XData = d.traj.p(1,1);
%     s.h.setpoint.YData = d.traj.p(1,2);
%     set_shape(s.h.setpoint, d.setpoint.p, s.h.shape_robot);
%     set_shape(s.h.collision, d.setpoint.p, s.h.shape_robot);
%     set_shape(s.h.subtarget, d.subtarget.p, s.h.shape_robot);
%     set_shape(s.h.target_shifted, d.target.p, s.h.shape_robot);
%     set_shape(s.h.target, d.input.robot.target, s.h.shape_robot);
% 
%     if(d.input.SubtargetAvoidPolygon.valid)
%         s.h.SubtargetAvoidPolygon.XData = d.input.SubtargetAvoidPolygon.polygon(1,:);
%         s.h.SubtargetAvoidPolygon.YData = d.input.SubtargetAvoidPolygon.polygon(2,:);
%         s.h.SubtargetAvoidPolygon.ZData = zeros(1, size(d.input.SubtargetAvoidPolygon.polygon,2));
%     else
%         s.h.SubtargetAvoidPolygon.XData = NaN;
%         s.h.SubtargetAvoidPolygon.YData = NaN;
%         s.h.SubtargetAvoidPolygon.ZData = NaN;
%     end
% 
%     s.h.info.String = sprintf('time: %1.2f s\nsample: %d\n\nsubtarget:\n - p = [%s]\n - v = [%s]\n - vmax = [%s]\n - amax = [%s]\n - action = %1.0f\n - eta = [%s] s\n - segment id = [%s]\n\nsetpoint:\n - p = [%s]\n - v = [%s]\n - speed = %1.2f m/s',...
%         i*d.par.Ts, i, num2str(d.subtarget.p,'%1.2f '), num2str(d.subtarget.v,'%1.2f '),num2str(d.subtarget.vmax,'%1.2f '),num2str(d.subtarget.amax,'%1.2f '),d.subtarget.action,num2str(d.subtarget.segment(3).t,'%1.2f '),num2str(d.subtarget.segment_id,'%1.0f '),...
%         num2str(d.setpoint.p,'%1.2f '),num2str(d.setpoint.v,'%1.2f '), norm(d.setpoint.v(1:2)));
% 
%     global traj_trials
%     if ~isempty(traj_trials)
%         if numel(traj_trials.item) > 5
%             collisionfree = arrayfun(@(item)item.subtarget.collisionfree, traj_trials.item);
%             p_robot = cat(3, traj_trials.item.p_robot);
%             p_robot = cat(1, p_robot, nan([1 size(p_robot, [2 3])]));
%             s.h.traj_trials_ok.XData = reshape(p_robot(:,1,collisionfree),[],1);
%             s.h.traj_trials_ok.YData = reshape(p_robot(:,2,collisionfree),[],1);
%             s.h.traj_trials_nok.XData = reshape(p_robot(:,1,~collisionfree),[],1);
%             s.h.traj_trials_nok.YData = reshape(p_robot(:,2,~collisionfree),[],1);
%             waitfor(msgbox('continue simulation...'))
%         else
%             s.h.traj_trials_ok.XData = nan;
%             s.h.traj_trials_ok.YData = nan;
%             s.h.traj_trials_nok.XData = nan;
%             s.h.traj_trials_nok.YData = nan;
%         end
%         traj_trials.item = [];
%     end
%     drawnow
% end
% 
% function set_shape(h, p, shape)
% 
% R = functions.rot(p(3));
% xy = bsxfun(@plus,p(1:2)',R*shape');
% h.XData = xy(1,:);
% h.YData = xy(2,:);
% h.ZData = zeros(1,size(xy,2));