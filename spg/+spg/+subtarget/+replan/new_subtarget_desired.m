function replan_subtarget = new_subtarget_desired(d)

chk = get_checks(d);

if chk.hasball
    checks = [
        chk.is_colliding
        chk.is_at_subtarget_soon
        chk.age_threshold_reached
        chk.starts_with_violation
%         chk.target_outside_penalty_area_while_subtarget_inside
        %           chk.start_braking
        %           chk.rotate_slow_with_large_angle_diff
        %           chk.rotate_fast_with_small_angle_diff
        %         chk.move_to_worse_position
        %         chk.move_slowly_to_better_position
        %          chk.large_angle_diff_with_ball
        ];
    
else %moving without ball
    checks = [
        chk.is_colliding
%         chk.start_braking
        chk.age_threshold_reached
         chk.is_at_subtarget_soon
%         chk.starts_with_violation
%         chk.target_outside_penalty_area_while_subtarget_inside
%          chk.is_close_to_subtarget
        ];
end

replan_subtarget = any(checks);

function chk = get_checks(d)

arrival_margin = 1.5; %[m]
eta_margin = .5; %[s]
thr_vrotate = d.par.vmax_rotate*.1;
thr_vmove = d.par.vmax_move*.1;
age_threshold = 3/d.par.Ts;

small_angle_diff = abs(mod(d.setpoint.p(3)-d.subtarget.p(3)+pi,2*pi)-pi)<d.par.scale_angle && abs(d.setpoint.v(3))<thr_vrotate;


chk = struct(...
    'age_threshold_reached',d.subtarget.age>=age_threshold,...
    'hasball', any(d.input.robot.skillID==1:4),...
    'is_colliding', ~d.subtarget.collisionfree,...
    'start_braking', all(d.subtarget.segment_id(1:2)>=2),...
    'small_angle_diff', small_angle_diff,...
    'rotate_slow_with_large_angle_diff', ~small_angle_diff && d.subtarget.amax(3)<d.par.amax_rotate && norm(d.setpoint.v(1:2))<thr_vmove,...
    'rotate_fast_with_small_angle_diff', small_angle_diff && d.subtarget.amax(3)==d.par.amax_rotate && abs(d.setpoint.v(3))<thr_vrotate,...
    'is_close_to_subtarget', norm(d.subtarget.p(1:2)-d.setpoint.p(1:2))<arrival_margin,...
    'is_at_subtarget_soon', d.subtarget.eta<eta_margin,...
    'subtarget_at_target', norm(d.subtarget.p(1:2)-d.target.p(1:2))<arrival_margin,...
    'finished_fast_rotation', small_angle_diff && d.subtarget.amax(3)==d.par.amax_rotate,...
    'fast_rotation_needed', ~small_angle_diff && d.subtarget.amax(3)<d.par.amax_rotate && norm(d.setpoint.v(1:2))<.3,...
    'move_to_worse_position', norm(d.setpoint.p(1:2)-d.target.p(1:2))+1 < norm(d.subtarget.p(1:2)-d.target.p(1:2)),...
    'move_slowly_to_better_position', norm(d.setpoint.p(1:2)-d.target.p(1:2))-2 > norm(d.subtarget.p(1:2)-d.target.p(1:2)) && norm(d.subtarget.vmax(1:2))<.99*d.par.vmax_move && small_angle_diff,...
    'large_angle_diff_with_ball', ~small_angle_diff && d.input.robot.skillID && d.subtarget.amax(3)<d.par.amax_rotate,...
    'starts_with_violation',d.subtarget.violation_count>0,...
    'target_outside_penalty_area_while_subtarget_inside', spg.subtarget.replan.get_distance_inside_penalty_area(d,d.subtarget.p)>0 && spg.subtarget.replan.get_distance_inside_penalty_area(d,d.target.p)<eps);


