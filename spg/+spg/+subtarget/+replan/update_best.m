function best = update_best(best, subtarget_candidate, target)

% Aneesh - can we make this eta_margin dependent on velocity of robot
% instead of a static number? Also time_extrapolation.
eta_margin = .5; %[s] try to replan if close to subtarget
minimal_improvement = .5; %[m] only replan if sufficient improvement
time_extrapolation = .5; %[s] incorporate velocity such that moving towards the target is beneficial

if subtarget_candidate.collisionfree
    
    if best.collisionfree && best.eta>eta_margin
        if subtarget_candidate.violation_count<=best.violation_count ...
                && norm(subtarget_candidate.p(1:2)+subtarget_candidate.v(1:2)*time_extrapolation-target.p(1:2)) < norm(best.p(1:2)+best.v(1:2)*time_extrapolation-target.p(1:2))-minimal_improvement 
            best = subtarget_candidate;
            best.age = 0;
        end
    else
        best = subtarget_candidate;
        best.age = 0;
    end
end
