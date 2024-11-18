function d = set(d)
    
%% update current subtarget

d.subtarget.p(3) = spg.subtarget.angle.set(d);
d.subtarget = spg.subtarget.check_collisionfree(d, d.subtarget, 0); 
d.subtarget.age = d.subtarget.age+1;

%% perform quickstop if required

if spg.subtarget.replan.quickstop_desired(d)
    d.subtarget = spg.subtarget.replan.quickstop(d, d.subtarget);
    d.subtarget.action = 0; %quickstop
    return
end

%% try to move to target

subtarget_target = spg.subtarget.replan.to_target(d);
if subtarget_target.collisionfree
    subtarget_target.action = 1; %move to target
    d.subtarget = subtarget_target;
    return
end

%% try to replan subtarget if desired

if spg.subtarget.replan.new_subtarget_desired(d)
    subtarget = spg.subtarget.replan.new_subtarget(d, d.subtarget);
    if subtarget.collisionfree
        subtarget.action = 2; %replan subtarget
        d.subtarget = subtarget;
        return
    end
end

%% keep collisionfree subtarget

if d.subtarget.collisionfree
    d.subtarget.action = 3; %keep subtarget
    return
end

%% else quickstop

d.subtarget = spg.subtarget.replan.quickstop(d, d.subtarget);
d.subtarget.action = 0; %quickstop
    
