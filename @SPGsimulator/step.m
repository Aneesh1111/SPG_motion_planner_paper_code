function d = step(s, d)

% switch s.mex_strategy*(exist('next_sample_mex.mexa64','file')~=0)
%     case 0
        d = spg.next_sample(d); %determines target/subtarget/setpoint
    % case 1
    %     d = next_sample_mex(d); %determines target/subtarget/setpoint
    % case 2
    %     seed = rng;
    %     dCheck = next_sample_mex(d); %determines target/subtarget/setpoint
    %     rng(seed)
    %     d = spg.next_sample(d); %determines target/subtarget/setpoint
    %     assert(max(abs(d.setpoint.p-dCheck.setpoint.p))<1e-14, 'setpoint deviates for mexed version')
    %     assert(max(abs(d.subtarget.p-dCheck.subtarget.p))<1e-14, 'subtarget deviates for mexed version')
end
