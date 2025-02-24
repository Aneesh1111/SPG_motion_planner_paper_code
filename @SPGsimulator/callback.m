function callback(s, choice)

switch choice
    case "run_stop"
        if isequal(s.h.action,"run")
            s.h.action = "stop";
        else
            if isequal(str2double(s.h.data_seed.String), s.data_seed)
                s.h.action = "run";
            else
                s.h.action = "restart";
            end
            run(s)
        end
        
    case "restart"
        s.h.action = "restart";
        
    case "pause"
        if s.h.action=="pause"
            s.h.action = "run";
        else
            s.h.action = "pause";
        end
       
    case "step"
        s.h.action = "step";
        
    case "random_seed_minus"
        set(s.h.data_seed,'string',num2str(max(0, s.data_seed-1)))
        s.h.action = "restart";
        
    case "random_seed_plus"
        set(s.h.data_seed,'string',num2str(s.data_seed+1))
        s.h.action = "restart";
        
    case "set_seed"
        value = str2double(s.h.data_seed.String);
        if isnumeric(value) && isfinite(value)
            s.h.action = "restart";
        else
            set(s.h.data_seed,'string',num2str(s.data_seed))
        end
        
    case "run_simulink"
        s.h.action = "stop";
        runSimulinkModel(s);
        
    case "quit"
        s.h.action = "stop";
        
end
drawnow
