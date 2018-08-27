function time= time2kill(row, weed_density, agent_speed)
    % calculate time to weed one row (traveling + time to kill) once there
    time = 209/agent_speed; % T_down_row;
    for i = 1:size(weed_density,1)
        time = time + min(120, 15*weed_density(i, row));
    end
    
end