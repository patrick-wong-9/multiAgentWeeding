function time = time2row(x1, x2, agent_speed)
    %calculate time to travel to next row
    time = abs(x1 - x2)/agent_speed; %T_to_row
end