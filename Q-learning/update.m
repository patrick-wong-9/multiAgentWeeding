agent_locs = ones(1, Nagents);      % start at (1,1)
available_rows = ones(1,Ndim);      % unoccupied rows 
rows_visible = zeros(1, Ndim);      % which rows we know reward
%observed_rows = zeros(1, Ndim);     
total_reward = 0;                       % sum of visible row rewards
reward_avg = 0;  

%{
Need to update:
    1. Weed/Environment
        weed_density
        weed_height
        seed_bank

    2. Central Planner
        cellAgent array
        state
        new state        
        total Reward
        R
    3. Row arrays
        rows_visible
        available_rows







%}