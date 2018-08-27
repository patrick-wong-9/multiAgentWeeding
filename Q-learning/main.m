%% main
%tasks, implement value method
% design some delta value
% switch to Q learning


clear all

%-------------------------------------------------------------------------%
%Weed Parameters
MAX_WEED = 5;       %max height agbot can cut weed
Ndim= 30;           %85; %number of rows

%Note still Ndim x Ndim grid to better emulate Javascript envir. rewards
weed_density = zeros(85, Ndim);
weed_height = zeros(85, Ndim); % = REWARD is sum of height in row, declared after
seed_min = 650;
seed_max = 1500;
%load('seedBank_30.mat') %fixed seedbank
lambda_row = 0.6:.01:.9;
%seed_bank = randi([seed_min, seed_max],85,Ndim); %min 10, max of 15
%seed_bank = ones(85,Ndim)*20; %fixed seedbank of 20

%-------------------------------------------------------------------------%
% Central Planner
N_exec = 1;                        % Number of executions of the algorithm
PEI_on = 1;                         % how many rows an agbot can see on each side
Nagents = 2; %5;                    % number of agents
N_store = 3;                       % number of total row rewards to store before checking for quasi stationary environment
zscore_th = .1;                    % zscore threshold for change in row average

agent_speed = 1;                    % feet per second
state = ones(1, Nagents);          % start at [0, 0] simulating farm
available_rows = ones(1,Ndim);      % unoccupied rows where agents can be assigned
show_row = zeros(1, Ndim);          % explored rows for information index I) --> VISIBLE ROWS
show_reward = zeros(1, Ndim);       % which rewards have known rewards

N_exp        =  zeros(1, Ndim);     % number of times a row has been visited--> resets after every Ndim rows resets
tot_rewards = zeros(1, Ndim);       % sum of rewards harvested from each row
true_N_exp = zeros(1,Ndim);         % actual count of number of times a row is visited. NEVER RESETS
true_tot_rewards = zeros(1,Ndim); % true count of rewards to calculate averages --> calculate delta

reward_history = zeros(N_store, Ndim, N_exec);  % keeps track of the avg reward each time a row is visited (may want to tune # of avgs kept)
rew_hist_idx   = ones(1, Ndim);         % keeps track of which row to enter in reward history

total_reward = 0;                      % sum of visible row rewards (for Q learning measurement)
avg_reward = 0;                         % approximate rew for unexplored rows during TARGETING EXPLORATION
agentCell = cell(1, Nagents);           % Initialize Agent Locations


%-------------------------------------------------------------------------%
% Time related parameters
days = 60;  %number of days to run 1 simulation (1 for now, should be 60)
T_delay = 2   *   (24*60*60);  %Time (seconds) before deploying robots  (1 day)
T_KILL = 15;                    %(secs) time to kill a weed seconds
T_down_row = 209/agent_speed;   %length of row / velocity
T_weed = 8*T_KILL;              %time to weed individual square

%-------------------------------------------------------------------------%
%Q Learning parameters
gamma = .9;        %time discounted rewards
eps_init = 0.2;     %epsilon greedy
eps_decay= 0;       %how much eps should decay...
alp_init = 0.99;     % Alpha - initial learning rate
alp_decay = 0.5;    % how much alp should decay
alpha = alp_init;

Keys = zeros(nchoosek(Ndim,Nagents) + Nagents-1, 1);    % use to find index in Q look up table
idxToState = zeros(size(Keys,1), Nagents);  % use to find state from index of Q table
Q_learning_count = zeros(1, N_exec);
%-------------------------------------------------------------------------%
% Initializing Q learning with Look-up Table

% NOTE: If Nagents is INCREASED FROM 2, add more nested for loops
%the following generates the Q table where each state is a unique index in
%the table. The agbot state [1, 2] is the same as [2, 1] for example.

idx = 1;
Keys(idx) = hash([1,1]);
for i = 1:Ndim
    for j = i+1:Ndim
        idx = idx + 1;
        currState = [i, j];
        idxToState(idx,:) = currState;
        Keys(idx) = hash(currState);
    end
end

data = zeros(2,N_exec); %reward in first row, episode in second
%Q = zeros(idx, Ndim); %Q TABLE for each state action pair
Weed_Environment = ones(85, Ndim, N_exec);
load('Q_Table_30_500runs_8p.mat')
freq = zeros(size(Keys,1), 1); %number of times a S-A pair has been visited

SB_data = zeros(1,N_exec);

weed_history = ones(85,30,N_exec);%keep track of field of failed weed heights

%% Plotting For Results
time_hist = [];
full_reward_hist = [];
time_rew = [];
rew = [];
ctr = 1;
delta_plot = [];
time_delta = [];

%%
tic
for ep = 1:N_exec%number of times to execute algorithm
    
    %----------------------------------------------------------------------
    % RESET Environment
    weed_density = zeros(85, Ndim);
    weed_height = zeros(85, Ndim);  % = REWARD is sum of height in row, declared later
    load('SB_6.5_15.mat')             % fixed seedbank
    lambda_row = lambda_row(randperm(length(lambda_row))); %randomly permute lambda each trial
    
    % RESET Row Arrays
    %RESET Rewards
    available_rows = ones(1,Ndim);      % unoccupied rows where agents can be assigned
    show_row = zeros(1, Ndim);          % explored rows for information index I) --> VISIBLE ROWS
    show_reward = zeros(1, Ndim);       % which rewards have known rewards
    
    N_exp        =  zeros(1, Ndim);     % number of times a row has been visited--> resets after every Ndim rows resets
    tot_rewards = zeros(1, Ndim);       % sum of rewards harvested from each row
    true_N_exp = zeros(1,Ndim);         % actual count of number of times a row is visited. NEVER RESETS
    true_tot_rewards = zeros(1,Ndim);   % true count of rewards to calculate averages --> calculate delta
    
    
    temp_rew = zeros(N_store, Ndim);        % temporary reward history for one ep
    rew_hist_idx   = ones(1, Ndim);         % keeps track of which row to enter in reward history
    
    avg_reward = 0;                         % approximate rew for unexplored rows during TARGETING EXPLORATION
    
    % Initialize Agent Locations to home position [0, 0]
    agentCell = cell(1, Nagents);
    state = ones(1, Nagents);
    
    %----------------------length of ONE experiment-------------------------
    ep %disp episode
    terminate = false;  %terminal state reached
    bestDel = 15;
    delta = 10*ones(1,Ndim);
    
    %----------------------------------------------------------------------
    
    exploration = true; % use exploration policy
    training =  true; % or test
    
    for t = 0:(60*60*24*days)
        %every hour, update weeds
        if (mod(t, 60*60) == 0)
            [weed_density, seed_bank, weed_height, R] = weed_grow(weed_height, weed_density, seed_bank, 60*60, lambda_row);
            if(size(seed_bank(seed_bank <= 0),1) == 85*30)
                disp('SEED BANK EMPTY')
                SB_data(ep) = t;
                break;
            end
        end
        
        if (t > T_delay)
            if(exploration == true)
                if(size(state(state == 1),2) == Nagents)
                    % random initial state
                    %state = randsample(30,Nagents)';
                    state =  [2,5]; %;
                    disp("Initial State: " + state)
                    for a = 1:Nagents
                        t2row =  time2row(0, state(a), agent_speed);
                        t2kill = time2kill(state(a), weed_density, agent_speed);
                        tend = t + t2row + t2kill;                          % calculate when agent will be free
                        agentCell{a} = agents(a,state(a),1,t,tend);         % initialize agent object
                        available_rows(state(a)) = 0;                       % row is occupied now
                        show_row(state(a)) = 0;                             % 0 for TIG (resets)
                        
                        tot_rewards(state(a)) = tot_rewards(state(a)) +R(state(a));     % keep track of sum of each row
                        N_exp(state(a)) = N_exp(state(a)) + 1;                          % number of times a row is explored
                        true_N_exp(state(a)) = true_N_exp(state(a)) + 1;                        %true count, never resets
                        true_tot_rewards(state(a)) = true_tot_rewards(state(a))+ R(state(a));   %true count of total reward
                        
                        temp_rew(rew_hist_idx(state(a)), state(a)) = true_tot_rewards(state(a))/ true_N_exp(state(a));
                        
                        % for plotting
                        full_reward_hist(true_N_exp(state(a)), state(a)) = true_tot_rewards(state(a))/ true_N_exp(state(a));
                        full_rew_time(true_N_exp(state(a)), state(a)) = t;
                        
                        rew = [rew, R(state(a))];
                        time_rew = [time_rew, t];
                        
                        %removes weeds from row specified and accompanying reward, R by
                        [weed_density, weed_height, R] = weedUpdate(agentCell{a}.loc , weed_density, weed_height);
                    end
                end
                
                [available_rows, show_reward] = updateRowArrays(state, show_reward, Nagents, R, PEI_on, available_rows, 0);
                
                new_state = state;
                for a = 1:Nagents
                    %if agent is free
                    if(agentCell{a}.busy == 0)
                        %run targeted exploration until delta < threshold
                        new_state(a) = getMaxValue(a, R, show_row, show_reward, state, tot_rewards, N_exp, PEI_on, agent_speed);
                        
                        %update total_rewards and N_exp
                        N_exp(new_state(a)) = N_exp(new_state(a)) + 1;           %resets after every NDim for average reward estimation
                        tot_rewards(new_state(a)) = tot_rewards(new_state(a)) + R(new_state(a)); %resets for average estimator
                        true_N_exp(new_state(a)) = true_N_exp(new_state(a)) + 1; %true count, NEVER resets
                        true_tot_rewards(new_state(a)) = true_tot_rewards(new_state(a)) + R(new_state(a));% true rewards, never resets
                        
                        %move free agent to the row and update
                        agentCell = agentUpdate(a, new_state, state, t, weed_density, agentCell, agent_speed);
                        %check to see if any weed is above the max height (5 inches)
                        terminate = terminalState(agentCell{a}.loc, weed_height, MAX_WEED);
                        
                        %%
                        
                        %%-------------------------------------for plotting -----------------------------------------------------------------------------------
                        %stores the past N_store (20) visits to each row
                        temp_rew(rew_hist_idx(new_state(a)), new_state(a)) = true_tot_rewards(new_state(a))/ true_N_exp(new_state(a));
                        total_reward = total_reward + R(new_state(a));
                        full_reward_hist(true_N_exp(new_state(a)), new_state(a)) = true_tot_rewards(new_state(a))/ true_N_exp(new_state(a));
                        full_rew_time(true_N_exp(new_state(a)), new_state(a)) = t;
                        rew = [rew, R(new_state(a))];
                        time_rew = [time_rew, t];
                        
                        
                        %%--------------------------------------------------------------------------------------------------------------------------
                        %%
                        if(length(rew) >= 5)
                            delta = std(rew(end-4:end));
                            delta_plot = [delta_plot, delta];
                            time_delta = [time_delta, t];
                            if(delta < 2)
                                delta;
                                %                                 exploration = false; %switch to q learning
                                %                                 break;
                            end                           
                        end
                        
                        %update row idx for reward history
                        if(rew_hist_idx(new_state(a)) < N_store)
                            rew_hist_idx(new_state(a)) = rew_hist_idx(new_state(a))+1;
                        else
                            rew_hist_idx(new_state(a)) = 1;
                        end
                        %{
                        %% old criteron utilizing diff of means for past 5 visits
                        %if all rows have been visited n times, see if all rows have low variance
                        %                         if(size(temp_rew(temp_rew == 0),1) == 0 )
                        %                             delta =( (( (temp_rew-mean(temp_rew))./mean(temp_rew) ).^2).^(0.5) );
                        %                             %delta = (zscore(temp).^2).^(0.5)
                        %                             %delta = sum( (zscore(temp).^2).^(1/2))/N_store;
                        %                         end
                        %                         % if(size(delta(delta < zscore_th), 2) >= 29)
                        %                         if(size(delta(delta > .10), 1) == 0)
                        %                             %disp("switch to q learning")
                        %                             weed_history(:,:,ep) = weed_height;
                        %                             reward_history(:,:,ep) = temp_rew;
                        %                             %--------------------uncomment 2 lines below
                        %                             %exploration = false; %switch to Q learning bit
                        %                             %break;
                        %                         end
                        %%
                        %}
                        
                        %after Ndim rows have been explored, reset the average reward estimator
                        if(sum(N_exp) >= Ndim)
                            show_row = zeros(1,Ndim);
                            N_exp = zeros(1,Ndim);
                            N_exp(new_state(a)) = N_exp(new_state(a)) + 1;
                            tot_rewards =zeros(1,Ndim);
                            avg_reward = 0;
                        end
                        
                        %Weeds row and Updates Weed environment
                        [weed_density, weed_height, R] = weedUpdate(agentCell{a}.loc, weed_density, weed_height);
                        
                        %UPDATE ROW ARRAYS after assigning agent a, for new agent location
                        [available_rows, show_reward] = updateRowArrays(state, show_reward, Nagents, R, false, available_rows, a);
                        show_row(new_state(a)) = 0;          % done weeding, reward not known now
                        
                        %update the current state
                        state = new_state;
                        
                    else
                        %if agent is busy, check to see if it will be free for the next time step
                        if(t == agentCell{a}.tend - 1)
                            agentCell{a}.busy = 0; %free agent up
                        end
                    end
                    if(terminate)
                        SB_data(ep) = 1;
                        weed_history(:,:,ep) = weed_height;
                        disp('WEED MAX Reached' + t)
                        break;
                    end
                end
                if(terminate)
                    break;
                end%
            end
            if(terminate == true)
                break;
            end
            
            %% Q LEARNING PORTION
            if(~exploration)
                if(size(state(state == 1),2) == Nagents)
                    % random initial state
                    state = [2,5];%randsample(30,Nagents)';
                    for a = 1:Nagents
                        t2row =  time2row(0, state(a), agent_speed);
                        t2kill = time2kill(state(a), weed_density, agent_speed);
                        tend = t + t2row + t2kill;                          % calculate when agent will be free
                        agentCell{a} = agents(a,state(a),1,t,tend);         % initialize agent object
                        %removes weeds from row specified and accompanying reward, R by
                        [weed_density, weed_height, R] = weedUpdate(agentCell{a}.loc , weed_density, weed_height);
                    end
                end
                %update row arrays iterating through all agents
                [available_rows, show_row] = updateRowArrays(state, show_row, Nagents, R, false, available_rows, 0);
                
                new_state = state;
                %epsilon-greedy exploration policy
                for a = 1:Nagents
                    s =  hash2Index( hash(state), Keys ); %current state hash INDEX
                    if(agentCell{a}.busy == 0) % Agent free to be assigned new row
                        Q_learning_count(1, ep) = Q_learning_count(1,ep) + 1;
                        freq(s,1) = freq(s,1) + 1;
                        
                        if(training)
                            if(rand > eps_init)
                                % GREEDY action
                                new_state(a) = getMaxQ(Q(s,:), state);
                            else
                                % RANDOM action (eps_init probability)
                                new_state(a) = datasample( find(available_rows == 1), 1);
                            end
                        else
                            new_state(a) = getMaxQ(Q(s,:), state);
                        end
                        
                        %% Tuned Quasi-stationary criterion
                        rew = [rew, R(new_state(a))];
                        if(length(rew) >= 8)
                            delta = std(rew(end-7:end));
                            if(delta < 2)
                                delta
                                exploration = false; %switch to q learning
                                break;
                                
                            else
                                %terminate = true;
                                disp("delta > 2")
                                exploration = true;
                                break
                            end
                        end
                        %%
                        
                        agentCell= agentUpdate(a, new_state, state, t, weed_density, agentCell, agent_speed);
                        
                        %checks to see if state is terminal
                        terminate = terminalState(agentCell{a}.loc, weed_height, MAX_WEED);
                        
                        if(terminate == true)
                            disp("Q failed: " + t)
                            break;
                        end
                        
                        total_reward = total_reward+ R(agentCell{a}.loc); %adds reward when agent is assigned new row.
                        
                        if(training)
                            s_p = hash2Index( hash(new_state), Keys );          %new state hash INDEX
                            if(mod(ep,20) == 0)
                                alpha = alp_init/(floor(ep/20))^alp_decay;
                            end
                            Q(s,new_state(a)) = Q(s,new_state(a)) +alpha*(R(new_state(a)) + gamma*(max(Q(s_p,:)) - Q(s,new_state(a))) ) ;
                            state = new_state;
                        end
                        if(terminate == true)
                            data(1,ep) = total_reward;
                            data(2,ep) = t;
                            Weed_Environment(:,:, ep) = weed_height;
                            break;
                        end
                        
                        %Weeds row and updates Weed environment
                        [weed_density, weed_height, R] = weedUpdate(agentCell{a}.loc, weed_density, weed_height);
                        available_rows(new_state(a)) = 0;   %occupied new row (so other agent can't be assigned in same timestep)
                        show_row(new_state(a)) = 0;     %unobserved after weeding
                        state = new_state;
                    else
                        %if agent is busy, check to see if it will be free for next time step
                        if(t == agentCell{a}.tend-1)
                            agentCell{a}.busy = 0; %free agent
                        end
                    end
                end
            end
            
        end
    end
end

toc

%% NESTED FUNCTIONS

function [available_rows, show_reward] = updateRowArrays(state, show_reward, Nagents, R, PEI_ON, available_rows, id)
    %UPDATE ROW ARRAYS
    Ndim = size(show_reward,2);
    
    if(id == 0)
        available_rows = ones(1,Ndim);
        for a = 1:Nagents
            left = max(1, state(a) - 1);
            right = min(Ndim, state(a) + 1);
            % check to see if row is not occupied by agbot
            if ( PEI_ON && size(find(state == left ), 2) == 0 && R(left) > 0)
                show_reward(max(1, left) ) = 1;
            end
            if ( PEI_ON && size(find(state == right), 2) == 0 && R(right) > 0)
                show_reward(min(Ndim, right)) = 1;
            end
            available_rows(state(a)) = 0;       %set to 0 so other agents won't go
            show_reward(state(a))   = 0;
        end
        
    else
        left = max(1, state(id) - 1);
        right = min(Ndim, state(id) + 1);
        % check to see if row is not occupied by agbot
        if ( PEI_ON && size(find(state == left ), 2) == 0 && R(left) > 0)
            show_reward(max(1, left) ) = 1;
        end
        if ( PEI_ON && size(find(state == right), 2) == 0 && R(right) > 0)
            show_reward(min(Ndim, right)) = 1;
        end
        available_rows(state(id)) = 0;       %set to 0 so other agents won't go
        show_reward(state(id))   = 0;
    end
end


%-------------------------------------------------------------------------%

function [weed_density, weed_height, R] = weedUpdate(row, weed_density, weed_height)
    %% updates weed environment and accompanying reward, R after agent has been assigned to a row
    weed_height(:,row) = zeros(85,1);      % assume instant weeding
    weed_density(:,row) = zeros(85,1);
    R = sum(weed_height,1);
end

%-------------------------------------------------------------------------%

function agentCell  = agentUpdate(idx, new_state, state, tstep, weed_density, agentCell, agent_speed)
    %% updates agents in the cell Array agent cell
    agentCell{idx}.loc = new_state(idx);
    tkill = time2kill(new_state(idx), weed_density, agent_speed);
    t2row =  time2row(new_state(idx), state(idx), agent_speed);
    agentCell{idx}.busy = 1; %busy, 0 is FREE
    agentCell{idx}.tstart = tstep;
    agentCell{idx}.tend = tkill + t2row + tstep;
end

%-------------------------------------------------------------------------%

function index = hash2Index(hashVal, Keys)
    %Converts from hash value to Q table SxA index
    index = find(Keys(:,1) == hashVal);
end

%-------------------------------------------------------------------------%

function hashVal = hash(arr)
    %% hashing for unique tabular index
    % How Q table is stored
    % for state [1, 5] agents are in row 1 and 5
    % --> hashValue = hash([1,5])
    % --> find(Keys == hashValue) which returns idx of Q_TABLE
    arr = sort(arr);
    hashVal = 0;
    pow = size(arr,2)-1;
    for i = 1:size(arr,2)
        hashVal = hashVal + arr(i)*31^(pow);
        pow = pow -1;
    end
    
end

%-------------------------------------------------------------------------%

function terminate = terminalState(row, weed_height, MAX_WEED)
    %% determine if there terminal state is reached
    if(size (weed_height(weed_height(:,:)> MAX_WEED), 1) > 0) %if weed greater than 5 inches, terminate
        terminate = true;
    else
        terminate = false;
    end
end

%-------------------------------------------------------------------------%