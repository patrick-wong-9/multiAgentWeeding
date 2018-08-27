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
seed_min = 10;
seed_max = 100;
load('seedBank_30.mat') %fixed seedbank
lambda_row = 0.6:.01:.9;
%seed_bank = randi([seed_min, seed_max],85,Ndim); %min 10, max of 15
%seed_bank = ones(85,Ndim)*20; %fixed seedbank of 20

%-------------------------------------------------------------------------%
% Central Planner
PEI_on = 1;                         % how many rows an agbot can see on each side
Nagents = 2; %5;                    % number of agents
agent_speed = 1;                    % feet per second
state = ones(1, Nagents);           % start at (1,1)
available_rows = ones(1,Ndim);      % unoccupied rows where agents can be assigned
rows_visible = zeros(1, Ndim);      % which rows we know reward
N_exp        =  zeros(1, Ndim);     % number of times a row has been visited
total_rewards = zeros(1, Ndim);     % sum of rewards harvested from each row

total_reward = 0;                       % sum of visible row rewards
reward_avg = 0;                         % approximate rew for unexplored rows
agentCell = cell(1, Nagents);           % Initialize Agent Locations
N_exec = 10;                        % Number of executions of the algorithm

%-------------------------------------------------------------------------%
% Time related parameters
days = 60;  %number of days to run 1 simulation (1 for now, should be 60)
T_delay = 2    *   (24*60*60);  %Time (seconds) before deploying robots  (1 day)
T_KILL = 15;                    %(secs) time to kill a weed seconds
T_down_row = 209/agent_speed;   %length of row / velocity
T_weed = 8*T_KILL;              %time to weed individual square

%-------------------------------------------------------------------------%
V = zeros(1, Ndim);

%-------------------------------------------------------------------------%
%Q Learning parameters
epochs = 200;
gamma = .9;        %time discounted rewards
eps_init = 0.2;     %epsilon greedy
eps_decay= 0;       %how much eps should decay...
alp_init = 0.99;     % Alpha - initial learning rate
alp_decay = 0.5;    % how much alp should decay
alpha = alp_init;





Keys = zeros(nchoosek(Ndim,Nagents) + Nagents-1, 1);    % use to find index in Q look up table
idxToState = zeros(size(Keys,1), Nagents);  % use to find state from index

%-------------------------------------------------------------------------%
% Initializing Q learning with Look-up Table

% NOTE: If Nagents is INCREASED FROM 2, add more nested for loops
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
Q = zeros(idx, Ndim); %Q TABLE for each state action pair
Weed_Environment = zeros(85, Ndim, N_exec);
%load('Q_Table_30.mat')
freq = zeros(size(Keys,1), 1); %number of times a S-A pair has been visited
%%
% How Q table is stored
% for state [1, 5] agents are in row 1 and 5
% --> hashValue = hash([1,5])
% --> find(Keys == hashValue) which returns idx of Q_TABLE

tic
for ep = 1:N_exec %number of times to execute algorithm
    
    % RESET Environment
    weed_density = zeros(85, Ndim);
    weed_height = zeros(85, Ndim);  % = REWARD is sum of height in row, declared later
    load('seedBank_30.mat')         %fixed seedbank
    lambda_row = lambda_row(randperm(length(lambda_row))); %randomly permute lambda each trial
    
    % RESET Row Arrays
    rows_visible = zeros(1, Ndim);          % which rows we know reward / explored rows
    available_rows = ones(1,Ndim);          % unoccupied rows
    N_exp        =  zeros(1, Ndim);         % number of times a row has been visited
    total_rewards = zeros(1, Ndim);         % sum of rewards harvested from each row (all time points)
    
    % RESET Rewards
    total_reward = 0;                       % sum of visible row rewards
    reward_avg = 0;                         % approximate rew for unexplored rows
    
    % Initialize Agent Locations to home position [0, 0]
    agentCell = cell(1, Nagents);
    state = zeros(1, Nagents);
    %----------------------length of ONE experiment-------------------------
    ep %disp episode
    
    exploration = true; % use exploration policy
    terminate = false;  %terminal state reached
    for t = 0:(60*60*24*days)
        % use V(x) = R(x) * I (x) for exploration
        while(exploration)
            if (mod(t, 60*60) == 0) %every hour, update weeds
                [weed_density, seed_bank, weed_height, R] = weed_grow(weed_height, weed_density, seed_bank, 60*60);
            end
            
            if(size(state(state == 0),2) == Nagents)
                % random initial state
                state = randsample(30,2)';                
                for a = 1:Nagents
                    t2row =  time2row(1, state(a));
                    t2kill = time2kill(state(a), weed_density);
                    tend = t + t2row + t2kill;                          % calculate when agent will be free
                    agentCell{a} = agents(a,state(a),1,t,tend);         % initialize agent object
                    available_rows(state(a)) = 0;               % row is occupied now
                    total_rewards(state(a)) = R(state(a));      % keep track of sum of each row
                    N_exp(state(a)) = N_exp(state(a)) + 1;      % number of times a row is explored
                    
                    %removes weeds from row specified and accompanying reward, R by
                    [weed_density, weed_height, R] = weedUpdate(agentCell{a}.loc , weed_density, weed_height);
                end
            end
            
            for a = 1:Nagents
                if(agentCell{a}.busy == 0)
                    
                    
                end
                
            end
            
            
        end
        
        if(terminate == true)
            %next episode
            break;
        end
        
        if (mod(t, 60*60) == 0) %every hour, update weeds
            [weed_density, seed_bank, weed_height, R] = weed_grow(weed_height, weed_density, seed_bank, 60*60);
        end
        
        if (t > T_delay) %allow for T_delay before deploying agbots
            %randomly assign agents at start
            if(size(state(state == 1),2) == Nagents)
                state = (randi([1,Ndim], 1, Nagents));                  % random initial state
                
                for a = 1:Nagents
                    t2row =  time2row(1, state(a));
                    t2kill = time2kill(state(a), weed_density);
                    tend = t + t2row + t2kill;                          % calculate when agent will be free
                    agentCell{a} = agents(a,state(a),1,t,tend);         % initialize agent object
                    available_rows(agentCell{a}.loc) = 0;               % row is occupied now
                    total_reward = total_reward + R(agentCell{a}.loc);  % add to total reward
                    
                    %updates weed environment and accompanying reward, R
                    [weed_density, weed_height, R] = weedUpdate(agentCell{a}.loc , weed_density, weed_height);
                end
            end
            
            %% UPDATE row arrays
            available_rows = ones(1,Ndim);
            % update visible rows // MAY need to optimize this...
            for a = 1:Nagents
                rows_visible(max(1, state(a) - 1)) = 1;     %left row observed
                rows_visible(min(Ndim, state(a) + 1)) = 1;  %right row observed
                available_rows(agentCell{a}.loc) = 0;       %set to 0 so other agents won't go
                rows_visible(agentCell{a}.loc)   = 0;       %
                state(a) = agentCell{a}.loc;
                new_state = state;
            end
      
            
            %epsilon-greedy exploration policy
            for a = 1:Nagents
                s =  hash2Index( hash(state), Keys ); %current state hash INDEX
                freq(s,1) = freq(s,1) + 1;
                if(agentCell{a}.busy == 0) % Agent free to be assigned new row
                    
                    if(rand > eps_init)
                        % GREEDY action
                        new_state(a) = findMaxQ(Q(s,:), state);
                    else
                        % RANDOM action (eps_init probability)
                        new_state(a) = datasample( find(available_rows == 1), 1);
                    end
                    agentCell= agentUpdate(a, new_state, t, weed_density, agentCell);
                    
                    %checks to see if state is terminal
                    terminate = terminalState(agentCell{a}.loc, weed_height, MAX_WEED);
                    
                    total_reward = total_reward + R(agentCell{a}.loc); %adds reward when agent is assigned new row.
                    s_p = hash2Index( hash(new_state), Keys );          %new state hash INDEX
                    if(mod(ep,20) == 0)
                        alpha = alp_init/(floor(ep/20))^alp_decay;
                    end
                    Q(s,new_state(a)) = Q(s,new_state(a)) +alpha*(R(new_state(a)) + gamma*(max(Q(s_p,:)) - Q(s,new_state(a))) ) ;
                    state = new_state;
                    
                    if(terminate == true)
                        data(1,ep) = total_reward;
                        data(2,ep) = t;
                        Weed_Environment(:,:, ep) = weed_height;
                        break;
                    end
                    
                    %Weeds row and updates Weed environment
                    [weed_density, weed_height, R] = weedUpdate(agentCell{a}.loc, weed_density, weed_height);
                    available_rows(new_state(a)) = 0;   %occupied new row (so other agent can't be assigned in same timestep)
                    rows_visible(new_state(a)) = 0;     %unobserved after weeding
                    
                else
                    %if agent is busy, check to see if it will be free for next time step
                    if(t == agentCell{a}.tend)
                        agentCell{a}.busy = 0; %free agent
                    end
                end
                
            end
            
        end
    end
end

toc

%% NESTED FUNCTIONS

%-------------------------------------------------------------------------%

function [weed_density, weed_height, R] = weedUpdate(row, weed_density, weed_height)
%% updates weed environment and accompanying reward, R after agent has been assigned to a row

weed_height(:,row) = zeros(85,1);      % assume instant weeding
weed_density(:,row) = zeros(85,1);
R = sum(weed_height,1);
end

%-------------------------------------------------------------------------%

function agentCell  = agentUpdate(idx, new_state, tstep, weed_density,agentCell)
%% updates agents in the cell Array agent cell
agentCell{idx}.loc = new_state(idx);
tkill = time2kill(new_state(idx), weed_density);
agentCell{idx}.busy = 1; %busy, 0 is FREE
agentCell{idx}.tstart = tstep;
agentCell{idx}.tend = tkill + tstep;
end

%-------------------------------------------------------------------------%

function index = hash2Index(hashVal, Keys)
%Converts from hash value to Q table SxA index
index = find(Keys(:,1) == hashVal);
end

%-------------------------------------------------------------------------%

function hashVal = hash(arr)
%% hashing for unique tabular index
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
if(size (weed_height(weed_height(:,row)> MAX_WEED), 1) > 0) %if weed greater than 5 inches, terminate
    terminate = true;
else
    terminate = false;
end
end


%-------------------------------------------------------------------------%

function time= time2kill(row, weed_density)
% calculate time to weed one row (traveling + time to kill) once there
time = 209/agent_speed; % T_down_row;
for i = 1:size(weed_density,1)
    time = time + min(120, 15*weed_density(i, row));
end

end

%-------------------------------------------------------------------------%
function time = time2row(x1, x2)
%calculate time to travel to next row
time = abs(x1 - x2)/agent_speed; %T_to_row
end




