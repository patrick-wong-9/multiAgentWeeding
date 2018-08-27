%% main
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
total_reward = 0;                       % sum of visible row rewards
reward_avg = 0;                         % approximate rew for unexplored rows
agentCell = cell(1, Nagents);           % Initialize Agent Locations
N_exec = 10;                            % Number of times to run algorithm

%-------------------------------------------------------------------------%
% Time related parameters
days = 60;  %number of days to run 1 simulation (1 for now, should be 60)
T_delay = 2*(24*60*60); %Time (seconds) before deploying robots  (1 day)
T_KILL = 15;                    %(secs) time to kill a weed seconds
T_down_row = 209/agent_speed;   %length of row / velocity
T_weed = 8*T_KILL;              %time to weed individual square

%-------------------------------------------------------------------------%
% Simple Algorithm
Keys = zeros(nchoosek(Ndim,Nagents) + Nagents-1, 1);    % use to find index in Q look up table
idxToState = zeros(size(Keys,1), Nagents);  % use to find state from index
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
freq = zeros(size(Keys,1), 1); %number of times a S-A pair has been visited
actions = 1:Ndim;
Weed_Environment = zeros(85, Ndim, N_exec);
%% Basic Algorithm
for ep = 1:N_exec %number of times to execute algorithm
    
    % RESET Environment
    weed_density = zeros(85, Ndim);
    weed_height = zeros(85, Ndim);  % = REWARD is sum of height in row, declared after
    load('seedBank_30.mat')         %fixed seedbank
    
    % RESET Row Arrays
    rows_visible = zeros(1, Ndim);          % which rows we know reward
    available_rows = ones(1,Ndim);
    
    % RESET Rewards
    total_reward = 0;                       % sum of visible row rewards
    reward_avg = 0;                         % approximate rew for unexplored rows
    
    % Initialize Agent Locations
    agentCell = cell(1, Nagents);
    state = ones(1, Nagents);
    %----------------------length of ONE experiment-------------------------
    ep
    terminate = false; %terminal state reached
    for t = 1:(60*60*24*days)
        if(terminate == true)
            %next episode
            break;
        end
        if (mod(t, 60*60) == 0) %every hour, update weeds
            [weed_density, seed_bank, weed_height, R] = weed_grow(weed_height, weed_density, seed_bank, 60*60);
        end
        
        %allow for T_delay before deploying agbots
        if (t > T_delay)
            %if at [1, 1]
            if(size(state(state == 1),2) == Nagents)
                state = [1, 2];
                for a = 1:Nagents
                    actions = stack(actions);
                    tkill = time2kill(state(a), weed_density);
                    tend = t + tkill;                                   % calculate when agent is free
                    agentCell{a} = agents(a,state(a),1,t,tend);
                end% initialize agent object
            end
            for a = 1: Nagents
                state(a) = agentCell{a}.loc; 
                new_state = state;
                %if agent is free
                if(agentCell{a}.mode == 0)
                    actions = stack(actions);                   
                    new_state(a) = actions(1);
                    
                    agentCell = agentUpdate(a, new_state, t, weed_density, agentCell);
                    [weed_density, weed_height, R] = weedUpdate(agentCell{a}.loc, weed_density, weed_height);
                    
                    terminate = terminalState(agentCell{a}.loc, weed_height, MAX_WEED);
                    if(terminate == true)
                        R(a) = -(10^12);
                    end
                    
                    total_reward = total_reward + R(agentCell{a}.loc);                
                    if(terminate == true)
                        data(1,ep) = total_reward;
                        data(2,ep) = t;
                        Weed_Environment(:,:, ep) = weed_height;
                        break;
                    end
                    
                else
                    if(t == agentCell{a}.tend)
                        agentCell{a}.mode = 0; %free agent
                    end
                end
            end            
        end      
    end
    
end






%% NESTED FUNCTIONS


function [weed_density, weed_height, R] = weedUpdate(row, weed_density, weed_height)
%% updates weed environment and accompanying reward, R after agent has been assigned to a row
weed_height(:,row) = zeros(85,1);      % assume instant weeding
weed_density(:,row) = zeros(85,1);
R = sum(weed_height,1);
end

function agentCell  = agentUpdate(idx, new_state, tstep, weed_density,agentCell)
%% updates agents in the cell Array agent cell
agentCell{idx}.loc = new_state(idx);
tkill = time2kill(new_state(idx), weed_density);
agentCell{idx}.mode = 1; %busy, 0 is FREE
agentCell{idx}.tstart = tstep;
agentCell{idx}.tend = tkill + tstep;
end


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

function terminate = terminalState(row, weed_height, MAX_WEED)
%% determine if there terminal state is reached
if(size (weed_height(weed_height(:,row)> MAX_WEED), 1) > 0) %if weed greater than 5 inches, terminate
    terminate = true;
else
    terminate = false;
end
end


function time= time2kill(row, weed_density)
%% calculate time to weed one row (traveling + time to kill)
time = 209; % T_down_row;
for i = 1:size(weed_density,1)
    time = time + min(120, 15*weed_density(i, row));
end
end

function actions = stack(actions)
actions(size(actions,2)+1) = actions(1);
actions(1) = [];
end


