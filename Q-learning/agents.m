%CURRENTLY NOT USED in Q learning
classdef agents
    %% agent data structure (simplified for now)
    properties
        id  % which agent it is
        loc % location
        busy  % busy 1 for busy, 0 free
        tstart %when agent reaches row
        tend % when agent can leave (after weeding)
        % speed %
      
        % target
    end
    methods
        %constructor
        function obj = agents(id, loc, busy, tstart, tend)
            obj.id = id;
            obj.loc = loc;
            obj.busy = busy; 
            obj.tstart = tstart;
            obj.tend = tend;
        end
        
        function obj = update(t_step, newLoc)
            if(obj.tend == t_step) %#ok<NODEF>
                obj.loc =  newLoc;
            end
        end
        
    end
    
end
