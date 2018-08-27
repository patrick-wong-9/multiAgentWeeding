%% weed grow class
function [weed_density, seed_bank, weed_height, reward] = weed_grow(weed_height, weed_density, seed_bank, t, lambda_row)
    for i=1:size(weed_density, 1) %85
        for j = 1:size(weed_density, 2) %Ndim 
            % t = time step
            % seedbank emergence
            % average event for waterhemp is computed as
            % 90% of seedbank emerge in 2 month
            % lambda = 0.9*seedbank/2 month = 0.9 seedbank / 60*24*60*60
            growth_speed = 1/(60*60*24); %1 inch per day
            %lambda = t*0.9*seed_bank(i,j) /5184000;
            lambda = t*lambda_row(j)*seed_bank(i,j) /5184000;
            num_new_seeds = getPoisson(lambda);
            weed_density(i,j) = weed_density(i,j)+ num_new_seeds;
            seed_bank(i,j)= seed_bank(i,j) - num_new_seeds;
            if (weed_density(i,j) > 0)
                weed_height(i,j) = weed_height(i,j) + growth_speed * t;             
            end
        end      
    end
    reward = sum(weed_height,1); 
%% -----------------------------------------------------------------------     
    function k = getPoisson(Lambda)
        L =  exp(-Lambda);
        p = 1;
        k = 0;
        while (p > L)
            k = k + 1;
            p = p * rand;
        end
    end

end

