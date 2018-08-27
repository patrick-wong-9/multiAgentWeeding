function actions = stack(actions)
    actions(size(actions,2)+1) = actions(1);
    actions(1) = []; 
end