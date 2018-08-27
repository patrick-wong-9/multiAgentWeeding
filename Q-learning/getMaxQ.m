function action = getMaxQ (rowQ, state)
% rowQ is a row of the State-Action Look up table Q

[Qvals, actions] = sort(rowQ, 'descend');
for i = 1:size(state,2)
    % removing the occupied rows
    r = find(actions == state(i));
    actions(r) = [];
    Qvals(r) = [];
end
I = find(max(Qvals) == Qvals);
if(size(I,2) == 1)
    action = actions(I);
else
    temp = datasample(I,1);
    action = actions(temp);
end



end