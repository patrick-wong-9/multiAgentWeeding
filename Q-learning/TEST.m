% test findMaxQ

available_rows = [1 1 0 1 0];


state = [3, 5];
Qrow = [1 1 0 -9 0];
[Qvals, actions] = sort(Qrow, 'descend');
for i = 1:size(state,2)
    % removing the occupied rows
    r = find(actions == state(i));
    actions(r) = [];
    Qvals(r) = [];
end
I = find(max(Qvals) == Qvals)
if(size(I,2) == 1)
    action = actions(I); 
else
    temp = datasample(I, 1); 
    action = actions(temp); 
end