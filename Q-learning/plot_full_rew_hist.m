figure
rew_temp = full_reward_hist(full_reward_hist~=0);
for i = 1:30
    time = full_rew_time(:,i);
    time = time(time > 0);
    reward = full_reward_hist(:,i);
    reward = reward(reward>0); 
    plot(time,reward)
    hold on

end
title('Row rewards vs Time')
xlabel('Time (seconds)')
ylabel('Row Reward Averages')
% title('Row Reward Avg vs. Time (criterion last 5 visits < 8% normalized difference of means')
% xlabel('Time')
% ylabel('Row Reward Averages')