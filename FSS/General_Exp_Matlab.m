% Sensor system validation
clear all
close all
clc

%% Bag Read
varname = strings;
% t_step = 0.005;
% cell_num = 5;
filename = "2021-06-03-16-13-43.bag";
bag = rosbag(filename);
k = 1;
for i = 1 : length(bag.AvailableTopics.Row)
    if ((string(bag.AvailableTopics.Row{i}) ~= "/rosout") && (string(bag.AvailableTopics.Row{i}) ~= "/rosout_agg"))
        if (string(bag.AvailableTopics.Row{i}) == "/FSS")
            [t_temp,temp] = topic_read(bag,bag.AvailableTopics.Row{i},'Data');
        else
            [t_temp,temp] = topic_read(bag,bag.AvailableTopics.Row{i},'Data');
        end
        Data.(['t_' bag.AvailableTopics.Row{i}(2:end)]) = t_temp;
        varname(k) = string([bag.AvailableTopics.Row{i}(2:end)]);
        Data.(varname(k)) = temp;
        k = k+1;
    end
    clear t_temp temp
end
varname(k) = 'FT_f';
% clear i bag temp_data k

%%
figure(1)
for i = 1:length(Data.FSS)
    subplot(2,1,1)
    plot(Data.FFT(i,:))
    subplot(2,1,2)
    plot(Data.FSS(i,:))
    title(i)
    pause(0.02)
end
