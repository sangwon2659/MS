function [t_data,data] = topic_read(bag,Topic_name, Message_name)

%%% bag = rosbag('2019-08-22-17-20-44.bag');
%%% [t_angle_dxl,angle_dxl] = topic_read(bag,'/angle_dxl','Data');


temp = select(bag,'Topic',Topic_name);
read_temp = readMessages(temp,'DataFormat','struct');
try
    data = cellfun(@(m) double(m.Data), read_temp);
    t_data = temp.MessageList.Time;
catch
    x = cellfun(@(m) double(m.Data), read_temp, 'UniformOutput', false);
    data = cell2mat(x')';
    t_data = temp.MessageList.Time;
end
% for i = 1 : length(read_temp)
%     data(i) = read_temp{i}.(Message_name);
%     t_data(i) = temp.MessageList.Time(i);
% end