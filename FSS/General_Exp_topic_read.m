function [t_data,data] = topic_read(bag,Topic_name, Message_name)

%%% bag = rosbag('2019-08-22-17-20-44.bag');
%%% [t_angle_dxl,angle_dxl] = topic_read(bag,'/angle_dxl','Data');


temp = select(bag,'Topic',Topic_name);
read_temp = readMessages(temp,'DataFormat','struct');

for i = 1 : length(read_temp)
    data(i,:) = read_temp{i}.(Message_name);
    t_data(i,:) = temp.MessageList.Time(i);
end
