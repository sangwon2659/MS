% Sensor system validation
clear all
close all
clc

t_step=0.005;

%% Bag Read
varname = strings;
filename = "BagFile/2021-06-08-15-01-03.bag";
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
clear i bag temp_data k

%% Load cell data interpolation
range_temp_min = [];
range_temp_max = [];
for i = 1 : length(varname)-2
    range_temp_min = [range_temp_min min(Data.(['t_' char(varname(i))]))];
    range_temp_max = [range_temp_max max(Data.(['t_' char(varname(i))]))];
end
t_range = max(range_temp_min) : t_step : min(range_temp_max) ;
t = t_range-max(range_temp_min);

for i = 1 : length(varname)-2
    Data.(varname(i)) = double(Data.(varname(i)))
end

% interp1
for i = 1 : length(varname)-2
    Data_i.(varname(i))=interp1(Data.(['t_' char(varname(i))]),Data.(varname(i)),t_range);
end

%%
Data_i.t_HCmotor = [];
Data_i.HCmotor = [];
temp = 0;
i = 1;
k = 1;
while (i<length(Data.t_HCmotor))
   Data_i.t_HCmotor(k) = t_range(k);
   if(i==1)
       Data_i.HCmotor(k) = 0;
   else
       Data_i.HCmotor(k) = Data.HCmotor(i);
   end
   temp = temp + t_step;
   if(i<length(Data.t_HCmotor)+1)
       if(Data_i.t_HCmotor(k) >= Data.t_HCmotor(i+1))
           i=i+1;
       end
   end
   display(i);
   k = k+1;
end

%%
figure(1)
for i = 3000:20:length(t_range)
    figure(1)
    subplot(3,1,1)
    plot(Data_i.FFT(i,[2:end]))
    ylim([0 50000])
    set(gcf,'Position',[500 500 2000 1000])
    title(i)
    subplot(3,1,2)
    stem(Data_i.FSS(i,1))
    ylim([0 5000000])
    title('FSS')
    subplot(3,1,3)
    stem(Data_i.HCmotor(i))
    ylim([-15 15])
    title('Slip/NoSlip')
    pause(0.02)
end
