%% Sensor system validation
clear all
close all
clc
t_step=0.005;

%% Bag Read
varname = strings;
filename = "2021-06-08-15-01-03.bag";
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
for i = 1 : length(varname)-1
    range_temp_min = [range_temp_min min(Data.(['t_' char(varname(i))]))];
    range_temp_max = [range_temp_max max(Data.(['t_' char(varname(i))]))];
end
t_range = max(range_temp_min) : t_step : min(range_temp_max) ;
t = t_range-max(range_temp_min);

for i = 1 : length(varname)-1
    Data.(varname(i)) = double(Data.(varname(i)));
end

% interp1
for i = 1 : length(varname)-1
    Data_i.(varname(i))=interp1(Data.(['t_' char(varname(i))]),Data.(varname(i)),t_range);
end

%%
Data_i.t_HCmotor = [];
Data_i.HCmotor = [];
temp = 0;
i = 1;
k = 1;
while (i<length(Data.t_HCmotor) && k<length(t_range))
   Data_i.t_HCmotor(k) = t_range(k);
   Data_i.HCmotor(k) = Data.HCmotor(i);
   temp = temp + t_step;
   if(i<length(Data.t_HCmotor)+1)
       if(Data_i.t_HCmotor(k) >= Data.t_HCmotor(i+1))
           i=i+1;
       end
   end
   k = k+1;
end

FSS_sum = sum(transpose(Data_i.FSS));
FSS_save = [];
Initial = 4400;
Final = 5600;
k=1;
for i = Initial:Final
    Fs = 300;
    FSS_sum_array = FSS_sum(i-Fs:i);
    FSS_sum_FFT = fft(FSS_sum_array);
    P2 = abs(FSS_sum_FFT/Fs);
    P1 = P2(1:Fs/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(Fs/2))/Fs;
    f = f(2:end);
    P1 = P1(2:end);
    FSS_save(k,:) = P1;
    k=k+1;
end

p_value = [];
for i=1:Fs/2
    Slip_Data = FSS_save(400:600,i);
    NonSlip_Data = FSS_save(200:400,i);

    [h,p] = kstest2(Slip_Data, NonSlip_Data, 'Alpha', 0.05);
    
    p_value(i) = p;
end

figure(1)
f = [1:Fs/2];
%plot(p_value)
stem(f, p_value, 'LineWidth', 2.5)
title('KS Test for FTT of Sample Frequency 150Hz', 'FontSize', 24)
ylabel('p value', 'FontSize', 16)
xlabel('Sample Frequency (Hz)', 'FontSize', 16)
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    











