
%% Sensor system validation
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

%%
Fs = 500;
FSS_sum = sum(transpose(Data_i.FSS));
for i = 500:10:length(FSS_sum)-Fs
    %FFT Computation
    FSS_sum_array = FSS_sum(i:i+Fs-1);
    FSS_sum_FFT = fft(FSS_sum_array);
    P2 = abs(FSS_sum_FFT/Fs);
    P1 = P2(1:Fs/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(Fs/2))/Fs;
    f = f(2:end);
    P1 = P1(2:end);
    FFT_(i-499,:) = P1;
    
    %Covariance Computation
    %transpose(X) declared as FFT_diff for convenience)
    
    %{
    FFT_diff = [Data_i.FSS(i+Fs-1,:)-Data_i.FSS(i+Fs-2,:);...
        Data_i.FSS(i+Fs-2,:)-Data_i.FSS(i+Fs-3,:);...
        Data_i.FSS(i+Fs-3,:)-Data_i.FSS(i+Fs-4,:);...
        Data_i.FSS(i+Fs-4,:)-Data_i.FSS(i+Fs-5,:);...
        Data_i.FSS(i+Fs-5,:)-Data_i.FSS(i+Fs-6,:)];
    
    FFT_diff = abs(FFT_diff);
    temp = transpose(FFT_diff)*FFT_diff;
    Cov = sum(temp(:)) - trace(temp);
    %}
    
    figure(1)
    %clf
    %{
    subplot(3,1,1)
    title('Raw FSS Data')
    ylabel('FSS Signal')
    xlabel('timesteps')
    plot(abs(FSS_sum(i-200+Fs:i+200+Fs)))
    ylim([4000000 9000000])
    hold on
    stem(200, 9000000)
    %}
    %subplot(2,1,1)
    set(gcf, 'Position', [500 500 2000 1000])
    stem(f,P1)    
    ylim([0 200000])
    grid on
    title('FFT Data')
    ylabel('FFT Amplitude')
    xlabel('Sample Frequency(Hz)')
    %subplot(2,1,2)
    %title('Slip Ground Truth Data')
    %ylabel('Slip == 10')
    %ylim([0 5000000])
    %stem(Cov)
    if(abs(Data_i.HCmotor(i+Fs-1))==10)
        text(150, 120000, 'Slip', 'Color', 'red', 'FontSize', 28)
    
    else
        text(150, 120000, 'No Slip', 'FontSize', 28)
    end
    %figure(2)
    %stem(abs(Data_i.HCmotor(i+Fs-1)))
    
end
