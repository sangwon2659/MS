%% Sensor system validation
clear all
close all
clc
t_step=0.005;

%% Bag Read
varname = strings;
filename = "rosbag/0721Screw_Not_Tight.bag";
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

%% Combining FSS Data
FSS_Together = Data_i.FSS;
FSS_Together(:,6:10) = Data_i.FSS_;

%%
FSS_sum = sum(transpose(FSS_Together));
Initial = 1670;
Final = 3200;

for i = Initial:10:Final
   
    %Covariance Computation
    %transpose(X) declared as FFT_diff for convenience)
    
    figure(1)
    set(gcf, 'Position', [0 0 1850 1000], 'color', 'white')
    
    FFT_diff = [FSS_Together(i,:)-FSS_Together(i-1,:)];
        
    FFT_diff = abs(FFT_diff);
    temp = transpose(FFT_diff)*FFT_diff;
    Cov = sum(temp(:)) - trace(temp);
    
    subplot(3,1,1)
    stem(1, Cov, 'LineWidth', 2.5)    
    xlim([0 2])
    ylim([0 5000000000])
    grid on
    title('Covariance of Data with Window Size n=1')
    ylabel('Covariance Amplitude')
    set(gca,'xtick',[])
    if(abs(Data_i.HCmotor(i))==10)
        text(1.5, 3600000000, 'Slip', 'Color', 'red', 'FontSize', 60)
    else
        text(1.5, 3600000000, 'No Slip', 'FontSize', 60)
    end
    
    FFT_diff_2 = [FSS_Together(i,:)-FSS_Together(i-1,:);...
        FSS_Together(i-1,:)-FSS_Together(i-2,:);...
        FSS_Together(i-2,:)-FSS_Together(i-3,:);...
        FSS_Together(i-3,:)-FSS_Together(i-4,:);...
        FSS_Together(i-4,:)-FSS_Together(i-5,:)];
    
    FFT_diff_2 = abs(FFT_diff_2);
    temp_2 = transpose(FFT_diff_2)*FFT_diff_2;
    Cov_2 = sum(temp_2(:)) - trace(temp_2);
   
    subplot(3,1,2)
    stem(1, Cov_2, 'LineWidth', 2.5)    
    xlim([0 2])
    ylim([0 5000000000])
    grid on
    title('Covariance of Data with Window Size n=5')
    ylabel('Covariance Amplitude')
    set(gca,'xtick',[])
    
    FFT_diff_3 = [FSS_Together(i,:)-FSS_Together(i-1,:);...
        FSS_Together(i-1,:)-FSS_Together(i-2,:);...
        FSS_Together(i-2,:)-FSS_Together(i-3,:);...
        FSS_Together(i-3,:)-FSS_Together(i-4,:);...
        FSS_Together(i-4,:)-FSS_Together(i-5,:);...
        FSS_Together(i-5,:)-FSS_Together(i-6,:);...
        FSS_Together(i-6,:)-FSS_Together(i-7,:);...
        FSS_Together(i-7,:)-FSS_Together(i-8,:);...
        FSS_Together(i-8,:)-FSS_Together(i-9,:);...
        FSS_Together(i-9,:)-FSS_Together(i-10,:)];
        
    FFT_diff_3 = abs(FFT_diff_3);
    temp_3 = transpose(FFT_diff_3)*FFT_diff_3;
    Cov_3 = sum(temp_3(:)) - trace(temp_3);
    
    subplot(3,1,3)
    stem(1, Cov_3, 'LineWidth', 2.5)    
    xlim([0 2])
    ylim([0 5000000000])
    grid on
    title('Covariance of Data with Window Size n=10')
    ylabel('Covariance Amplitude')
    set(gca,'xtick',[])
    
    suptitle(['Covariance of Data with Window Size n=1, n=5 and n=10 (Timestep: ' num2str(i-Initial),')'])
    
    frame = getframe(1);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im,256);
    filename_ = 'Cov_n1_n5_n10.gif';
    del = 0.1;
    if(i == Initial)
      imwrite(imind,cm,filename_,'gif','Loopcount',inf,'DelayTime',del);
    else
      imwrite(imind,cm,filename_,'gif','WriteMode','append','DelayTime',del);
    end
    
end











