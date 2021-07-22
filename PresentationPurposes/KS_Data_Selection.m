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
FSS_Together(:,5) = 0;

%% Figure
Initial = 9400;
Final = 12000;
Fs = 40;

Slip_Data = FSS_Together(9400:9750, :);
Non_Slip_Data = FSS_Together(9918:10518, :);

FSS_sum = sum(transpose(FSS_Together));

%Force Derivative
Force_Derivative =[];
for i = Initial:Final
    Force_Derivative(i-Initial+1) = abs(FSS_sum(i)-FSS_sum(i-1));
end

%Covariance
Covariance_ = [];
for i = Initial:Final
    FFT_diff = [FSS_Together(i,:)-FSS_Together(i-1,:);...
    FSS_Together(i-1,:)-FSS_Together(i-2,:);...
    FSS_Together(i-2,:)-FSS_Together(i-3,:);...
    FSS_Together(i-3,:)-FSS_Together(i-4,:);...
    FSS_Together(i-4,:)-FSS_Together(i-5,:)];
    
    FFT_diff = abs(FFT_diff);
    temp = transpose(FFT_diff)*FFT_diff;
    Cov = sum(temp(:)) - trace(temp);
    Covariance_(i-Initial+1) = Cov;
end

%Data Plot
for i = Initial:10:Final
    
    figure(1)
    subplot(4,1,1)
    set(gcf, 'Position', [2000 -500 1000 2200], 'color', 'white')
    plot(FSS_sum(Initial:Final), 'color', 'black')
    hold on
    xline(i-Initial, 'r', 'LineWidth', 2)
    hold off
    title('Sum of FSS Data', 'FontSize', 16)
    ylabel('FSS Signal Sum', 'FontSize', 16)
    xlabel('Timestep', 'FontSize', 16)
    ylim([0 2500000])
    grid on
    
    if(abs(Data_i.HCmotor(i))==10)
        text(2000, 1800000, 'Slip', 'Color', 'red', 'FontSize', 36)
    
    else
        text(2000, 1800000, 'No Slip', 'FontSize', 36)
    end
    
    subplot(4,1,2)
    plot(Force_Derivative, 'color', [0.4660 0.6740 0.1880])
    hold on
    xline(i-Initial, 'r', 'LineWidth', 2)
    hold off
    title('Derivative of FSS Data', 'FontSize', 16)
    ylabel('FSS Derivative', 'FontSize', 16)
    xlabel('Timestep', 'FontSize', 16)
    ylim([0 100000])
    grid on
    
    subplot(4,1,3)
    plot(Covariance_, 'color', [0.4940 0.1840 0.5560])
    hold on
    xline(i-Initial, 'r', 'LineWidth', 2)
    hold off
    ylim([0 5000000000])
    grid on
    title('Covariance of Data with Window Size n=5', 'FontSize', 16)
    ylabel('Covariance Amplitude', 'FontSize', 16)
    xlabel('Timestep', 'FontSize', 16)
    set(gca,'xtick',[])
    
    FSS_sum_array = FSS_sum(i-Fs:i);
    FSS_sum_FFT = fft(FSS_sum_array);
    P2 = abs(FSS_sum_FFT/Fs);
    P1 = P2(1:Fs/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(Fs/2))/Fs;
    f = f(2:end);
    P1 = P1(2:end);
    
    subplot(4,1,4)
    stem(f,P1, 'LineWidth', 2.5, 'color', [0.8500 0.3250 0.0980])    
    xlim([0 21])
    ylim([0 200000])
    grid on
    title('FFT Data of 40 Sample Frequencies', 'FontSize', 16)
    ylabel('FFT Amplitude', 'FontSize', 16)
    xlabel('Sample Frequency(Hz)', 'FontSize', 16)
    
    h = suptitle(['Comparison of the Four Data Processing Methods (Timestep: ' num2str(i-Initial), ')'])
    set(h, 'FontSize', 20)
    
    frame = getframe(1);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im,256);
    filename_ = 'KS.gif';
    del = 0.1;
    if(i == Initial)
      imwrite(imind,cm,filename_,'gif','Loopcount',inf,'DelayTime',del);
    else
      imwrite(imind,cm,filename_,'gif','WriteMode','append','DelayTime',del);
    end
end

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    











