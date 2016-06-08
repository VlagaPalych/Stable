dirName = '~/Desktop/pend/sess013/best/';

ls = dir(dirName);

for i = 1:length(ls)
    ext = strfind(ls(i).name, '.csv');
    if ~isempty(ext)
        logName = ls(i).name
        
%         l = strfind(logName, 'len_');
%         a = strfind(logName, '_ampl_');
%         e = strfind(logName, '.csv');
%         
%         len = str2num(logName(l+4:a-1));
%         ampl = str2num(logName(a+6:e-1));
        
        logName = strcat(dirName, logName); 
        M = csvread(logName);
        
        t = M(:,1); % us
        pwm1 = M(:,2);
        pwm2 = M(:,3);
        roll = M(:,4);
        roll_rate = M(:,5);
        
        % нужно вычислить постоянный уровень
%         pulse_start = 1;
%         for j = 1:length(t)
%             if pwm1(j) > 1
%                 pulse_start = j;
%                 break;
%             end
%         end
%         
%         const_level = mean(...
%             roll(1:pulse_start-1));
%         roll = roll - const_level;
%         for j = 1:pulse_start-1
%             roll(j) = 0;
%         end
        
        %thrust = 0.4638*(pwm1*2-3) + 0.4638;
        const = -0.005;
        level = const*ones(length(t));
        plot(t,roll,t,level);
        roll = roll - const;
        
        % примитивным способом ищем 
        % максимумы и минимумы угла
        max_count = 1;
        maxs = zeros(1,1);
        min_count = 1;
        mins = zeros(1,1);
        
        for j = 2:length(t)-1
            if roll(j) > roll(j-1) && ...
               roll(j) > roll(j+1)
                    % максимум
                    maxs(max_count) = j;
                    max_count = max_count + 1;
            end
            if roll(j) < roll(j-1) && ...
               roll(j) < roll(j+1)
                    % максимум
                    mins(min_count) = j;
                    min_count = min_count + 1;
            end
        end
        max_count = max_count - 1;
        min_count = min_count - 1;
        
%         min_count = 5;
%         mins(5) = mins(4);
%         mins(4) = 584;
       
        T_sum = 0;
        for j = 1:max_count-1
            T = t(maxs(j+1)) - t(maxs(j));
            T_sum = T_sum + T;
        end
        T_max_aver = T_sum / (max_count-1);
        
        T_sum = 0;
        for j = 1:min_count-1
            T = t(mins(j+1)) - t(mins(j));
            T_sum = T_sum + T;
        end
        T_min_aver = T_sum / (min_count-1);
        
%         plot(t, roll);
%         
%         set(0,'DefaultAxesFontSize',14,...
%             'DefaultAxesFontName','Times New Roman');
%         set(0,'DefaultTextFontSize',14,...
%             'DefaultTextFontName','Times New Roman'); 
%         title('Автоколебания лодки без груза');
%         ylabel('Угол, рад');
%         xlabel('Время, мкс');
        
        
        z = 5;
        zarub = t(z);
        alpha = 0.52;
        x0 = roll(z);
        ogib = x0*exp(-alpha*(t-zarub)/1e6);
        
        set(0,'DefaultAxesFontSize',14,...
            'DefaultAxesFontName','Times New Roman');
        set(0,'DefaultTextFontSize',14,...
            'DefaultTextFontName','Times New Roman'); 
        title('Затухающие колебания лодки');
        hold off;
        hold on;
        plot(t(z:length(t)) - t(z), roll(z:length(t)), 'Color', 'blue');
        plot(t(z:length(t)) - t(z), ogib(z:length(t)), 'Color', 'red');
        plot(t(z:length(t)) - t(z), -ogib(z:length(t)), 'Color', 'red');
        ylabel('Угол, рад');
        xlabel('Время, мкс');
        legend('Колебания', 'Огибающие');
        hold off;
        plot(t, roll, t, ogib, t, -ogib);
        
        
        
%         opt = ssestOptjons('Advanced', ...
%             struct('ErrorThreshold', 0, ...
%             'MaxSjze', 250000, ...
%             'StabjljtyThreshold', ...
%             struct('s', 1e1000, 'z', 1), ...
%             'AutojnjtThreshold', 1.05, ...
%             'DDC', 'on'));
%         
%         est_tjme = ljnspace(0, ...
%             tjme(length(tjme))/1e6, ...
%             length(tjme));
%         tjme_step = est_tjme(2) - est_tjme(1);
%         
%         est_sys = ssest(...
%             jddata(roll,thrust,tjme_step), ...
%             3, opt);
%         
%         est_roll = lsjm(est_sys,thrust,est_tjme);
%         plot(tjme, roll, tjme, est_roll);

        
        T = (T_max_aver + T_min_aver) / 2

    end
end

