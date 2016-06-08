% TIME_StartTime,
% OUT0_Out0,
% OUT0_Out1,
% OUT0_Out2,
% OUT0_Out3,
% OUT0_Out4,
% OUT0_Out5,
% OUT0_Out6,
% OUT0_Out7,
% ATT_qw,
% ATT_qx,
% ATT_qy,
% ATT_qz,
% ATT_Roll,
% ATT_Pitch,
% ATT_Yaw,
% ATT_RollRate,
% ATT_PitchRate,
% ATT_YawRate,
% ATT_GX,
% ATT_GY,
% ATT_GZ

logName = '~/Desktop/log/bez_gruza/log001.csv';

M = csvread(logName);

time = M(:,1); % us
pwm1 = M(:,2);
pwm2 = M(:,3);
roll = M(:,14);

% нужно найти и запомнить 
% постоянный уровень угла;
% пока просто визуально

% lpFilt = designfilt('lowpassfir', ...
%     'SampleRate', 100, ...
%     'PassbandFrequency', 1, ...
%     'StopbandFrequency', 2, ...
%     'StopbandAttenuation', 80, ...
%     'DesignMethod', 'kaiserwin');
% 
% filt_roll = filter(lpFilt, roll);

pwm1 = pwm1 / 1000;

plot(time, roll, time, pwm1);

before = 4;
after = 10;

start_index = -1;
for i=1:length(time)
    if pwm1(i) > 1 && start_index == -1
        start_index = i;
        ampl = pwm1(i);
    end
    
    if pwm1(i) == 1 && start_index ~= -1
        stop_index = i;
        len = stop_index - start_index;
        
        cut_start = start_index - before*len;
        for j = start_index-1:-1:cut_start
            if pwm1(j) > 1
                cut_start = j+1;
                break;
            end
        end
        cut_stop = stop_index + after*len;
        for j = stop_index+1:1:cut_stop
            if pwm1(j) > 1
                cut_stop = j-1;
                break;
            end
        end
        
        plot(time(cut_start:cut_stop), ...
            roll(cut_start:cut_stop), ...
            time(cut_start:cut_stop), ...
            pwm1(cut_start:cut_stop));
        
        start_index = -1;
        
        % записываем отрезок в файл
        cut_len = cut_stop - cut_start;
        cut = zeros(cut_len, 3);
        
        for k = 1:cut_len
            cut(k,1) = time(cut_start+k) - ...
                time(cut_start);
            cut(k,2) = pwm1(cut_start+k);
            cut(k,3) = roll(cut_start+k);
        end
        
        pulse_file = ...
            sprintf('~/Desktop/log/bez_gruza/pulses/len_%d_ampl_%d.csv', len, ampl);
        csvwrite(pulse_file, cut);
    end
end






















