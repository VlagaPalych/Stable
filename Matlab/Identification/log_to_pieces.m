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

dir_name = '~/Desktop/imp/bez_gruza/';
log_name = 'log001.csv';
[time, pwm1, pwm2, roll, roll_rate] = ...
    read_log(dir_name, log_name); 

pwm1 = pwm1 / 1000;

plot(roll);
plot(time, roll, time, pwm1);

before = 4;
after = 40;

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
        cut_stop = min(length(time), ...
            stop_index + after*len);
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
        
        pulse_file = ...
        sprintf('pulses/len_%d_ampl_%d.csv',...
    len, ampl);

        cut_log(dir_name, log_name, ...
            pulse_file, cut_start, cut_stop);
    end
end






















