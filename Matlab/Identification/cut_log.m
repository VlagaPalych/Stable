function [ ok ] = cut_log( dir_name, log_name, ...
    cut_name, from, to)

    [time, pwm1, pwm2, roll, roll_rate] = ...
    read_log(dir_name, log_name); 
    len = to - from;
        
    cut = zeros(len, 5);
    for k = 1:len
            cut(k,1) = time(from+k) - ...
                time(from);
            cut(k,2) = pwm1(from+k);
            cut(k,3) = pwm2(from+k);
            cut(k,4) = roll(from+k);
            cut(k,5) = roll_rate(from+k);
    end
        
    csvwrite(strcat(dir_name,cut_name), cut);
    ok = 1;
end

