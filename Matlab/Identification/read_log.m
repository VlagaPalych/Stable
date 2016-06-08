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

function [ time, pwm1, pwm2, roll, roll_rate ] ...
    = read_log( dir_name, log_name )
    
    log_name = strcat(dir_name, log_name);
    M = csvread(log_name);

    time = M(:,1); % us
    pwm1 = M(:,2);
    pwm2 = M(:,3);
    roll = M(:,14);
    roll_rate = M(:,17);
end

