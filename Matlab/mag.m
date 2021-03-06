clc;
clear;

A(1,:,:) = [
    1   0   0
    0   1   0
    0   0   1];

A(2,:,:) = [
    0   0   -1
    0   1   0
    -1  0   0];

A(3,:,:) = [
    0   -1  0
    0   0   1
    -1  0   0];

A(4,:,:) = [
    0   0   -1
    0   -1   0
    -1   0   0];

A(5,:,:) = [
    0   1   0
    0   0   -1
    -1   0   0];

A(6,:,:) = [
    0   -1   0
    0   0   -1
    1   0   0];

A(7,:,:) = [
    0   0   -1
    0   -1   0
    1   0   0];

A(8,:,:) = [
    0   1   0
    0   0   1
    1   0   0];

A(9,:,:) = [
    0   0   -1
    0   1   0
    1   0   0];

A(10,:,:) = [
    0   -1   0
    1   0   0
    0   0   1];

A(11,:,:) = [
    -1   0   0
    0   -1   0
    0   0   1];

A(12,:,:) = [
    0   1   0
    -1   0   0
    0   0   1];

A(13,:,:) = [
    -1   0   0
    0   0   1
    0   1   0];

A(14,:,:) = [
    0   0   -1
    -1   0   0
    0   1   0];

A(15,:,:) = [
    1   0   0
    0   0   -1
    0   1   0];

A(16,:,:) = [
    0   0   1
    1   0   0
    0   1   0];

A(17,:,:) = [
    1   0   0
    0   0   1
    0   -1   0];

A(18,:,:) = [
    0   0   -1
    1   0   0
    0   -1   0];

A(19,:,:) = [
    -1   0   0
    0   0   -1
    0   -1   0];

A(20,:,:) = [
    0   0   1
    -1   0   0
    0   -1   0];
A(21,:,:) = [
    0   1   0
    1   0   0
    0   0   -1];

A(22,:,:) = [
    0   -1   0
    -1   0   0
    0   0   -1];

log_number = 784:1:805;

N = 20;

for i = 1:1:N
    logName = sprintf('../Logs/log%d.txt', log_number(i));
    logFile = fopen(logName, 'r');
    data = fscanf(logFile, '%f %f %f %f %f %f\n', [6 Inf]);

    mx = data(4,:)';
    my = data(5,:)';
    mz = data(6,:)';
    M(i,:) = [mean(mx) mean(my) mean(mz)]'
end















