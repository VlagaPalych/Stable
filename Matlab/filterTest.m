function [ matlabOut, myOut ] = filterTest( fir, in )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
cf = fir.numerator;

matlabOut = filter(fir, in);

myLen = fix(length(in) / 16);
myOut = zeros(1, myLen);
myCount = 1;

for i = length(cf):16:length(in)
    y = 0;
    for j = 1:1:length(cf)
        y = y + in(i-j+1) * cf(j);
    end
    myOut(myCount) = y;
    myCount = myCount + 1;
end

end

