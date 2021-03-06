clc;
clear;

N = 6;

known_points = [0       0       1
                0       0      -1
                1       0       0
                -1      0       0
                0       1       0
                0       -1      0];

measurements = [0.033      0.010      0.937
                0.103      -0.009      -1.071
                1.076     -0.020      -0.036
                -0.927    0.006      -0.092
                0.079      1.004     -0.069
                0.048      -0.997    -0.064];             
            
%  measurements = [0.024      0.048      1.055
%                 -0.024      -0.024      -0.995
%                 0.984     0.024      0.024
%                 -0.984    0.024      0.048
%                 0.012      0.995     0.024
%                 -0.035      -0.995    0.048];           
            
% measurements = [21      1      337
%                 23      13      -174
%                 279     8      87
%                 -227    7       84
%                 17      263     83
%                 35      -248    75]; 
            
% measurements = [25      2      338
%                 30      12      -173
%                 279     9      86
%                 -228    5       83
%                 17      263     89
%                 33      -248    75]; 
            
% measurements = [29      -4      338
%                 17      14      -175
%                 279     8      87
%                 -229    7       84
%                 16      262     91
%                 34      -248    75];             
            
% measurements = [25      1      337
%                 28      10      -174
%                 279     4      93
%                 -227    5       89
%                 24      264     89
%                 28      -248    78]; 
            
% measurements = [24      1      337
%                 27      16      -174
%                 280     10      86
%                 -227    10       85
%                 24      264     82
%                 27      -248    72]; 
            
% measurements = [24      1      337
%                 27      12      -174
%                 280     11      86
%                 -227    4       81
%                 23      264     90
%                 29      -248    67];  
            
% measurements = [25      1      337
%                 27      25      -174
%                 280     16      74
%                 -227    2       85
%                 20      264     88
%                 28      -248    73];            
            
% measurements = [20      -1      338
%                 27      18      -173
%                 279     14      87
%                 -228    1       73
%                 12      263     68
%                 35      -248    80];
            
% measurements = [21      -6      335
%                 35      28      -174
%                 280     1      90
%                 -227    10       91
%                 24      263     100
%                 23      -247    96];
            
            
z3 = zeros(1, 3);

A = zeros(N*3,12);
b = zeros(N*3, 1);

for i = 1:1:N
    A(3*(i-1)+1,:) = [known_points(i,:) z3 z3 1 0 0];
    A(3*(i-1)+2,:) = [z3 known_points(i,:) z3 0 1 0];
    A(3*(i-1)+3,:) = [z3 z3 known_points(i,:) 0 0 1];
    
    for j = 1:1:3
        b(3*(i-1)+j) = measurements(i,j);
    end
end

A_eq = A' * A;
b_eq = A' * b;

teta = linsolve(A_eq, b_eq);

S = [teta(1) teta(2) teta(3)
     teta(4) teta(5) teta(6)
     teta(7) teta(8) teta(9)];
 
o = [teta(10); teta(11); teta(12)];

invS = inv(S)
o

for i = 1:1:N
    invS*(measurements(i,:)' - o);
end
