function [ val ] = cost( teta )

G = [
     0.0064    0.0018    1.3207
    0.2020   -0.2336    1.2844
    0.4109   -0.4391    1.1547
    0.7246   -0.7127    0.5758
    0.6868   -0.5910   -0.2031
    0.4469   -0.2883   -0.5594
    0.0758    0.0910   -0.6801
    0.1572    0.0965   -0.6781
    0.4119   -0.2585   -0.5830
   -0.4750    0.3127    1.0906
   -0.5314    0.2955    1.0453
   -0.8684    0.1742    0.1453
    0.1020    1.0328    0.2230
    0.1234    0.8934   -0.2027
   -0.5676    0.5475   -0.2141
   -0.7492    0.0148   -0.2008
   -0.5055    0.2529    1.0877
   -0.0387    0.6584    1.0998
    ];
val = 0;
for k = 1:1:18
    val = val + (1 - norm(transf(G(k,:), teta))^2)^2;
end


end

