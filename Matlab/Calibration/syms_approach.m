syms g11 g12 g13 g21 g22 g23 g31 g32 g33 m1 m2 m3 ...
    Ai11 Ai12 Ai13 Ai21 Ai22 Ai23 Ai31 Ai32 Ai33 ...
    M01 M02 M03 Mi1 Mi2 Mi3;

G = [g11 g12 g13
     g21 g22 g23
     g31 g32 g33];
 
m = [m1
     m2
     m3];
 
Ai = [Ai11 Ai12 Ai13
      Ai21 Ai22 Ai23
      Ai31 Ai32 Ai33];
  
M0 = [M01
      M02
      M03];
  
Mi = [Mi1
      Mi2
      Mi3];
  

Vi = G * M0 - Ai'*G*Mi + (Ai' - eye(3))*G*m;

Wi = Vi' * Vi

grad = [diff(Wi, g11)
        diff(Wi, g12)
        diff(Wi, g13)
        diff(Wi, g21)
        diff(Wi, g22)
        diff(Wi, g23)
        diff(Wi, g31)
        diff(Wi, g32)
        diff(Wi, g33)
        diff(Wi, m1)
        diff(Wi, m2)
        diff(Wi, m3)];
    
