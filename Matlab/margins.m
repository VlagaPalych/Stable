params;

boat = tf(1, [a2 0 a0])

controller = pid(-a0 + 2, 0, 0.2)

open_loop = controller * boat
closed_loop = open_loop / (1 + open_loop);

[Gm Pm, Wgm, Wpm] = margin(open_loop);
Gm_dB = 20*log10(Gm)
Pm
%step(closed_loop);
%bode(open_loop);