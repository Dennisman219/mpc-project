clear all 
close all

LTI.A = [-0.0151 -60.5651 0 -32.174;
     -0.0001 -1.3411 0.9929 0;
      0.00018 43.2541 -0.86939 0;
      0      0       1      0];

LTI.B = [-2.516 -13.136;
     -0.1689 -0.2514;
     -17.251 -1.5766;
      0        0];

LTI.C = [0 1 0 0;
     0 0 0 1];

LTI.D = [0 0; 0 0]

x0 = [0;0;0;0];

dim.nx = 4
dim.ny = 2
dim.nu = 2
dim.N = 2

Q = eye(4)
R = 0.1*eye(2)

[X, K, L, info] = idare(LTI.A,LTI.B,Q,R,[],[])

sys = ss(LTI.A, LTI.B, LTI.C, LTI.D)

Ts = 0.01;
sysd = c2d(sys,Ts)

[P, S]=predmodgen(LTI, dim)

Q = eye(2)
[H,h,const] = costgen(P, S, Q, R, dim, x0)


% sys_setmpc = setmpcsignals(sysd);
% mpcobj = mpc(sys_setmpc, Ts, 10);
% Tstop = 1;
% 
% num_sim_steps = round(Tstop/Ts);
% r = [ones(num_sim_steps,1), ones(num_sim_steps,1)];
% 
% sim(mpcobj,num_sim_steps,r)

% m = 16; n = 8;
% A = randn(m,n);
% b = randn(m,1);
% 

