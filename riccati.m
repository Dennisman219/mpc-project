A = [-0.0151 -60.5651 0 -32.174;
     -0.0001 -1.3411 0.9929 0;
      0.00018 43.2541 -0.86939 0;
      0      0       1      0];
B = [-2.516 -13.136;
     -0.1689 -0.2514;
     -17.251 -1.5766;
      0        0];
C = [0 1 0 0;
     0 0 0 1];
D = [0 0;
     0 0];

Q = eye(4)
R = 0.1*eye(2)

[X, K, L, info] = idare(A,B,Q,R,[],[])

sys = ss(A, B, C, D)

Ts = 0.01;
sysd = c2d(sys,Ts)

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
cvx_begin
    variable x[t] (n), t=1..10
    variable u[t] (m), t=0..9
   
cvx_end


