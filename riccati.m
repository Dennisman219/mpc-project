clear all 
close all
% continuous linearised system
A = [-0.0151 -60.5651 0 -32.174;
     -0.0001 -1.3411 0.9929 0;
      0.00018 43.2541 -0.86939 0;
      0      0       1      0];

B = [-2.516 -13.136;
     -0.1689 -0.2514;
     -17.251 -1.5766;
      0        0];

C = eye(4)

D = [0 0; 0 0; 0 0; 0 0]

Ts = 0.1;
sys = ss(A, B, C, D)
sys_dis = c2d(sys,Ts)

% discrete systems
LTI.A = sys_dis.A
LTI.B = sys_dis.B
LTI.C = sys_dis.C
LTI.D = sys_dis.D

x0 = [0;0;0;0];
x_ref= [0;0;0;-10];

dim.nx = 4
dim.ny = 4
dim.nu = 2
dim.N = 10

Q = eye(4)
R = eye(2)


% [X, K, L, info] = idare(LTI.A,LTI.B,Q,R,[],[])


[P, S]=predmodgen(LTI, dim)
[H,h,const] = costgen(P, S, Q, R, dim, (x0))

x1 = x0;
time = 100

u = sdpvar(2*dim.N, 1);  

u_min = -25*ones(2*dim.N, 1);  % Minimum input value
u_max = 25*ones(2*dim.N, 1);   % Maximum input value
Constraint = [u_min <= u; u <= u_max];  % Add input constraints 


Objective = 0.5*u'*H*u+h'*u         %define cost function

optimize(Constraint,Objective)      %solve the problem
uopt_1=value(u)                       %assign the solution to uopt1

x1=LTI.A*x0+LTI.B*uopt_1(1:dim.nu);
x1_values = zeros(dim.nx, time);


x1_values(:, 1) = C*x1;
c1_values = zeros(dim.nu, time);

for i = 2:time
    [H1,h1,const1]=costgen(P,S,Q,R,dim,(x1-x_ref));
       
    Objective = 0.5*u'*H1*u+h1'*u;

    optimize(Constraint,Objective);      
    uopt=value(u);
    
    c1_values(:, i) = uopt(1:dim.nu);    
    
    x1=LTI.A*x1+LTI.B*uopt(1:dim.nu);

    x1_values(:, i) = C*x1;
end    

plot(x1_values');
xlabel('Iteration');
ylabel('Value of x1');
title('Values of x1 over iterations');
legend('x1_1', 'x1_2', 'x1_3', 'x1_4'); % Add legend entries based on the dimensions of x1

figure;
stairs(c1_values');
xlabel('Iteration');
ylabel('Value of x1');
title('Values of x1 over iterations');
legend('u', 'u1'); % Add legend entries based on the dimensions of x1

% figure;
% plot(uopt_1');



% cvx_begin 
%     variable uopt1(dim.nu*dim.N)
%     minimize(0.5*uopt1'*H1*uopt1+h1'*uopt1)
% cvx_end

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

