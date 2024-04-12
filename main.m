clear all 
close all
addpath('functions/')
% Linearizing for longitudinal h = 3,000 ft and M = 0.60
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

x0 = [1;0;0;0];
x_ref= [0;0;0;0];

dim.nx = 4
dim.ny = 4
dim.nu = 2
dim.N = 5

Q = 13*eye(4)
R = 0.7*eye(2)

[X, K, L, info] = idare(LTI.A,LTI.B,Q,R,[],[])

% get eigenvalues and eigenvectors
e_p = eig(X)
[V, D]  = eig(X)

% Build hyper rectangle 

% Solve quadratic program
system = LTISystem('A', LTI.A, 'B', LTI.B);
system.x.min = [-300; -90; -90; -90];
system.x.max = [300; 90; 90; 90];
system.u.min = [-25, -25];
system.u.max = [25, 25];

system.x.penalty = QuadFunction( Q );
system.u.penalty = QuadFunction( R );

P = system.LQRPenalty;
Tset = system.LQRSet


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


State_constraints_plus = kron(ones(dim.N, 1),[300; 90; 30; 90])
State_constraints_min = -1*State_constraints_plus;
% for i = 2:time    
%     min_lim_ = State_constraints_min - (P*(x1)); 
%     max_lim_ = State_constraints_plus - (P*(x1));
% 
%     Constraint = [min_lim_ <= S*u; S*u <=max_lim_; u_min <= u; u <= u_max];  % Add input constraints 
% 
% 
%     [H1,h1,const1]=costgen(P,S,Q,R,dim,(x1-x_ref));
% 
%     Objective = 0.5*u'*H1*u+h1'*u;
% 
%     optimize(Constraint,Objective);      
%     uopt=value(u);
% 
%     c1_values(:, i) = uopt(1:dim.nu);    
% 
%     x1=LTI.A*x1+LTI.B*uopt(1:dim.nu);
% 
%     x1_values(:, i) = C*x1;
% end    
% 
% fprintf("size P:")
% size(P) 
% fprintf("size S:")
% size(S)
% fprintf("size H1:")
% size(H1)
% fprintf("size h1:")
% size(h1)
% 
% plot(x1_values');
% xlabel('Iteration');
% ylabel('Value of x1');
% title('Values of x1 over iterations');
% legend('x1_1', 'x1_2', 'x1_3', 'x1_4'); % Add legend entries based on the dimensions of x1
% 
% figure;
% stairs(c1_values');
% xlabel('Iteration');
% ylabel('Value of x1');
% title('Values of x1 over iterations');
% legend('u', 'u1'); % Add legend entries based on the dimensions of x1
% 
% 