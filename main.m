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

C = eye(4);

D = [0 0; 0 0; 0 0; 0 0];

Ts = 0.1;
sys = ss(A, B, C, D);
sys_dis = c2d(sys,Ts);


% discrete systems
LTI.A = sys_dis.A;
LTI.B = sys_dis.B;
LTI.C = sys_dis.C;
LTI.D = sys_dis.D;


% reference
x0 = [0;0;0;0];
x_ref= [0;0;0;-2];


% dimensions
dim.nx = 4;
dim.ny = 4;
dim.nu = 2;
dim.N =10;
time = 15
x1_values = zeros(dim.nx, time);
c1_values = zeros(dim.nu, time);


% weighting matrices
% Q = eye(4);
% R = eye(2);
Q =   [1 0 0 0; 
    0 1 0 0; 
    0 0 1 0; 
    0 0 0 1000]
R = 0.1 * eye(2);


% discrete ricatti optimal solutions
[X, K, L, info] = idare(LTI.A,LTI.B,Q,R,[],[]);


% get the P and the S
[P, S]=prediction_matrices(LTI, dim);


% calculate Q_bar
Q_tilde = kron(eye(dim.N), Q);
R_tilde = kron(eye(dim.N), R);
Q_bar = [ Q_tilde, zeros(dim.N*4, 4); zeros(4, dim.N*4), X];


% input contraints
u = sdpvar(2*dim.N, 1);  
u_min = -25*ones(2*dim.N, 1);  
u_max = 25*ones(2*dim.N, 1);   
Constraint = [u_min <= u; u <= u_max]; 


% do one solver pass
x_bar = P*x0 + S * u
Objective = x_bar' * Q_bar * x_bar + u' * R_tilde * u  
optimize(Constraint,Objective)     
uopt_1=value(u)     
x1=LTI.A*x0+LTI.B*uopt_1(1:dim.nu);
x1_values(:, 1) = C*x1;
c1_values(:, 1) = uopt_1(1:dim.nu);


% state constraints
State_constraints_plus = kron(ones(dim.N + 1, 1),[100; 100; 50; 10]);
State_constraints_min = -1*State_constraints_plus;


% Finding the terminal set
system = LTISystem('A', LTI.A, 'B', LTI.B);
system.x.min = [-100; -100; -50; -10];
system.x.max = [100; 100; 50; 10];
system.u.min = [-25, -25];
system.u.max = [25, 25];
system.x.penalty = QuadFunction( Q );
system.u.penalty = QuadFunction( R );
Pen = system.LQRPenalty;
Tset = system.LQRSet;


% Expanding the matrices to form inequality constraints that the solver can
% take
A_t = Tset.A;
B_t=Tset.b;
A_t_bar = kron(eye(dim.N+1), A_t)
B_t_bar = kron(ones(dim.N+1, 1), B_t)

% solver settings
ops = sdpsettings('verbose',0, 'debug',0, 'showprogress', 0)

for i = 2:time  
    min_lim_ = State_constraints_min - (P*(x1)); 
    max_lim_ = State_constraints_plus - (P*(x1));

    % Add input constraints, state contraints and constraints produced by
    % the LQR set
    Constraint = [min_lim_ <= S*u; S*u <=max_lim_; u_min <= u; u <= u_max;  A_t_bar*S*u <=(B_t_bar -A_t_bar*P*x1) ]; 

    %calculate new cost
    x_bar = P*(x1-x_ref) + S * u
    Objective = 0.5*x_bar' * Q_bar * x_bar + 0.5*u' * R_tilde * u  

    optimize(Constraint,Objective, ops);      
    uopt=value(u);

    c1_values(:, i) = uopt(1:dim.nu);    

    x1=LTI.A*x1+LTI.B*uopt(1:dim.nu);

    x1_values(:, i) = C*x1;
end

plot(x1_values');
xlabel('time step');
ylabel('state values');
title('State evolution N=10');
legend('u', 'w', 'q', 'θ'); % Add legend entries based on the dimensions of x1

figure;
stairs(c1_values');
xlabel('time step');
ylabel('state values');
title('Inputs N=10');
legend('u0', 'u1'); % Add legend entries based on the dimensions of x1

% [K2,S2,e] = dlqr(LTI.A,LTI.B,Q,R) 
% figure()
% sys2 = ss((LTI.A - LTI.B*K2), zeros(4, 2),  eye(4), LTI.D )
% stepplot(sys2)



