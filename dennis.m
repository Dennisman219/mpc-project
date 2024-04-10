 %% Plant / System

 A = [-0.0151 -60.5651 0 -32.174; % 4x4 state matrix
          -0.0001 -1.3411 0.9929 0;
           0.00018 43.2541 -0.86939 0;
           0      0       1      0];

 B = [-2.516 -13.136; % 4x2 input matrix
         -0.1689 -0.2514;
         -17.251 -1.5766;
          0        0];

 C = [0 1 0 0; % 2x4 output matrix
          0 0 0 1];

 D = [0 0; % idk
          0 0];

 system = ss(A,B,C,D);

 %% Discrete System

 T = 0.01;
 discreteSystem = c2d(system,T);

 %% Constraints


 
 %% Cost function

 Q = eye(4) % Q is state cost
 R = 0.1*eye(2) % R is input cost
 % What is terminal cost (S/P?)

 [X, K, L, info] = idare(LTI.A,LTI.B,Q,R,[],[]) % X is unique stabilizing solution, K is state-feedback gain, L is closed-loop eigenvalues

 %% Time simulation

 duration = 100;
 for k = 1:duration
    
 end
