% computes a control invariant set for LTI system x^+ = A*x+B*u
system = LTISystem('A', [1 1; 0 0.9], 'B', [1; 0.5]);
system.x.min = [-5; -5];
system.x.max = [5; 5];
system.u.min = -1;
system.u.max = 1;
InvSet = system.invariantSet()
InvSet.plot()