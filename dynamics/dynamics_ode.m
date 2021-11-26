function dx = dynamics_ode(t, x, rbm)
  % First order ODE's for the swing phase
  % Inputs:
  %  ~ t: the instant in time
  %  ~ x: vector of extended generalized coordinates and velocities
  %  ~ stepNum: the number of steps since the simulation started
  %  ~ modelParams: struct containing model parameters
  %  ~ ctrlParams: struct containing control parameters
  % Output:
  %  ~ dx: vector of velocities and accelerations
  
  NB = rbm.Model.nd;
  dx = zeros(2*NB,1);
  dx(1:NB) = x(NB+1:2*NB);
  q = x(1:NB);
  dq = x(NB+1:2*NB);
  D = H_matrix('f', q);
  C = C_terms('f', q, dq);
  B = rbm.InputMap;
  if 0 % damping
    u = -1*x(NB+1:2*NB);
  else
    u = zeros(NB,1);
  end  
  dx(NB+1:2*NB) = D\(B*u - C);
end