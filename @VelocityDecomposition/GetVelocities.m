function [vd] = GetVelocities(vd, rbm) 
  import casadi.*
  
  % Inertia matrix 
  G = rbm.Dynamics.H_matrix;
 
  fprintf('\nGet controlled velocities:\n')
  tic
  v = rbm.States.dq.sym;
  for i = 1:numel(vd.F)
    fprintf(['  - w', num2str(i), '\n'])
    w{i} = vd.Y{i}'*G*v;
    vd.w{i} = w{i};
  end
  toc
  
  fprintf('\nGet uncontrolled velocities:\n')
  tic
  for i = 1:numel(vd.Fs)
    fprintf(['  - s', num2str(i), '\n'])
    s{i} = vd.Yperp{i}'*G*v;
    vd.s{i} = s{i};
  end
  toc
end
