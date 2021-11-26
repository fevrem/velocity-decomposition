function coeff = B5_fun(obj, rbm, Yp)
  import casadi.*
  
  Vec = SX.zeros(rbm.Model.nd, 1);
  for k = 1:rbm.Model.nd
    Vec(k,1) = jacobian(rbm.Dynamics.PE, rbm.States.q.sym{k});
  end
  coeff = Vec'*Yp;
end
