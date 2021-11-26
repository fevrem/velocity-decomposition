function coeff = B3_fun(obj, rbm, Y, Yperp, Yp)
  import casadi.*
  
  Vec = SX.zeros(rbm.Model.nd, 1);
  for k = 1:rbm.Model.nd
    Jac = 0;
    Krm_product = 0;
    for i = 1:rbm.Model.nd
      Jac = Jac + jacobian(Y(k), rbm.States.q.sym{i}) * Yperp(i);
      for j = 1:rbm.Model.nd
        Krm = rbm.Dynamics.Krm{k}(i,j);
        Krm_product = Krm_product + Krm*Yperp(i)*Y(j);
      end
    end
    Vec(k,1) = Jac + Krm_product;
  end
  coeff = Vec'*rbm.Dynamics.H_matrix*Yp;
end
