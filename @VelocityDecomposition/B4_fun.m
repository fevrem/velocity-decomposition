function coeff = B4_fun(obj, rbm, Yperp1, Yperp2, Yp)
  import casadi.*
  
  Vec = SX.zeros(rbm.Model.nd, 1);
  for k = 1:rbm.Model.nd
    Jac = 0;
    Krm_product = 0;
    for i = 1:rbm.Model.nd
      Jac = Jac + jacobian(Yperp2(k), rbm.States.q.sym{i}) * Yperp1(i);
      for j = 1:rbm.Model.nd
        Krm = rbm.Dynamics.Krm{k}(i,j);
        Krm_product = Krm_product + Krm*Yperp1(i)*Yperp2(j);
      end
    end
    Vec(k,1) = Jac + Krm_product;
  end
  coeff = Vec'*rbm.Dynamics.H_matrix*Yp;  
end
