function Kr = ChristoffelSymbols(obj, rbm, i, j, k)
  import casadi.*
  
  G = obj.H_matrix;
  Gsharp = obj.H_matrix_inv;
  q = rbm.States.q.sym;
  Kr = 0;
  for l = 1:rbm.Model.nd
    Kr = Kr + 1/2*Gsharp(k,l)*(jacobian(G(i,l), q(j)) + jacobian(G(j,l), q(i)) - jacobian(G(i,j), q(l)));
  end
end
