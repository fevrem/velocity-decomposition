function vd = GetCoupling(vd, rbm)
  import casadi.*
  
  fprintf('\nGet coupling coefficients:\n')
  tic
  for l = 1:numel(vd.Fs)
    fprintf(['Uncontrolled velocity #', num2str(l), ' -- '])
    fprintf('B1')
    B1{l} = SX.zeros(numel(vd.F), numel(vd.F));
    for i = 1:numel(vd.F)
      for j = 1:numel(vd.F)
        B1{l}(i,j) = B1_fun(vd, rbm, vd.Y{i}, vd.Y{j}, vd.Yperp{l});
      end
    end
    vd.B1{l} = B1{l};
    
    fprintf(', B2')
    B2{l} = SX.zeros(numel(vd.F), numel(vd.Fs));
    for i = 1:numel(vd.F)
      for j = 1:numel(vd.Fs)
        B2{l}(i,j) = B2_fun(vd, rbm, vd.Y{i}, vd.Yperp{j}, vd.Yperp{l});
      end
    end
    vd.B2{l} = B2{l};
    
    fprintf(', B3')
    B3{l} = SX.zeros(numel(vd.Fs), numel(vd.F));
    for i = 1:numel(vd.Fs)
      for j = 1:numel(vd.F)     
        B3{l}(i,j) = B3_fun(vd, rbm, vd.Y{j}, vd.Yperp{i}, vd.Yperp{l});
      end
    end
    vd.B3{l} = B3{l};
    
    fprintf(', B4')
    B4{l} = SX.zeros(numel(vd.Fs), numel(vd.Fs));
    for i = 1:numel(vd.Fs)
      for j = 1:numel(vd.Fs)
        B4{l}(i,j) = B4_fun(vd, rbm, vd.Yperp{i}, vd.Yperp{j}, vd.Yperp{l});
      end
    end
    vd.B4{l} = B4{l};
    
    fprintf(', B5\n')
    B5{l} = B5_fun(vd, rbm, vd.Yperp{l});
    vd.B5{l} = B5{l};
  end
  toc
  
end
