function vd = GetCouplingDW(vd, rbm)
  import casadi.*
  
  fprintf('\nGet coupling coefficients for dw/dt:\n')
  tic
  for l = 1:numel(vd.F)
    fprintf(['Controlled velocity #', num2str(l), ' -- '])
    fprintf('A1')
    A1{l} = SX.zeros(numel(vd.F), numel(vd.F));
    for i = 1:numel(vd.F)
      for j = 1:numel(vd.F)
        A1{l}(i,j) = B1_fun(vd, rbm, vd.Y{i}, vd.Y{j}, vd.Y{l});
      end
    end
    vd.A1{l} = A1{l};
    
    fprintf(', A2')
    A2{l} = SX.zeros(numel(vd.F), numel(vd.Fs));
    for i = 1:numel(vd.F)        
      for j = 1:numel(vd.Fs)
        A2{l}(i,j) = B2_fun(vd, rbm, vd.Y{i}, vd.Yperp{j}, vd.Y{l});
      end
    end
    vd.A2{l} = A2{l};
    
    fprintf(', A3')
    A3{l} = SX.zeros(numel(vd.Fs), numel(vd.F));
    for i = 1:numel(vd.Fs)            
      for j = 1:numel(vd.F)     
        A3{l}(i,j) = B3_fun(vd, rbm, vd.Y{j}, vd.Yperp{i}, vd.Y{l});
      end
    end
    vd.A3{l} = A3{l};
    
    fprintf(', A4')
    A4{l} = SX.zeros(numel(vd.Fs), numel(vd.Fs));
    for i = 1:numel(vd.Fs)
      for j = 1:numel(vd.Fs)
        A4{l}(i,j) = B4_fun(vd, rbm, vd.Yperp{i}, vd.Yperp{j}, vd.Y{l});
      end
    end
    vd.A4{l} = A4{l};
    
    fprintf(', A5\n')
    A5{l} = B5_fun(vd, rbm, vd.Y{l});
    vd.A5{l} = A5{l};
  end
  toc
end
