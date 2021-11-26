function vd_data = numeval_decomposition(rbm, vd, t, q, qd)
  % Evaluate the decomposition numerically 
  
  arguments
    rbm (1,1) DynamicalSystem
    vd (1,1) VelocityDecomposition
    t (1,:) double
    q (:,:) double
    qd (:,:) double
  end
  validateattributes(t, {'double'}, {'increasing'})
  validateattributes(q, {'double'}, {'size', [rbm.Model.nd, numel(t)]})
  validateattributes(qd, {'double'}, {'size', [rbm.Model.nd, numel(t)]})

  % preallocate arrays
  w = zeros(numel(vd.F), numel(t));
  s = zeros(numel(vd.Fs), numel(t));
  Y = mat2cell(repmat(zeros(rbm.Model.nd, numel(t)), 1, numel(vd.F)), rbm.Model.nd, repmat(numel(t), 1, numel(vd.F)));
  Yperp = mat2cell(repmat(zeros(rbm.Model.nd, numel(t)), 1, numel(vd.Fs)), rbm.Model.nd, repmat(numel(t), 1, numel(vd.Fs)));
  ds = zeros(numel(vd.Fs), numel(t));
  b1term = zeros(numel(vd.Fs), numel(t));
  b2term = zeros(numel(vd.Fs), numel(t));
  b3term = zeros(numel(vd.Fs), numel(t));
  b4term = zeros(numel(vd.Fs), numel(t));
  b5term = zeros(numel(vd.Fs), numel(t));
  b1coeff = mat2cell(repmat(zeros(numel(vd.F)^2, numel(t)), 1, numel(vd.Fs)), numel(vd.F)^2, repmat(numel(t), 1, numel(vd.Fs)));
  b2coeff = mat2cell(repmat(zeros(numel(vd.F)*numel(vd.Fs), numel(t)), 1, numel(vd.Fs)), numel(vd.F)*numel(vd.Fs), repmat(numel(t), 1, numel(vd.Fs)));
  b3coeff = mat2cell(repmat(zeros(numel(vd.F)*numel(vd.Fs), numel(t)), 1, numel(vd.Fs)), numel(vd.F)*numel(vd.Fs), repmat(numel(t), 1, numel(vd.Fs)));
  b4coeff = mat2cell(repmat(zeros(numel(vd.Fs)^2, numel(t)), 1, numel(vd.Fs)), numel(vd.Fs)^2, repmat(numel(t), 1, numel(vd.Fs)));
  b5coeff = mat2cell(repmat(zeros(1, numel(t)), 1, numel(vd.Fs)), 1, repmat(numel(t), 1, numel(vd.Fs)));

  % evaluate directions & velocities
  fprintf('\nEvaluate directions and velocities numerically:\n')
  tic
  for k = 1:numel(t)
    for i = 1:numel(vd.F)  
      w(i,k) = vd.w_cfun{i}('f', q(:,k), qd(:,k));
      Y{i}(:,k) = vd.Y_cfun{i}('f', q(:,k));        
    end
    for i = 1:numel(vd.Fs)
      s(i,k) = vd.s_cfun{i}('f', q(:,k), qd(:,k));
      Yperp{i}(:,k) = vd.Yperp_cfun{i}('f', q(:,k));
    end
  end
  toc

  % check G-orthonormality conditions
  fprintf('\nCheck G-orthonormality conditions:\n')
  msgErr = 'Error.\nInvalid decomposition.';
  tic
  for k = 1:numel(t)
    % evaluate inertia matrix numerically
    G = H_matrix('f', q(:,k));
    for i = 1:numel(vd.F)
      for j = 1:numel(vd.F)
        ortho_eval = round(Y{i}(:,k)'*G*Y{j}(:,k), 6);
        if j == i
          if ~(ortho_eval == 1), error([msgErr, '\n<Y%d,Y%d>_M ~= 1'], i, j), end
        else
          if ~(ortho_eval == 0), error([msgErr, '\n<Y%d,Y%d>_M ~= 0'], i, j), end
        end
      end
    end
 
    for i = 1:numel(vd.Fs)
      for j = 1:numel(vd.Fs)
        ortho_eval = round(Yperp{i}(:,k)'*G*Yperp{j}(:,k), 6);
        if j == i
          if ~(ortho_eval == 1), error([msgErr, '\n<Yperp%d,Yperp%d>_M ~= 1'], i, j), end
        else
          if ~(ortho_eval == 0), error([msgErr, '\n<Yperp%d,Yperp%d>_M ~= 0'], i, j), end
        end
        
      end
    end
    
    for i = 1:numel(vd.F)
      for j = 1:numel(vd.Fs)        
        ortho_eval = round(Y{i}(:,k)'*G*Yperp{j}(:,k), 6);
        if ~(ortho_eval == 0), error([msgErr, '\n<Y%d,Yperp%d>_M ~= 0'], i, j), end
      end
    end
  end
  toc

  fprintf('\nEvaluate coupling coefficients numerically:\n')
  tic
  for l = 1:numel(vd.Fs)
    for k = 1:numel(t)
      % B1 contribution
      B1term = 0;
      B1 = vd.B1_cfun{l}('f', q(:,k));
      for i = 1:numel(vd.F)
        for j = 1:numel(vd.F)
          B1term = B1term + B1(i,j)*w(i,k)*w(j,k);
        end
      end
      
      % B2 contribution
      B2term = 0;
      B2 = vd.B2_cfun{l}('f', q(:,k));
      for i = 1:numel(vd.F)
        for j = 1:numel(vd.Fs)
          B2term = B2term + B2(i,j)*w(i,k)*s(j,k);
        end
      end
      
      % B3 contribution
      B3term = 0;
      B3 = vd.B3_cfun{l}('f', q(:,k));
      for i = 1:numel(vd.F)
        for j = 1:numel(vd.Fs)
          B3term = B3term + B3(j,i)*w(i,k)*s(j,k);
        end
      end
      
      % B4 contribution
      B4term = 0;
      B4 = vd.B4_cfun{l}('f', q(:,k));
      for i = 1:numel(vd.Fs)
        for j = 1:numel(vd.Fs)
          B4term = B4term + B4(i,j)*s(i,k)*s(j,k);
        end
      end
      
      % gravity contribution to ds/dt
      B5term = vd.B5_cfun{l}('f', q(:,k));
      
      % ds/dt is negative sum of all terms
      ds(l,k) = -B1term -B2term -B3term -B4term -B5term;
      
      % store coupling coefficients
      b1coeff{l}(:,k) = reshape(B1, numel(vd.F)^2, 1);
      b2coeff{l}(:,k) = reshape(B2, numel(vd.F)*numel(vd.Fs), 1);
      b3coeff{l}(:,k) = reshape(B3, numel(vd.F)*numel(vd.Fs), 1);
      b4coeff{l}(:,k) = reshape(B4, numel(vd.Fs)^2, 1);
      b5coeff{l}(1,k) = B5term;
      
      % store coupling terms
      b1term(l,k) = B1term;
      b2term(l,k) = B2term;
      b3term(l,k) = B3term;
      b4term(l,k) = B4term;
      b5term(l,k) = B5term;
    end
  end
  toc

  % pack output data
  vd_data.t = t;
  vd_data.q = q;
  vd_data.qd = qd;
  vd_data.w = w;
  vd_data.s = s;
  vd_data.Y = Y;
  vd_data.Yperp = Yperp;
  vd_data.b1term = b1term;
  vd_data.b2term = b2term;
  vd_data.b3term = b3term;
  vd_data.b4term = b4term;
  vd_data.b5term = b5term;
  vd_data.b1coeff = b1coeff;
  vd_data.b2coeff = b2coeff;
  vd_data.b3coeff = b3coeff;
  vd_data.b4coeff = b4coeff;
  vd_data.b5coeff = b5coeff;
  vd_data.ds = ds;
end