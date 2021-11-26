function [vd] = GetDirections(vd, rbm)
  % Inertia matrix 
  G = rbm.Dynamics.H_matrix;
  
  %{
    Need inv(InertiaMatrix) to form dual vector fields:
    ~ A frequent misuse of inv arises when solving the system of linear equations Ax = b. 
    ~ One way to solve the equation is with x = inv(A)*b. A better way, from the standpoint
    ~ of both execution time and numerical accuracy, is to use the matrix backslash operator x = A\b. 
    ~ This produces the solution using Gaussian elimination, without explicitly forming the inverse. 
  %}
  
  fprintf('\nGet controlled directions (Gram-Schmidt):\n')
  tic
  for i = 1:numel(vd.F)
    fprintf(['  - Y', num2str(i), '\n'])
    validateattributes(vd.F{i}, {'double'}, {'size', [numel(rbm.States.q.sym),1]})
    if i ==1
      X{i} = G\vd.F{i};
    else
      gramschmidt_sum = 0;
      for k = 1:i-1
        gramschmidt_sum = gramschmidt_sum + (((G\vd.F{i})'*G*X{k})/(X{k}'*G*X{k}))*X{k};
      end
      X{i} = G\vd.F{i} - gramschmidt_sum;
    end
    Y{i} = X{i}/sqrt(X{i}'*G*X{i});
    vd.Y{i} = Y{i};
  end
  toc
  
  fprintf('\nGet uncontrolled directions (Gram-Schmidt):\n')
  tic
  for i = 1:numel(vd.Fs)
    fprintf(['  - Yperp', num2str(i), '\n'])
    validateattributes(vd.Fs{i}, {'double'}, {'size', [numel(rbm.States.q.sym),1]})
    if i ==1
      Xp{i} = vd.Fs{i};
    else    
      for k = 1:i-1
        if k == 1
          gramschmidt_sum = ((vd.Fs{i}'*G*Xp{k})/(Xp{k}'*G*Xp{k}))*Xp{k};
        else
          gramschmidt_sum = gramschmidt_sum + ((vd.Fs{i}'*G*Xp{k})/(Xp{k}'*G*Xp{k}))*Xp{k};
        end
      end
      Xp{i} = vd.Fs{i} - gramschmidt_sum;
    end
    Yp{i} = Xp{i}/sqrt(Xp{i}'*G*Xp{i});
    vd.Yperp{i} = Yp{i};
  end
  toc
end
