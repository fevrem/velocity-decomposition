function plot_decomposition(rbm, vd, data)  
  arguments
    rbm (1,1) DynamicalSystem
    vd (1,1) VelocityDecomposition
    data (1,1) struct
  end

  % Check for missing fields
  if isfield(data, 't')
    t = data.t;
  else
    error('Field ''t'' is missing in struct ''data''.'); 
  end
  validateattributes(t, {'double'}, {'increasing'})

  if isfield(data, 'qd')
    qd = data.qd;
  else
    error('Field ''qd'' is missing in struct ''data''.'); 
  end
  validateattributes(qd, {'double'}, {'size', [rbm.Model.nd, numel(t)]})

  if isfield(data, 'q')
    q = data.q;
  else
    error('Field ''q'' is missing in struct ''data''.'); 
  end
  validateattributes(q, {'double'}, {'size', [rbm.Model.nd, numel(t)]})

  if isfield(data, 'w')
    w = data.w;
  else
    error('Field ''w'' is missing in struct ''data''.'); 
  end    
    validateattributes(w, {'double'}, {'size', [numel(vd.F), numel(t)]})

  if isfield(data, 's')
    s = data.s;
  else
    error('Field ''s'' is missing in struct ''data''.'); 
  end    
  validateattributes(s, {'double'}, {'size', [numel(vd.Fs), numel(t)]})

  if isfield(data, 'Y')
    Y = data.Y;
  else
    error('Field ''Y'' is missing in struct ''data''.'); 
  end    
  validateattributes(Y, {'cell'}, {'size', [1, numel(vd.F)]})
  for i = 1:numel(vd.F)
    validateattributes(Y{i}, {'double'}, {'size', [rbm.Model.nd, numel(t)]})
  end

  if isfield(data, 'Yperp')
    Yperp = data.Yperp;
  else
    error('Field ''Yperp'' is missing in struct ''data''.'); 
  end    
  validateattributes(Yperp, {'cell'}, {'size', [1, numel(vd.Fs)]})
  for i = 1:numel(vd.Fs)
    validateattributes(Yperp{i}, {'double'}, {'size', [rbm.Model.nd, numel(t)]})
  end

  if isfield(data, 'b1coeff')
    b1coeff = data.b1coeff;
  else
    error('Field ''b1coeff'' is missing in struct ''data''.'); 
  end   
  validateattributes(b1coeff, {'cell'}, {'size', [1, numel(vd.Fs)]})
  for i = 1:numel(vd.Fs)
    validateattributes(b1coeff{i}, {'double'}, {'size', [numel(vd.F)^2, numel(t)]})
  end

  if isfield(data, 'b2coeff')
    b2coeff = data.b2coeff;
  else
    error('Field ''b2coeff'' is missing in struct ''data''.'); 
  end   
  validateattributes(b2coeff, {'cell'}, {'size', [1, numel(vd.Fs)]})
  for i = 1:numel(vd.Fs)
    validateattributes(b2coeff{i}, {'double'}, {'size', [numel(vd.F)*numel(vd.Fs), numel(t)]})
  end

  if isfield(data, 'b3coeff')
    b3coeff = data.b3coeff;
  else
    error('Field ''b3coeff'' is missing in struct ''data''.'); 
  end   
  validateattributes(b3coeff, {'cell'}, {'size', [1, numel(vd.Fs)]})
  for i = 1:numel(vd.Fs)
    validateattributes(b3coeff{i}, {'double'}, {'size', [numel(vd.F)*numel(vd.Fs), numel(t)]})
  end

  if isfield(data, 'b4coeff')
    b4coeff = data.b4coeff;
  else
    error('Field ''b4coeff'' is missing in struct ''data''.'); 
  end   
  validateattributes(b4coeff, {'cell'}, {'size', [1, numel(vd.Fs)]})
  for i = 1:numel(vd.Fs)
    validateattributes(b4coeff{i}, {'double'}, {'size', [numel(vd.Fs)^2, numel(t)]})
  end

  if isfield(data, 'b5coeff')
    b5coeff = data.b5coeff;
  else
    error('Field ''b5coeff'' is missing in struct ''data''.'); 
  end   
  validateattributes(b5coeff, {'cell'}, {'size', [1, numel(vd.Fs)]})
  for i = 1:numel(vd.Fs)
    validateattributes(b5coeff{i}, {'double'}, {'size', [1, numel(t)]})
  end

  if isfield(data, 'b1term')
    b1term = data.b1term;
  else
    error('Field ''b1term'' is missing in struct ''data''.'); 
  end   
  validateattributes(b1term, {'double'}, {'size', [numel(vd.Fs), numel(t)]})

  if isfield(data, 'b2term')
    b2term = data.b2term;
  else
    error('Field ''b2term'' is missing in struct ''data''.'); 
  end   
  validateattributes(b2term, {'double'}, {'size', [numel(vd.Fs), numel(t)]})

  if isfield(data, 'b3term')
    b3term = data.b3term;
  else
    error('Field ''b3term'' is missing in struct ''data''.'); 
  end   
  validateattributes(b3term, {'double'}, {'size', [numel(vd.Fs), numel(t)]})

  if isfield(data, 'b4term')
    b4term = data.b4term;
  else
    error('Field ''b4term'' is missing in struct ''data''.'); 
  end   
  validateattributes(b4term, {'double'}, {'size', [numel(vd.Fs), numel(t)]})

  if isfield(data, 'b5term')
    b5term = data.b5term;
  else
    error('Field ''b5term'' is missing in struct ''data''.'); 
  end   
  validateattributes(b5term, {'double'}, {'size', [numel(vd.Fs), numel(t)]})

  if isfield(data, 'ds')
    ds = data.ds;
  else
    error('Field ''ds'' is missing in struct ''data''.'); 
  end   
  validateattributes(ds, {'double'}, {'size', [numel(vd.Fs), numel(t)]})

  [interpreter,linewidth,fontname,fontsize] = Anim.plt_settings();
  fontsettings.interpreter = interpreter;
  fontsettings.linewidth = linewidth;
  fontsettings.fontname = fontname;
  fontsettings.fontsize = fontsize;

  % Velocities
  fig = figure; 
  fig.NumberTitle = 'off';
  fig.MenuBar = 'none';
  fig.Name = 'Decomposed Velocities';

  subplot(2,1,1); hold on; grid on;
  p = cell(1,numel(vd.F));
  plab = cell(1,numel(vd.F));
  for i = 1:numel(vd.F)
    p{i} = plot(t, w(i,:), 'LineWidth', 2);
    plab{i} = ['$w_', num2str(i), '$'];
  end
  hleg = legend([p{:}], plab{:}); clear p plab
  if i >= 2
    title('Controlled Velocities')
  else
    title('Controlled Velocity')
  end
  xl = xlabel('time (s)');
  yl = ylabel(['$', decomposedVelocityUnit(), '$']);
  set([hleg, xl, yl],...
  'Interpreter', fontsettings.interpreter,...
  'FontSize', fontsettings.fontsize,...
  'FontName', fontsettings.fontname)

  subplot(2,1,2); hold on; grid on;
  p = cell(1,numel(vd.Fs));
  plab = cell(1,numel(vd.Fs));
  for i = 1:numel(vd.Fs)
    p{i} = plot(t, s(i,:), 'LineWidth', 2);
    plab{i} = ['$s_', num2str(i), '$'];
  end
  hleg = legend([p{:}], plab{:}); clear p plab
  if i >= 2
    title('Uncontrolled Velocities')
  else
    title('Uncontrolled Velocity')
  end
  xl = xlabel('time (s)');
  yl = ylabel(['$', decomposedVelocityUnit(), '$']);
  set([hleg, xl, yl],...
  'Interpreter', fontsettings.interpreter,...
  'FontSize', fontsettings.fontsize,...
  'FontName', fontsettings.fontname)

  % Directions 
  fig = figure; 
  fig.NumberTitle = 'off';
  fig.MenuBar = 'none';
  fig.Name = 'Directions';
  [np, ~] = Anim.numSubplots(rbm.Model.nd);
  for k = 1:rbm.Model.nd
    subplot(np(1), np(2), k); hold on; grid on;
    if k <= numel(vd.F)
      p = cell(1,rbm.Model.nd);
      plab = cell(1,rbm.Model.nd);
      for n = 1:rbm.Model.nd
        p{n} = plot(t, Y{k}(n,:), 'LineWidth', 2);
        plab{n} = ['$Y_{', num2str(k), '}(' num2str(n), ')$'];
      end
      hleg = legend([p{:}], plab{:}); clear p plab
    else
      p = cell(1,rbm.Model.nd);
      plab = cell(1,rbm.Model.nd);
      for n = 1:rbm.Model.nd
        p{n} = plot(t, Yperp{k-numel(vd.F)}(n,:), 'LineWidth', 2);
        plab{n} = ['$Y^{\perp}_{', num2str(k-numel(vd.F)), '}(' num2str(n), ')$'];
      end
      hleg = legend([p{:}], plab{:}); clear p plab
    end
    xl = xlabel('time (s)');
    yl = ylabel(['$', directionsUnit(), '$']);
    set([hleg, xl, yl],...
    'Interpreter', fontsettings.interpreter,...
    'FontSize', fontsettings.fontsize,...
    'FontName', fontsettings.fontname)
  end

  % Reconstruct velocities
  fig = figure; 
  fig.NumberTitle = 'off';
  fig.MenuBar = 'none';
  fig.Name = 'Doted velocities';
  for k = 1:rbm.Model.nd
    subplot(np(1), np(2), k); hold on; grid on;
    if k <= numel(vd.F)
      p = cell(1,rbm.Model.nd);
      plab = cell(1,rbm.Model.nd);
      for n = 1:rbm.Model.nd
        p{n} = plot(t, w(k,:).*Y{k}(n,:), 'LineWidth', 2);
        plab{n} = ['$w_{', num2str(k), '}\cdot Y_{', num2str(k), '}(' num2str(n), ')$'];
      end
      hleg = legend([p{:}], plab{:}); clear p plab
    else
      p = cell(1,rbm.Model.nd);
      plab = cell(1,rbm.Model.nd);
      for n = 1:rbm.Model.nd
        p{n} = plot(t, s(k-numel(vd.F),:).*Yperp{k-numel(vd.F)}(n,:), 'LineWidth', 2);
        plab{n} = ['$s_{', num2str(k-numel(vd.F)), '}\cdot Y^{\perp}_{', num2str(k-numel(vd.F)), '}(' num2str(n), ')$'];
      end
      hleg = legend([p{:}], plab{:}); clear p plab
    end
    xl = xlabel('time (s)');
    yl = ylabel('rad/s');
    set([hleg, xl, yl],...
    'Interpreter', fontsettings.interpreter,...
    'FontSize', fontsettings.fontsize,...
    'FontName', fontsettings.fontname)
  end

  fig = figure; 
  fig.NumberTitle = 'off';
  fig.MenuBar = 'none';
  fig.Name = 'Reconstructed velocities';
  for k = 1:rbm.Model.nd
    subplot(np(1), np(2), k); hold on; grid on;
    p = cell(1,rbm.Model.nd);
    plab = cell(1,rbm.Model.nd);
    for n = 1:rbm.Model.nd
      if n <= numel(vd.F)
        % reconstruct each velocity
        if n == 1 % there is at least one controlled direction
          vRec = w(n,:).*Y{n}(k,:);
        else
          vRec = vRec + w(n,:).*Y{n}(k,:);
        end
        p{n} = plot(t, w(n,:).*Y{n}(k,:), 'LineWidth', 2);
        plab{n} = ['$w_{', num2str(n), '}\cdot Y_{', num2str(n), '}(' num2str(k), ')$'];
      else
        vRec = vRec + s(n-numel(vd.F),:).*Yperp{n-numel(vd.F)}(k,:);
        p{n} = plot(t, s(n-numel(vd.F),:).*Yperp{n-numel(vd.F)}(k,:), 'LineWidth', 2);
        plab{n} = ['$s_{', num2str(n-numel(vd.F)), '} \cdot Y^{\perp}_{', num2str(n-numel(vd.F)), '}(' num2str(k), ')$'];
      end
    end
    p{n+1} = plot(t, vRec, 'r', 'LineWidth', 2);
    plab{n+1} = ['$\dot{q}_{', num2str(k), '}$ (rec)'];
    p{n+2} = plot(t, qd(k,:), 'k:', 'LineWidth', 2);
    plab{n+2} = ['$\dot{q}_{', num2str(k), '}$'];
    xl = xlabel('time (s)');
    yl = ylabel('rad/s');
    hleg = legend([p{:}], plab{:}); clear p plab   
    set([hleg, xl, yl],...
    'Interpreter', fontsettings.interpreter,...
    'FontSize', fontsettings.fontsize,...
    'FontName', fontsettings.fontname)
  end

  % DS/DT
  fig = figure; 
  fig.NumberTitle = 'off';
  fig.MenuBar = 'none';
  fig.Name = 'ds/dt';
  [np, ~] = Anim.numSubplots(numel(vd.Fs));
  for k = 1:numel(vd.Fs)
    subplot(np(1), np(2), k); hold on; grid on;
    p{1} = plot(t, ds(k,:), 'r', 'LineWidth', 2);
    plab{1} = ['$\frac{ds_{', num2str(k), '}}{dt}$'];
    p{2} = plot(t, gradient(s(k,:),t), 'k:', 'LineWidth', 2);
    plab{2} = ['$\frac{ds_{', num2str(k), '}}{dt}$ (dirty)'];
    hleg = legend([p{:}], plab{:}); clear p plab
    xl = xlabel('time (s)');
    yl = ylabel('$\sqrt{kg} \cdot \frac{m}{s^2}$');
    set([hleg, xl, yl],...
    'Interpreter', fontsettings.interpreter,...
    'FontSize', fontsettings.fontsize,...
    'FontName', fontsettings.fontname)
  end

  % Coefficients
  for k = 1:numel(vd.Fs)
    fig = figure; 
    fig.NumberTitle = 'off';
    fig.MenuBar = 'none';
    fig.Name = ['Coupling Coefficients (', num2str(k), ')'];
    for i = 1:4
      switch i
        case 1
          bcoeff2plot = b1coeff{k};
        case 2
          bcoeff2plot = b2coeff{k};
        case 3
          bcoeff2plot = b3coeff{k};
        case 4
          bcoeff2plot = b4coeff{k};
      end
      subplot(2, 2, i); hold on; grid on;
      for j = 1:size(bcoeff2plot, 1)
        p{j} = plot(t, bcoeff2plot(j,:), 'LineWidth', 2);
        plab{j} = ['$B^{', num2str(k), '}_{', num2str(i), '}(', num2str(j), ')$'];
      end
      hleg = legend([p{:}], plab{:}); clear p plab
      xl = xlabel('time (s)');
      yl = ylabel('$\frac{1}{m \cdot \sqrt{kg}}$');
      set([hleg, xl, yl],...
      'Interpreter', fontsettings.interpreter,...
      'FontSize', fontsettings.fontsize,...
      'FontName', fontsettings.fontname)
    end
  end
  
  % Terns
  for k = 1:numel(vd.Fs)
    fig = figure; 
    fig.NumberTitle = 'off';
    fig.MenuBar = 'none';
    fig.Name = ['Coupling Terms (', num2str(k), ')'];
    for i = 1:5
      switch i
        case 1
          bterm2plot = b1term(k,:);
        case 2
          bterm2plot = b2term(k,:);
        case 3
          bterm2plot = b3term(k,:);
        case 4
          bterm2plot = b4term(k,:);
        case 5
          bterm2plot = b5term(k,:);
      end
      subplot(3, 2, i); hold on; grid on;
      p = plot(t, bterm2plot, 'LineWidth', 2);
      plab = ['$B^{', num2str(k), '}_{', num2str(i), '}$ term'];
      xl = xlabel('time (s)');
      yl = ylabel('$\sqrt{kg} \cdot \frac{m}{s^2}$');
      hleg = legend(p, plab); clear p plab
      set([hleg, xl, yl],...
      'Interpreter', fontsettings.interpreter,...
      'FontSize', fontsettings.fontsize,...
      'FontName', fontsettings.fontname)
    end
  end

  % Coupling & Gravity
  fig = figure; 
  fig.NumberTitle = 'off';
  fig.MenuBar = 'none';
  fig.Name = 'DS/DT = Coupling + Gravity';
  [np, ~] = Anim.numSubplots(numel(vd.Fs));
  for k = 1:numel(vd.Fs)    
    subplot(np(1), np(2), k); hold on; grid on;
    tit = title(['$\frac{ds_{', num2str(k), '}}{dt}$']);      
    p{1} = plot(t, -b1term(k,:)-b2term(k,:)-b3term(k,:), 'LineWidth', 2);
    plab{1} = 'Coupling';
    p{2} = plot(t, -b5term(k,:), 'LineWidth', 2);
    plab{2} = 'Gravity';   
    hleg = legend([p{:}], plab{:}); clear p plab
    xl = xlabel('time (s)');
    yl = ylabel('$\sqrt{kg} \cdot \frac{m}{s^2}$');
    set([hleg, xl, yl, tit],...
    'Interpreter', fontsettings.interpreter,...
    'FontSize', fontsettings.fontsize,...
    'FontName', fontsettings.fontname)
  end    
end

  
function strg = directionsUnit()
  strg = '\sqrt{kg} \cdot m';
end


function strg = decomposedVelocityUnit()  
  strg = '\sqrt{kg} \cdot \frac{m}{s}';
end
