function  robot = planar_5_link_minimal()  
  robot.nd = 5;
  robot.parent = 0:4;
  robot.name = 'Planar 5-Link Biped';
  
  % gravity vector
  robot.gravity = [0 0 -9.81]';
  
  % using Denavit Hartenberg convention, R joints are in +Z-direction    
  robot.jtype{1} = 'Rz'; % q1
  robot.jtype{2} = 'Rz'; % q2
  robot.jtype{3} = 'Rz'; % q3
  robot.jtype{4} = 'Rz'; % q4
  robot.jtype{5} = 'Rz'; % q5
  
  % ----------- Geometric and inertial parameters ------------- %
  model_prms = Model.planar_5_link_params();
  
  l{1} = model_prms.L_lower_leg;
  l{2} = model_prms.L_upper_leg;
  l{3} = model_prms.L_torso;
  l{4} = model_prms.L_upper_leg;
  l{5} = model_prms.L_lower_leg;
  robot.l = l;
  
  m{1} = model_prms.M_lower_leg;
  m{2} = model_prms.M_upper_leg;
  m{3} = model_prms.M_torso;
  m{4} = model_prms.M_upper_leg;
  m{5} = model_prms.M_lower_leg;    
  robot.m = m;
  robot.m_torso = model_prms.M_torso;
  robot.l_torso = model_prms.L_torso;
  robot.total_mass = sum([robot.m{:}]);
  
  % Denavit Hartenberg parameters
  % {0}-frame to {1}-frame
  d_0 = 0;
  theta_0 = 0;
  
  % {2}-frame to {3}-frame
  alpha{1} = -pi/2;
  a{1}     = 0;
  d{1}     = 0;
  theta{1} = 0;
  
  % {3}-frame to {4}-frame
  d{2}     = 0;
  alpha{2} = 0;
  a{2}     = robot.l{1};
  theta{2} = 0;
  
  % {4}-frame to {5}-frame
  d{3}     = 0;
  alpha{3} = 0;
  a{3}     = robot.l{2};
  theta{3} = 0;
  
  % {5}-frame to {6}-frame
  d{4} =  0;
  alpha{4} = 0; 
  a{4} = 0;
  theta{4} = 0;
  
  % {6}-frame to {7}-frame
  d{5} =  0;
  alpha{5} = 0; 
  a{5} = robot.l{4};
  theta{5} = 0;
  
  for i = 1:robot.nd
    if i == 1 
      robot.Xtree{i} = xlt([a{i} 0 d{i}]')*round(rotx(alpha{i}))*round(rotz(theta_0))*xlt([0 0 d_0]');
    else
      robot.Xtree{i} = xlt([a{i} 0 d{i}]')*round(rotx(alpha{i}))*round(rotz(theta{i-1}))*xlt([0 0 d{i-1}]');
    end
  end
  
  % Spatial inertias
  for i = 1:robot.nd
    robot.I{i} = mcI( m{i},...
    [l{i}/2 0 0],...
    diag([0,1/12*m{i}*l{i}^2,1/12*m{i}*l{i}^2]) );
  end  
  
  robot.appearance.base = ... 
  { 'tiles',[-10 10;-10 10; 0 0],.5};            
  
  rsphere = 0.02;
  rlink = 0.017;
  for i = 1:robot.nd
    if i == 2
      robot.appearance.body{i} = ...
      {'colour',[204 0 0]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere};
    elseif i == 1
      robot.appearance.body{i} = ...
      {'colour',[204 0 0]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere };
    elseif i == 3
      robot.appearance.body{i} = ...
      {'colour',[1 150 1]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere };            
    elseif i == 4 || i == 5
      robot.appearance.body{i} = ...
      {'colour',[0 128 255]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere };
    end
  end
  
  robot.q = Var('q', robot.nd);
  robot.qd = Var('dq', robot.nd);
  robot.qdd = Var('ddq', robot.nd);
  
  % input torques
  robot.u = Var('u', 4);
  
  % actuation matrix (n x m)
  robot.B = [ zeros(1,4) ; eye(4,4) ];
  
  % selects the actuated DoF
  robot.H0 = robot.B';
  
  % define theta (increasing phase variable)
  robot.c = [1 1/2 0 0 0];
end
