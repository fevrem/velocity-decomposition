function  robot = spatial_5_link_minimal()  
  robot.nd = 8;
  robot.parent = 0:7;
  robot.name = 'Spatial 5-Link Biped';
  
  % gravity vector
  robot.gravity = [0 0 -9.81]';
  
  % using Denavit Hartenberg convention, R joints are in +Z-direction    
  robot.jtype{1} = 'Rz'; % q1
  robot.jtype{2} = 'Rz'; % q2
  robot.jtype{3} = 'Rz'; % q3
  robot.jtype{4} = 'Rz'; % q4
  robot.jtype{5} = 'Rz'; % q5
  robot.jtype{6} = 'Rz'; % q6
  robot.jtype{7} = 'Rz'; % q7
  robot.jtype{8} = 'Rz'; % q8
  
  model_prms = Model.spatial_5_link_params();
  
  l{1} = 0;
  l{2} = model_prms.L_lower_leg;
  l{3} = model_prms.L_upper_leg;
  l{4} = 0;
  l{5} = model_prms.L_pelvis;
  l{6} = 0;
  l{7} = model_prms.L_upper_leg;
  l{8} = model_prms.L_lower_leg;
  robot.l = l;
  
  m{1} = 0;
  m{2} = model_prms.M_lower_leg;
  m{3} = model_prms.M_upper_leg;
  m{4} = 0;
  m{5} = model_prms.M_pelvis;
  m{6} = 0;
  m{7} = model_prms.M_upper_leg;
  m{8} = model_prms.M_lower_leg;
  robot.m = m;
  robot.m_torso = model_prms.M_torso;
  robot.l_torso = model_prms.L_torso;
  robot.total_mass = sum([robot.m{:}]) + model_prms.M_torso;
  
  % {0}-frame to {1}-frame
  d_0 = 0;
  %     theta_0 = pi/2;
  theta_0 = -pi/2;
  
  % {4}-frame to {5}-frame
  alpha{1} = -pi/2;
  a{1}     = 0;
  d{1}     = 0;
  theta{1} = 0;
  
  % {5}-frame to {6}-frame
  d{2}     = 0;
  alpha{2} = pi/2;
  a{2}     = l{1};
  theta{2} = 0;
  
  % {6}-frame to {7}-frame
  d{3}     = 0;
  a{3}     = l{2};
  alpha{3} = 0;
  theta{3} = 0;
  
  % {7}-frame to {8}-frame
  d{4}     = 0;
  a{4}     = l{3};
  alpha{4} = 0;
  theta{4} = 0;
  
  % {8}-frame to {9}-frame
  d{5}     = 0;
  alpha{5} = -pi/2;
  a{5}     = l{4};
  theta{5} = 0;
  
  % {9}-frame to {10}-frame
  d{6}     = 0;
  alpha{6} = pi;
  a{6}     = l{5};
  theta{6} = 0;
  
  % {10}-frame to {11}-frame
  d{7}     = 0;
  alpha{7} = pi/2;
  a{7}     = l{6};
  theta{7} = 0;
  
  % {11}-frame to {12}-frame
  d{8}     = 0;
  alpha{8} = 0;
  a{8}     = l{7};
  theta{8} = 0;
  
  for i = 1:robot.nd
    if i == 1 
      robot.Xtree{i} = xlt([a{i} 0 d{i}]')*round(rotx(alpha{i}))*round(rotz(theta_0))*xlt([0 0 d_0]');
    else
      robot.Xtree{i} = xlt([a{i} 0 d{i}]')*round(rotx(alpha{i}))*round(rotz(theta{i-1}))*xlt([0 0 d{i-1}]');
    end
  end
  
  % -------------------- Spatial inertias ---------------------- %    
  for i = 1:robot.nd
    robot.I{i} = mcI( m{i},...
    [l{i}/2 0 0],...
    diag([0,1/12*m{i}*l{i}^2,1/12*m{i}*l{i}^2]) );
  end  
  
  % ------------------ for plotting purposes ------------------- %
  robot.appearance.base = ... 
  { 'tiles',[-10 10;-10 10; 0 0],.5};            
  
  rsphere = 0.02;
  rlink = 0.017;
  for i = 1:robot.nd
    if i == robot.nd
      robot.appearance.body{i} = ...
      {'colour',[0 128 255]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[l{i} 0 0], rsphere,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere };          
    elseif i == 2
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
      {'colour',[204 0 0]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere };            
    elseif i == 4 || i == 5
      robot.appearance.body{i} = ...
      {'colour',[1 150 1]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere };
    else
      robot.appearance.body{i} = ...
      {'colour',[0 128 255]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere };
    end
  end
  
  % ------------ Add torso to kinematic chain --------------- %
  %  Add torso inertia to pelvis inertia
  %  Define imaginary frame {T} at base of torso
  %  Assume torso link is aligned with +X-axis of {T}-frame
  %  Finally, add appearance of torso to pelvis link
  %  variables:
  %       I_torso: spatial inertia of torso in {T}-frame
  %       r_9_to_T: position of base of torso along pelvis link
  %       R_9_to_T: torso rigidly attached to pelvis pointing up
  %       X_9_to_T: spatial transform from {9}-frame to {T}-frame
  %       X_T_to_9_star: spatial force transform from {T} to {9}
  %       I_torso_in_9: express torso spatial inertia in {T}-frame
  I_torso = mcI( model_prms.M_torso,...
  [model_prms.L_torso/2 0 0],...
  diag([ 0 , 1/12*model_prms.M_torso*model_prms.L_torso^2 , 1/12*model_prms.M_torso*model_prms.L_torso^2 ]) );
  [p_9_to_T , R_9_to_T] = get_torso_frame_minimal(robot);
  X_9_to_T = [ R_9_to_T , zeros(3,3);
  -R_9_to_T*crm(p_9_to_T) , R_9_to_T ];
  X_T_to_9_star = [ R_9_to_T' , crm(p_9_to_T)*R_9_to_T' ;
  zeros(3,3) , R_9_to_T' ];
  I_torso_in_9 = X_T_to_9_star * I_torso * X_9_to_T;
  robot.I{5} = robot.I{5} + I_torso_in_9;
  robot.appearance.body{5} = [ robot.appearance.body{5}(1:5),...
  'colour',[1 150 1]/255,...
  'cyl',[l{5}/2 0 0 ; l{5}/2 -model_prms.L_torso 0], rlink+0.1*rlink,...
  robot.appearance.body{5}(6:10) ];
  
  robot.q = Var('q', robot.nd);
  robot.qd = Var('dq', robot.nd);
  robot.qdd = Var('ddq', robot.nd);
  
  % input torques
  robot.u = Var('u', 6);
  
  % actuation matrix (n x m)
  robot.B = [ zeros(2,6) ; eye(6,6) ];
  
  % selects the actuated DoF
  robot.H0 = robot.B';
  
  % define theta (increasing phase variable)
  robot.c = [0 -1 -1/2 0 0 0 0 0];
end


function [p_9_to_T , R_9_to_T] = get_torso_frame_minimal(model)
  p_9_to_T = [model.l{5}/2;0;0];
  R_9_to_T = round(rotz(-pi/2));
  R_9_to_T = R_9_to_T(1:3,1:3);
end
