function  robot = planar_5_link()
  robot.nd = 12;
  robot.parent = 0:11;
  robot.name = 'Spatial 5-Link Biped';
 
  % gravity vector
  robot.gravity = [0 0 -9.81]';
  
  % using Denavit Hartenberg convention, R joints are in +Z-direction
  robot.jtype{1}  = 'Px'; % x-pos of stance foot
  robot.jtype{2}  = 'Py'; % y-pos of stance foot
  robot.jtype{3}  = 'Pz'; % z-pos of stance foot
  robot.jtype{4}  = 'Rz'; % yaw-rot of stance foot
  robot.jtype{5}  = 'Rz'; % q1
  robot.jtype{6}  = 'Rz'; % q2
  robot.jtype{7}  = 'Rz'; % q3
  robot.jtype{8}  = 'Rz'; % q4
  robot.jtype{9}  = 'Rz'; % q5
  robot.jtype{10} = 'Rz'; % q6
  robot.jtype{11} = 'Rz'; % q7
  robot.jtype{12} = 'Rz'; % q8
  
  % Geometric and inertial parameters 
  model_prms = Model.spatial_5_link_params();

  l{1} = 0;
  l{2} = 0;
  l{3} = 0;
  l{4} = 0;
  l{5} = 0;
  l{6} = model_prms.L_lower_leg;
  l{7} = model_prms.L_upper_leg;
  l{8} = 0;
  l{9} = model_prms.L_pelvis;
  l{10} = 0;
  l{11} = model_prms.L_upper_leg;
  l{12} = model_prms.L_lower_leg;
  robot.l = l;
  
  m{1} = 0;
  m{2} = 0;
  m{3} = 0;
  m{4} = 0;
  m{5} = 0;
  m{6} = model_prms.M_lower_leg;
  m{7} = model_prms.M_upper_leg;
  m{8} = 0;
  m{9} = model_prms.M_pelvis;
  m{10} = 0;
  m{11} = model_prms.M_upper_leg;
  m{12} = model_prms.M_lower_leg;
  robot.m = m;
  robot.m_torso = model_prms.M_torso;
  robot.l_torso = model_prms.L_torso;
  robot.total_mass = sum([robot.m{:}]) + model_prms.M_torso;
  
  % Spatial inertias
  for i = 1:robot.nd
    robot.I{i} = mcI( m{i},...
    [ l{i}/2 , 0 , 0 ],...
    diag( [ 0 , 1/12*m{i}*l{i}^2 , 1/12*m{i}*l{i}^2 ] ) );
  end  
    
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
    elseif i == 1
      robot.appearance.body{i} = {};
    elseif i == 2 
      robot.appearance.body{i} = {};
    elseif i == 6
      robot.appearance.body{i} = ...
      {'colour',[204 0 0]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere};
    elseif i == 5
      robot.appearance.body{i} = ...
      {'colour',[204 0 0]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere };
    elseif i == 7
      robot.appearance.body{i} = ...
      {'colour',[204 0 0]/255,...
      'cyl', [0 0 0; l{i}+0.01 0 0], rlink,...
      'colour',[0.2,0.2,0.2],...
      'sphere',[0 0 0], rsphere };            
    elseif i == 8 || i == 9
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
  [p_9_to_T , R_9_to_T] = Model.get_torso_frame(robot);
  X_9_to_T = [ R_9_to_T , zeros(3,3);
  -R_9_to_T*crm(p_9_to_T) , R_9_to_T ];
  X_T_to_9_star = [ R_9_to_T' , crm(p_9_to_T)*R_9_to_T' ;
  zeros(3,3) , R_9_to_T' ];
  I_torso_in_9 = X_T_to_9_star * I_torso * X_9_to_T;
  robot.I{9} = robot.I{9} + I_torso_in_9;
  robot.appearance.body{9} = [ robot.appearance.body{9}(1:5),...
  'colour',[1 150 1]/255,...
  'cyl',[l{9}/2 0 0 ; l{9}/2 -model_prms.L_torso 0], rlink+0.1*rlink,...
  robot.appearance.body{9}(6:10) ];
  robot.Xtree = TreeTransforms(robot);
  
  robot.q = Var('q', robot.nd);
  robot.qd = Var('dq', robot.nd);
  robot.qdd = Var('ddq', robot.nd);
  
  % input torques
  robot.u = Var('u', 6);
  
  % actuation matrix (n x m)
  robot.B = [ zeros(6,6) ; eye(6,6) ];
  
  % selects the actuated DoF
  robot.H0 = [zeros(6) , eye(6)];
  
  % define theta (increasing phase variable)
  robot.c = [0 0 0 0 0 -1 -1/2 0 0 0 0 0];
end


function DH = DH_table( model )
  
  % ------------ Denavit Hartenberg parameters ---------------- %
  % {0}-frame to {1}-frame
  d0 = 0;
  theta0 = pi/2;
  
  % {0}-frame to {1}-frame
  alpha{1} = 0;
  a{1}     = 0;
  d{1}     = 0;
  theta{1} = 0;
  
  % {1}-frame to {2}-frame
  alpha{2} = 0;
  a{2}     = 0;
  d{2}     = 0;
  theta{2} = 0;
  
  % {2}-frame to {3}-frame
  alpha{3} = 0;
  a{3}     = 0;
  d{3}     = 0;
  theta{3} = 0;
  
  % {3}-frame to {4}-frame
  alpha{4} = 0;
  a{4}     = 0;
  d{4}     = 0;
  theta{4} = 0;
  
  % {4}-frame to {5}-frame
  alpha{5} = -pi/2;
  a{5}     = 0;
  d{5}     = 0;
  theta{5} = 0;
  
  % {5}-frame to {6}-frame
  d{6}     = 0;
  alpha{6} = pi/2;
  a{6}     = model.l{5};
  theta{6} = 0;
  
  % {6}-frame to {7}-frame
  d{7}     = 0;
  a{7}     = model.l{6};
  alpha{7} = 0;
  theta{7} = 0;
  
  % {7}-frame to {8}-frame
  d{8}     = 0;
  a{8}     = model.l{7};
  alpha{8} = 0;
  theta{8} = 0;
  
  % {8}-frame to {9}-frame
  d{9}     = 0;
  alpha{9} = -pi/2;
  a{9}     = model.l{8};
  theta{9} = 0;
  
  % {9}-frame to {10}-frame
  d{10}     = 0;
  alpha{10} = pi;
  a{10}     = model.l{9};
  theta{10} = 0;
  
  % {10}-frame to {11}-frame
  d{11}     = 0;
  alpha{11} = pi/2;
  a{11}     = model.l{10};
  theta{11} = 0;
  
  % {11}-frame to {12}-frame
  d{12}     = 0;
  alpha{12} = 0;
  a{12}     = model.l{11};
  theta{12} = 0;
  
  DH.d_0 = d0;
  DH.theta_0 = theta0;
  DH.theta = theta;
  DH.d = d;
  DH.alpha = alpha;
  DH.a = a;
end


function Xtree = TreeTransforms(model)
  DH = DH_table( model )
  for i = 1:model.nd
    if i == 1 
      Xtree{i} = xlt([DH.a{i} 0 DH.d{i}]')*round(rotx(DH.alpha{i}))*round(rotz(DH.theta_0))*xlt([0 0 DH.d_0]');
    else
      Xtree{i} = xlt([DH.a{i} 0 DH.d{i}]')*round(rotx(DH.alpha{i}))*round(rotz(DH.theta{i-1}))*xlt([0 0 DH.d{i-1}]');
      
    end
  end  
end
