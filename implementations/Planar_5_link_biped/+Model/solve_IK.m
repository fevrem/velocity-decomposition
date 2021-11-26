function [IKsol,IKterms] = solve_IK(RoBoDyn)
  R_0_to_1 = RoBoDyn.T{1}(1:3,1:3);
  R_0_to_2 = RoBoDyn.T{2}(1:3,1:3);
  R_0_to_3 = RoBoDyn.T{3}(1:3,1:3);
  R_0_to_4 = RoBoDyn.T{4}(1:3,1:3);
  R_0_to_5 = RoBoDyn.T{5}(1:3,1:3);
  R_0_to_6 = RoBoDyn.T{6}(1:3,1:3);
  R_0_to_7 = RoBoDyn.T{7}(1:3,1:3);
  R_0_to_8 = RoBoDyn.T{8}(1:3,1:3);
  R_0_to_9 = RoBoDyn.T{9}(1:3,1:3);
  R_0_to_10 = RoBoDyn.T{10}(1:3,1:3);
  R_0_to_11 = RoBoDyn.T{11}(1:3,1:3);
  R_0_to_12 = RoBoDyn.T{12}(1:3,1:3);
  
  % introduce {13}-frame "pointing in same direction" as {3}
  R_13_to_12 = Model.get_R_13_to_12;
  
  R_3_to_13 = R_13_to_12 * R_0_to_12 * R_0_to_3';
  
  % quantities needed to solve the inverse kinematics problem
  r13 = R_3_to_13(1,3);
  r23 = R_3_to_13(2,3);
  r31 = R_3_to_13(3,1);
  r32 = R_3_to_13(3,2);
  r33 = R_3_to_13(3,3);
  
  % solve IK problem
  phi_x_var = atan2( sqrt(r31^2+r32^2) , r33 );
  phi_y_var = atan2( r23/sin(phi_x_var) , -r13/sin(phi_x_var) );
  phi_z_var = atan2( r32/sin(phi_x_var) , r31/sin(phi_x_var) );
  
  IKterms.phix{1} = sqrt(r31^2+r32^2);
  IKterms.phix{2} = r33;
  IKterms.phiy{1} = r23/sin(phi_x_var);
  IKterms.phiy{2} = -r13/sin(phi_x_var);
  IKterms.phiz{1} = r32/sin(phi_x_var);
  IKterms.phiz{2} = r31/sin(phi_x_var);
  
  % pack the solution of IK
  IKsol{1} = phi_z_var;
  IKsol{2} = phi_x_var;
  IKsol{3} = phi_y_var;
  
end
