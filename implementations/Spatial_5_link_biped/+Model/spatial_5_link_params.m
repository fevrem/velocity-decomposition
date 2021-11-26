function model_prms = spatial_5_link_params()
  L_lower_leg = 0.4;  % m
  L_upper_leg = 0.4;  % m 
  L_pelvis    = 0.25;  % m
  L_torso     = 0.3;  % m
  
  M_lower_leg = 2;  % kg
  M_upper_leg = 2;  % kg
  M_pelvis    = 2;  % kg
  M_torso     = 5;  % kg
 
  model_prms.L_lower_leg = L_lower_leg;
  model_prms.L_upper_leg = L_upper_leg;
  model_prms.L_pelvis = L_pelvis;
  model_prms.L_torso = L_torso;
  model_prms.M_lower_leg = M_lower_leg;
  model_prms.M_upper_leg = M_upper_leg;
  model_prms.M_pelvis = M_pelvis;
  model_prms.M_torso = M_torso;
end
