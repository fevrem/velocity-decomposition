function [p_9_to_T , R_9_to_T] = get_torso_frame(model)
  p_9_to_T = [model.l{9}/2;0;0];
  R_9_to_T = round(rotz(-pi/2));
  R_9_to_T = R_9_to_T(1:3,1:3);
end
