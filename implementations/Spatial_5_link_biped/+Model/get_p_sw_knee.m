function p_sw_knee_in_0 = get_p_sw_knee(RoBoDyn)
  % Based on the definition of Homogeneous Transforms, p = - p_k_in_k
  p_sw_knee_in_sw_knee =  RoBoDyn.T{12}(1:3,4);
  p_sw_knee_in_0 =  -RoBoDyn.T{12}(1:3,1:3)'*p_sw_knee_in_sw_knee;
end