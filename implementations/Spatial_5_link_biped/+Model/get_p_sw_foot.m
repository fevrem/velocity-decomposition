function p_sw_foot_in_0 = get_p_sw_foot(RoBoDyn)
  p_sw_knee_in_0 = Model.get_p_sw_knee(RoBoDyn);
  p_sw_foot_in_0 = p_sw_knee_in_0 + RoBoDyn.T{12}(1:3,1:3)'*[RoBoDyn.model.l{12};0;0];
end