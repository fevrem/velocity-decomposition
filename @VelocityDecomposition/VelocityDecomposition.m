classdef VelocityDecomposition < handle 
  
  properties (GetAccess = public, SetAccess = protected)
    F
    Fs
    Y
    Y_cfun
    w  
    w_cfun
    Yperp  
    Yperp_cfun
    s
    s_cfun        
    B1
    B1_cfun
    B2
    B2_cfun
    B3
    B3_cfun
    B4
    B4_cfun
    B5
    B5_cfun
    A1
    A1_cfun
    A2
    A2_cfun
    A3
    A3_cfun
    A4
    A4_cfun
    A5
    A5_cfun
  end
  
  methods
    
    function vd = VelocityDecomposition(rbm, controlled_directions, uncontrolled_directions)
      arguments 
        rbm (1,1) DynamicalSystem
        controlled_directions (1,:) cell
        uncontrolled_directions (1,:) cell
      end
      
      import casadi.*
      
      vd.F = controlled_directions;
      vd.Fs = uncontrolled_directions;    
      if numel(vd.F) + numel(vd.Fs) ~= numel(rbm.States.q.sym)
        error('Error.\nThere must be %d directions, not %d.', numel(rbm.States.q.sym), numel(vd.F) + numel(vd.Fs));
      end
      
      % get controlled and uncontrolled directions symbolically
      vd = GetDirections(vd, rbm);
      
      % get decomposed controlled and uncontrolled velocities symbolically
      vd = GetVelocities(vd, rbm);
      
      % get coupling between controlled and uncontrolled directions
      vd = GetCoupling(vd, rbm);
      
      % generate C-functions, Mex C-functions, and create handles to Mex functions
      vd = MexDecomposition(vd, rbm)
    end
  end
end
