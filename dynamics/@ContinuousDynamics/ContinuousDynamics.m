classdef ContinuousDynamics < handle 
  
  properties (GetAccess = public, SetAccess = protected)
    H_matrix  
    C_terms
    KE
    PE
    p_com
    v_com
    Krm
    H_matrix_inv
  end
  
  methods
    
    function obj = ContinuousDynamics(sys)
      import casadi.*
      
      [H,C] = HandC(obj, sys);
      obj.H_matrix = H;
      obj.C_terms  = C; 
      opts = struct('main', true, 'mex', true);
      f = Function('f', {sys.States.q.sym}, {obj.H_matrix}); 
      CfileName = 'H_matrix.c';
      f.generate(CfileName, opts);
      fprintf(['\n', CfileName, '\n'])
      mex('-largeArrayDims', CfileName)
      clear f   
            
      f = Function('f', {sys.States.q.sym, sys.States.dq.sym}, {obj.C_terms});   
      CfileName = 'C_terms.c';
      f.generate(CfileName, opts);
      fprintf(['\n', CfileName, '\n'])
      mex('-largeArrayDims', CfileName)
      clear f 
      
      [KE, PE, p_com, v_com] = EnergyAndMomentum(obj, sys);
      obj.KE = KE;
      obj.PE = PE;
      obj.p_com = p_com;
      obj.v_com = v_com;
      
      f = Function('f', {sys.States.q.sym}, {obj.p_com});   
      CfileName = 'Pcom.c';
      f.generate(CfileName, opts);
      fprintf(['\n', CfileName, '\n'])
      mex('-largeArrayDims', CfileName)
      clear f
      
      obj.H_matrix_inv = inv(obj.H_matrix);
      Krm = mat2cell(repmat(SX.zeros(sys.Model.nd, sys.Model.nd), 1, sys.Model.nd), sys.Model.nd, repmat(sys.Model.nd, 1, sys.Model.nd));
      for i = 1:sys.Model.nd
        for j = 1:sys.Model.nd
          for k = 1:sys.Model.nd            
            Krm{k}(i,j) = ChristoffelSymbols(obj, sys, i, j, k);
          end
        end
      end
      obj.Krm = Krm;
    end
  end
end
