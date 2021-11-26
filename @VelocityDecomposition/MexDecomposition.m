function [vd] = MexDecomposition(vd, rbm)
  import casadi.*
  
  % Generate and mex c-functions of coupling expressions
  opts = struct('main', true, 'mex', true);
  
  % Create handles to mex'ed C-functions
  for i = 1:numel(vd.F)
    f = Function('f', {rbm.States.q.sym}, {vd.Y{i}});  
    CfileName = ['Y', num2str(i), '.c'];
    f.generate(CfileName, opts);
    fprintf(['\n', CfileName, '\n'])
    mex('-largeArrayDims', CfileName)
    clear f   
    vd.Y_cfun{i} = str2func(['Y', num2str(i)]);
    
    f = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {vd.w{i}});   
    CfileName = ['w', num2str(i), '.c'];
    f.generate(CfileName, opts);
    fprintf(['\n', CfileName, '\n'])
    mex('-largeArrayDims', CfileName)
    clear f    
    vd.w_cfun{i} = str2func(['w', num2str(i)]);
  end
  
  for i = 1:numel(vd.Fs)
    f = Function('f', {rbm.States.q.sym}, {vd.Yperp{i}});   
    CfileName = ['Yperp', num2str(i), '.c'];
    f.generate(CfileName, opts);
    fprintf(['\n', CfileName, '\n'])
    mex('-largeArrayDims', ['Yperp', num2str(i),'.c'])
    clear f   
    vd.Yperp_cfun{i} = str2func(['Yperp', num2str(i)]);
    
    f = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {vd.s{i}});   
    CfileName = ['s', num2str(i), '.c'];
    f.generate(CfileName, opts);
    fprintf(['\n', CfileName, '\n'])
    mex('-largeArrayDims', CfileName)
    clear f    
    vd.s_cfun{i} = str2func(['s', num2str(i)]);
  end
  
  for l = 1:numel(vd.Fs)
    f = Function('f', {rbm.States.q.sym}, {vd.B1{l}});   
    CfileName = ['B1_', num2str(l), '.c'];
    f.generate(CfileName, opts);
    fprintf(['\n', CfileName, '\n'])
    mex('-largeArrayDims', CfileName)
    clear f   
    vd.B1_cfun{l} = str2func(['B1_', num2str(l)]);
    
    f = Function('f', {rbm.States.q.sym}, {vd.B2{l}});   
    CfileName = ['B2_', num2str(l), '.c'];
    f.generate(CfileName, opts);
    fprintf(['\n', CfileName, '\n'])
    mex('-largeArrayDims', CfileName)
    clear f     
    vd.B2_cfun{l} = str2func(['B2_', num2str(l)]);
    
    f = Function('f', {rbm.States.q.sym}, {vd.B3{l}});   
    CfileName = ['B3_', num2str(l), '.c'];
    f.generate(CfileName, opts);
    fprintf(['\n', CfileName, '\n'])
    mex('-largeArrayDims', CfileName)
    clear f   
    vd.B3_cfun{l} = str2func(['B3_', num2str(l)]);
    
    f = Function('f', {rbm.States.q.sym}, {vd.B4{l}});   
    CfileName = ['B4_', num2str(l), '.c'];
    f.generate(CfileName, opts);
    fprintf(['\n', CfileName, '\n'])
    mex('-largeArrayDims', CfileName)
    clear f   
    vd.B4_cfun{l} = str2func(['B4_', num2str(l)]);
    
    f = Function('f', {rbm.States.q.sym}, {vd.B5{l}});   
    CfileName = ['B5_', num2str(l), '.c'];
    f.generate(CfileName, opts);
    fprintf(['\n', CfileName, '\n'])
    mex('-largeArrayDims', CfileName)
    clear f    
    vd.B5_cfun{l} = str2func(['B5_', num2str(l)]);
  end
end
