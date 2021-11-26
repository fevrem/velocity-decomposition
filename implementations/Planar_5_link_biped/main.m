clear ; clc ; close all

load('planar-5link-100mps.mat')

if 0
  [rbm] = ld_model(...
  {'model', @Model.planar_5_link},...
  {'debug', false});
  data = optimal_solution
else
  [rbm] = ld_model(...
  {'model', @Model.planar_5_link_minimal},...
  {'debug', false});
  data = optimal_solution;
  data.q = data.q(3:end,:);
  data.qd = data.qd(3:end,:);
  data.qdd = data.qdd(3:end,:)
end

% input force
for i = 1:4
  F{i} = rbm.Model.H0(i,:)';
end

% vectors for the uncontrolled directions
Fs{1} = [1; 0; 0; 0; 0];


% call VD class constructor
vd = VelocityDecomposition(rbm, F, Fs);

% new time
tEven = linspace(data.t(1), data.t(end), 1e3);
for i = 1:5
  qEven(i,:) = interp1(data.t, data.q(i,:), tEven);
  qdEven(i,:) = interp1(data.t, data.qd(i,:), tEven);
  qddEven(i,:) = interp1(data.t, data.qdd(i,:), tEven);
end
for i = 1:4
  uEven(i,:) = interp1(data.t, data.u(i,:), tEven);
end

% EVALUATE DECOMPOSITION
import casadi.*

fprintf('B1')
for i = 1:numel(vd.F)
  for j = 1:numel(vd.F)
    B1_1_fun = Function('f', {rbm.States.q.sym}, {vd.B1{1}(i,j)});
    for k = 1:numel(tEven)
      B1_1_num{i,j}(k) = full(B1_1_fun(qEven(:,k)));
    end
  end
end

fprintf(', B2')
for i = 1:numel(vd.F)
  for j = 1:numel(vd.Fs)
    B2_1_fun = Function('f', {rbm.States.q.sym}, {vd.B2{1}(i,j)});
    for k = 1:numel(tEven)
      B2_1_num{i,j}(k) = full(B2_1_fun(qEven(:,k)));
    end
  end
end

fprintf(', B3')
for i = 1:numel(vd.Fs)
  for j = 1:numel(vd.F)     
    B3_1_fun = Function('f', {rbm.States.q.sym}, {vd.B3{1}(i,j)});
    for k = 1:numel(tEven)
      B3_1_num{i,j}(k) = full(B3_1_fun(qEven(:,k)));
    end        
  end
end

fprintf(', B4')
for i = 1:numel(vd.Fs)
  for j = 1:numel(vd.Fs)
    B4_1_fun = Function('f', {rbm.States.q.sym}, {vd.B4{1}(i,j)});
    for k = 1:numel(tEven)
      B4_1_num{i,j}(k) = full(B4_1_fun(qEven(:,k)));
    end        
  end
end

fprintf(', B5')
B5_1_fun = Function('f', {rbm.States.q.sym}, {vd.B5{1}});
for k = 1:numel(tEven)
  B5_1_num(k) = full(B5_1_fun(qEven(:,k)));
end        

fprintf(', s')
s_1_fun = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {vd.s{1}});
for k = 1:numel(tEven)
  s_1_num(k) = full(s_1_fun(qEven(:,k), qdEven(:,k)));
end  

fprintf(', w\n')
w_1_fun = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {vd.w{1}});
w_2_fun = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {vd.w{2}});
w_3_fun = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {vd.w{3}});
w_4_fun = Function('f', {rbm.States.q.sym, rbm.States.dq.sym}, {vd.w{4}});
for k = 1:numel(tEven)
  w_1_num(k) = full(w_1_fun(qEven(:,k), qdEven(:,k)));
  w_2_num(k) = full(w_2_fun(qEven(:,k), qdEven(:,k)));
  w_3_num(k) = full(w_3_fun(qEven(:,k), qdEven(:,k)));
  w_4_num(k) = full(w_4_fun(qEven(:,k), qdEven(:,k)));
end  

H_fun = Function('f', {rbm.States.q.sym}, {rbm.Dynamics.H_matrix});
for k = 1:numel(tEven)
  H_num{k} = full(H_fun(qEven(:,k)));
  det_H_num(k) = det(full(H_fun(qEven(:,k))));
  for i = 1:5
    for j = 1:5
      H_num2{i,j}(k) = H_num{k}(i,j);
    end
  end
end  

for i = 1:4
  Y_fun{i} = Function('f', {rbm.States.q.sym}, {vd.Y{i}});
  for k = 1:numel(tEven)
    Y_num{i}(:,k) = full(Y_fun{i}(qEven(:,k)));
  end 
end

for i = 1:1
  Yp_fun{i} = Function('f', {rbm.States.q.sym}, {vd.Yperp{i}});
  for k = 1:numel(tEven)
    Yp_num{i}(:,k) = full(Yp_fun{i}(qEven(:,k)));
  end  
end
