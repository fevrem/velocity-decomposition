function plt_contact_force( rbm , nlp , Fc , xDataArg , xData , varargin )
  [interpreter,linewidth,fontname,fontsize] = Plt.plt_settings();
  
  if nargin == 6
    plt_op = varargin{1};
    active_co = plt_op{1};
    active_style = plt_op{2};
    inactive_co = plt_op{3};
    inactive_style = plt_op{4};
  else
    active_co = 'g';
    active_style = '-';
    inactive_co = 'r';
    inactive_style = '--';     
  end
  
  % ground reaction forces and moments
  GRF_fx = Fc(:,1);
  GRF_fy = Fc(:,2);
  GRF_fz = Fc(:,3);
  GRF_nz = Fc(:,4);
  
  Plt.friction('spatial',nlp.con.friction.type,...
  nlp.con.friction.mu,...
  nlp.con.normal_force.min,...
  GRF_fx, GRF_fy, GRF_fz )
  plot3(GRF_fx,GRF_fy,GRF_fz,'k','linewidth',linewidth);
  
  figure; hold on; grid on
  plot(xData,GRF_nz,'k','linewidth',linewidth)
  supt = title('Normal Moment (Optimal Solution)');
  supt.FontWeight = 'bold';
  supt.FontName = fontname;
  supt.FontSize = fontsize;
  if nlp.con.normal_moment.bool
    yline(-nlp.con.normal_moment.max,'linewidth',linewidth,'Color',active_co,'LineStyle',active_style);
    yline(nlp.con.normal_moment.max,'linewidth',linewidth,'Color',active_co,'LineStyle',active_style);
  else
    yline(-nlp.con.normal_moment.max,'linewidth',linewidth,'Color',inactive_co,'LineStyle',inactive_style);
    yline(nlp.con.normal_moment.max,'linewidth',linewidth,'Color',inactive_co,'LineStyle',inactive_style);
  end
  yline(0,'linewidth',linewidth);  
  Plt.lablz( xDataArg , xData , '$n_z$ (N$\cdot$m)' , interpreter , fontname , fontsize ) 
end
