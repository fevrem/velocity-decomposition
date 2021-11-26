function [pitch,roll] = absolute_torso_angles(x)

roll  =  x(5) - x(9) ;
pitch = x(6) + x(7) + x(8) ;

end