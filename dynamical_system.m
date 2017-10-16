function xd = globallyStableDS(init_p, target_p)
 K = 5;
 if (target_p - init_p)<0.01
     xd = 0;
 else
     xd = 10*(target_p - init_p);%/norm(target_p - init_p);%
 end   
end
