function [ee_command_velocity, control_point_link_velocity, link_min_distance] = calculateModulationSpeed( raw_speed, ee_control_point, link_control_point, obstacle_points, sim_dimention, spourious_point )
%DISTANCE_CALCULATE Summary of this function goes here
%   Detailed explanation goes here
  min_distance = 0.4;
  link_min_distance = 0.4;
  obs_sur = 0; 
  link_obs_sur = 0;
  if sim_dimention == 2
      sum_rep_vec = [0;0];  
      link_sum_rep_vec = [0;0];
      for i=1:1:size(obstacle_points,2)
         obs_pt_con_pt = ee_control_point - obstacle_points(:,i);
         obs_link_con_pt = link_control_point - obstacle_points(:,i);
         distance = norm(obs_pt_con_pt);
         distacne_obs_link = norm(obs_link_con_pt);
         if distance < 0.4
             obs_sur = 1;
             rep_vec = (2./(1+exp((6.7*norm(obs_pt_con_pt)-1)*8)))*obs_pt_con_pt/distance;
             sum_rep_vec = rep_vec + sum_rep_vec;
             if distance < min_distance
                  min_distance = distance;
             end  
         end
         if distacne_obs_link < 0.4
             link_obs_sur = 1;
             link_rep_vec = (2./(1+exp((6.7*distacne_obs_link-1)*8)))*obs_link_con_pt/distacne_obs_link;
             link_sum_rep_vec = link_sum_rep_vec + link_rep_vec;
             if distacne_obs_link < link_min_distance
                  link_min_distance = distacne_obs_link;
             end            
         end
      end
      if obs_sur == 0
          ee_command_velocity = raw_speed;
      else
         if norm(sum_rep_vec) == 0
             unit_rep_vec_1 = [0;0];
         elseif spourious_point == 0
             unit_rep_vec_1 = sum_rep_vec/norm(sum_rep_vec);
             unit_rep_vec_2 = [unit_rep_vec_1(2, 1);-unit_rep_vec_1(1,1)];
             T = [unit_rep_vec_1(1,1), unit_rep_vec_1(2,1); 
                  unit_rep_vec_2(1,1), unit_rep_vec_2(2,1)];
             T_inv = [unit_rep_vec_1(1,1), unit_rep_vec_2(1,1); 
                      unit_rep_vec_1(2,1), unit_rep_vec_2(2,1)];
             if unit_rep_vec_1'*raw_speed < 0
                 lamda_1 = 1 - 2./(1+exp((6.7*min_distance-1)*8));
                 lamda_2 = 1 + 0.5./(1+exp((6.7*min_distance-1)*8));
                 E = [lamda_1,    0     ; 
                         0   , lamda_2 ];
             else
                 lamda_1 = 1 + 1./(1+exp((6.7*min_distance-1)*8));
                 lamda_2 = 1 + 1./(1+exp((6.7*min_distance-1)*8));
                 E = [lamda_1,    0     ; 
                         0   , lamda_2 ]; 
             end
             ee_command_velocity = (T*E*T_inv)*raw_speed;
             if (norm(raw_speed) > 0) && (norm(ee_command_velocity)<0.01)
                spourious_point = 1;
             end 
         else
             unit_rep_vec_1 = sum_rep_vec/norm(sum_rep_vec);
             unit_rep_vec_2 = [unit_rep_vec_1(2, 1);-unit_rep_vec_1(1,1)];
             T = [unit_rep_vec_1(1,1), unit_rep_vec_1(2,1); 
                  unit_rep_vec_2(1,1), unit_rep_vec_2(2,1)];
             T_inv = [unit_rep_vec_1(1,1), unit_rep_vec_2(1,1); 
                      unit_rep_vec_1(2,1), unit_rep_vec_2(2,1)];
             if unit_rep_vec_1'*raw_speed < 0
                 lamda_1 = 1 - 2./(1+exp((6.7*min_distance-1)*8));
                 lamda_2 = 1 + 0.5./(1+exp((6.7*min_distance-1)*8));
                 E = [lamda_1,    0     ; 
                         0   , lamda_2 ];
             else
                 lamda_1 = 1 + 1./(1+exp((6.7*min_distance-1)*8));
                 lamda_2 = 1 + 1./(1+exp((6.7*min_distance-1)*8));
                 E = [lamda_1,    0     ; 
                         0   , lamda_2 ]; 
             end
             ee_command_velocity = (T*E*T_inv)*10*unit_rep_vec_2;
             if unit_rep_vec_1'*raw_speed > 0 
                spourious_point = 0;
             end 
         end 
      end
      if link_obs_sur == 1
             control_point_link_velocity = (2./(1+exp((6.7*link_min_distance-1)*8)))*link_sum_rep_vec/norm(link_sum_rep_vec);
      else
             control_point_link_velocity = [0;0];
      end
  end
  if sim_dimention == 3
      sum_rep_vec = [0;0;0];  
      link_sum_rep_vec = [0;0;0];
      for i=1:1:size(obstacle_points,2)
         obs_pt_con_pt = ee_control_point - obstacle_points(:,i);
         obs_link_con_pt = link_control_point - obstacle_points(:,i);
         distance = norm(obs_pt_con_pt);
         distacne_obs_link = norm(obs_link_con_pt);
         if distance < 0.4
             obs_sur = 1;
             rep_vec = (2./(1+exp((5*norm(obs_pt_con_pt)-1)*8)))*obs_pt_con_pt/distance;
             sum_rep_vec = rep_vec + sum_rep_vec;
             if distance < min_distance
                  min_distance = distance;
             end  
         end
         if distacne_obs_link < 0.4
             link_obs_sur = 1;
             link_rep_vec = (2./(1+exp((5*distacne_obs_link-1)*8)))*obs_link_con_pt/distacne_obs_link;
             link_sum_rep_vec = link_sum_rep_vec + link_rep_vec;
             if distacne_obs_link < link_min_distance
                  link_min_distance = distacne_obs_link;
             end 
         end
      end
      if obs_sur == 0
          ee_command_velocity = raw_speed;
      else
         if norm(sum_rep_vec) == 0
             unit_rep_vec_1 = [0;0;0];
         elseif spourious_point == 0
             unit_rep_vec_1 = sum_rep_vec/norm(sum_rep_vec);
             unit_rep_vec_2 = [unit_rep_vec_1(2, 1);-unit_rep_vec_1(1,1); 0];
             unit_rep_vec_2 = unit_rep_vec_2/norm(unit_rep_vec_2);
             unit_rep_vec_3 = cross(unit_rep_vec_1, unit_rep_vec_2);
             unit_rep_vec_3 = unit_rep_vec_3/norm(unit_rep_vec_3);
             T = [unit_rep_vec_1(1,1), unit_rep_vec_1(2,1), unit_rep_vec_1(3,1); 
                  unit_rep_vec_2(1,1), unit_rep_vec_2(2,1), unit_rep_vec_2(3,1);
                  unit_rep_vec_3(1,1), unit_rep_vec_3(2,1), unit_rep_vec_3(3,1)];
             T_inv = [unit_rep_vec_1(1,1), unit_rep_vec_2(1,1), unit_rep_vec_3(1,1); 
                      unit_rep_vec_1(2,1), unit_rep_vec_2(2,1), unit_rep_vec_3(2,1);
                      unit_rep_vec_1(3,1), unit_rep_vec_2(3,1), unit_rep_vec_3(3,1)];
             if unit_rep_vec_1'*raw_speed < 0
                 lamda_1 = 1 - 2./(1+exp((5*min_distance-1)*8));
                 lamda_2 = 1 + 1./(1+exp((5*min_distance-1)*8));
                 lamda_3 = 1 + 1./(1+exp((5*min_distance-1)*8));
                 E = [lamda_1,    0   ,    0  ; 
                         0   , lamda_2,    0  ;
                         0   ,    0   , lamda_3];
             else
                 lamda_1 = 1 + 1./(1+exp((5*min_distance-1)*8));
                 lamda_2 = 1 + 1./(1+exp((5*min_distance-1)*8));
                 lamda_3 = 1 + 1./(1+exp((5*min_distance-1)*8));
                 E = [lamda_1,    0   ,    0  ; 
                         0   , lamda_2,    0  ;
                         0   ,    0   , lamda_3];
             end
                 ee_command_velocity = (T_inv*E*T)*raw_speed;
                 if (norm(raw_speed) > 0) && (norm(ee_command_velocity)<0.05)
                     spourious_point = 1;
                 end 
          else
             unit_rep_vec_1 = sum_rep_vec/norm(sum_rep_vec);
             unit_rep_vec_2 = [unit_rep_vec_1(2, 1);-unit_rep_vec_1(1,1); 0];
             unit_rep_vec_2 = unit_rep_vec_2/norm(unit_rep_vec_2);
             unit_rep_vec_3 = cross(unit_rep_vec_1, unit_rep_vec_2);
             unit_rep_vec_3 = unit_rep_vec_3/norm(unit_rep_vec_3);
             T = [unit_rep_vec_1(1,1), unit_rep_vec_1(2,1), unit_rep_vec_1(3,1); 
                  unit_rep_vec_2(1,1), unit_rep_vec_2(2,1), unit_rep_vec_2(3,1);
                  unit_rep_vec_3(1,1), unit_rep_vec_3(2,1), unit_rep_vec_3(3,1)];
             T_inv = [unit_rep_vec_1(1,1), unit_rep_vec_2(1,1), unit_rep_vec_3(1,1); 
                      unit_rep_vec_1(2,1), unit_rep_vec_2(2,1), unit_rep_vec_3(2,1);
                      unit_rep_vec_1(3,1), unit_rep_vec_2(3,1), unit_rep_vec_3(3,1)];
             if unit_rep_vec_1'*raw_speed < 0
                 lamda_1 = 1 - 2./(1+exp((5*min_distance-1)*8));
                 lamda_2 = 1 + 1./(1+exp((5*min_distance-1)*8));
                 lamda_3 = 1 + 1./(1+exp((5*min_distance-1)*8));
                 E = [lamda_1,    0   ,    0  ; 
                         0   , lamda_2,    0  ;
                         0   ,    0   , lamda_3];
             else
                 lamda_1 = 1 + 1./(1+exp((5*min_distance-1)*8));
                 lamda_2 = 1 + 1./(1+exp((5*min_distance-1)*8));
                 lamda_3 = 1 + 1./(1+exp((5*min_distance-1)*8));
                 E = [lamda_1,    0   ,    0  ; 
                         0   , lamda_2,    0  ;
                         0   ,    0   , lamda_3];
             end
             ee_command_velocity = (T*E*T_inv)*3*unit_rep_vec_2;
             if unit_rep_vec_1'*raw_speed > 0 
                  spourious_point = 0;
             end 
         end
      end
      if link_obs_sur == 1
             control_point_link_velocity = (10./(1+exp((5*link_min_distance-1)*6)))*link_sum_rep_vec/norm(link_sum_rep_vec);
      else
             control_point_link_velocity = [0;0;0];
     end
  end
end
