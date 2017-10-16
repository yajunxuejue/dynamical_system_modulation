function rep_vec = calculateModulationSpeed(control_point, obstacle_points)
%DISTANCE_CALCULATE Summary of this function goes here
%   Detailed explanation goes here
obs_pt_con_pt = control_point - obstacle_points(:,i);
rep_vec = (2./(1+exp((6.7*norm(obs_pt_con_pt)-1)*8)))*obs_pt_con_pt/distance;
end
