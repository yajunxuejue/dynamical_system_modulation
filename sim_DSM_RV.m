function [sim_result, command_speed] = simPointMDS( obs_points, init_pose, target_pose ,inte_step, bSaveAVI, sim_dimention)
%SIMPOINTMDS Summary of this function goes here
  %Detailed explanation goes here
  if bSaveAVI
    writerObj=VideoWriter('square_squriouse_equilibram.avi');  %// 定义一个视频文件用来存动画  
    open(writerObj);                    %// 打开该视频文件  
    %writeVideo(writerObj,rand(300));
  end

  if sim_dimention == 2
      spourious_point = 0;
      arrived_target = 1;
      i=1;
      r = 0.4;
      phi = 0:0.01:2*pi;
      x = r*cos(phi) + init_pose(1,1);
      y = r*sin(phi) + init_pose(2,1);
      obs = plot(obs_points(1,:), obs_points(2,:),'r.','markersize',5);
      hold on;
      h = plot(init_pose(1, 1), init_pose(2, 1), 'b*', 'erasemode', 'none','markersize',5,'linewidth',1.5);
      hold on;
      c = plot(x, y,'--');
      axis([0 3 0 3]);
      xlabel('x(m)');
      ylabel('y(m)');
      axis equal;
      grid on;
      while arrived_target
          speed = globallyStableDS(init_pose, target_pose);
          if speed == 0
              arrived_target = 0; 
          else
              com_speed = calculateModulationSpeed(speed, init_pose, [100;100], obs_points, 2, spourious_point);
              command_speed(1,i) = norm(com_speed);
              command_speed(2,i) = 0.01*i;
%               if norm(com_speed)<0.01
%                   arrived_target = 0;
%               end
              init_pose =init_pose + inte_step*com_speed;
              sim_result(:,i) = init_pose;
              x = r*cos(phi) + init_pose(1,1);
              y = r*sin(phi) + init_pose(2,1);
              set(h, 'XData', sim_result(1, :), 'YData', sim_result(2, :));
              set(c, 'XData', x, 'YData', y);
              if i < 50
                 obs_points(2, 39) = obs_points(2, 39) - 0.001;
              end
              set(obs, 'XData', obs_points(1, :), 'YData', obs_points(2, :));
              drawnow
              if bSaveAVI
                  frame = getframe;            %// 把图像存入视频文件中  
                  writeVideo(writerObj,frame); %// 将帧写入视频
              end
              pause(0.01);
          end
          i=i+1;
      end
      if bSaveAVI
          close(writerObj); %// 关闭视频文件句柄 
      end
  end
  if sim_dimention == 3
      %Robotic toolbox KUKA LWR IIWA
      startup_rvc;
      mdl_LWR;
      %
      global spourious_point;
      spourious_point = 0;
      clf;
      pause(3);
      arrived_target_3 = 1;
      i=1;
      plot3(target_pose(1,1), target_pose(2,1), target_pose(3,1), 'r*','markersize',10);
      hold on;
      obs = plot3(obs_points(1,:), obs_points(2,:), obs_points(3,:),'r.','markersize',5);
      hold on;
      h = plot3(init_pose(1, 1), init_pose(2, 1), init_pose(3, 1), 'g*', 'erasemode', 'none','markersize',5,'linewidth',1.5);
      %hold on
      %[x, y, z] = ellipsoid(init_pose(1,1), init_pose(2,1), init_pose(3,1), 0.4, 0.4, 0.4); 
      %sur_area = mesh(x, y, z);
      %axis([0 1.5 0 1.5 0 1.5]);
      set(gca,'FontSize',20); %set the axis font size
      hold on;
      %fill3([0 1.5 1.5 0], [0 0 1.5 1.5], [0 0 0 0],'c');
      %axis equal;
      grid on;
      xlabel('x(m)');
      ylabel('y(m)');
      zlabel('z(m)');
      %view(2);
      while arrived_target_3
          speed_3 = globallyStableDS(init_pose, target_pose);
          if speed_3 == 0
              arrived_target_3 = 0; 
          else
              if i==1
                 q_seed = [0,1,0,-1.5,0,0.5,0];
              end
              jacob_LWR = LWR.jacob0(q_seed);
              LWR_rpinv = pinv(jacob_LWR);
              T_cp = LWR_partial.fkine([q_seed(1,1),q_seed(1,2),q_seed(1,3)]);
              cp_position = [T_cp.t(1,1); T_cp.t(2,1); T_cp.t(3,1)];
              jacob_LWR_partial = LWR_partial.jacob0([q_seed(1,1),q_seed(1,2),q_seed(1,3)]);
              LWR_partial_rpinv = pinv(jacob_LWR_partial);
              [ee_com_speed, cp_com_speed, cp_min_distance] = calculateModulationSpeed(speed_3, init_pose, cp_position, obs_points, 3, spourious_point);
              init_pose =init_pose + inte_step*ee_com_speed;
              sim_result(:,i) = init_pose;
              weight =1 - 1./(exp((5*cp_min_distance-1)*12));
              if weight<0
                  weight = 0;
              end
              q_dot_ = LWR_partial_rpinv*[cp_com_speed(1,1); cp_com_speed(2,1); cp_com_speed(3,1);0;0;0];
              q_dot_1 = LWR_rpinv*weight*[ee_com_speed(1,1); ee_com_speed(2,1); ee_com_speed(3,1);0;0;0] ;
              q_dot_2 = (eye(7) - weight*LWR_rpinv*jacob_LWR)*[q_dot_;0;0;0;0];
              q_dot = q_dot_1 + 10*q_dot_2;
              q = q_seed + inte_step*q_dot';
              % T_new = [ 1, 0, -0.1411, init_pose(1,1);
              %          0, 1, 0,       init_pose(2,1);
              %          0, 0, 1,       init_pose(3,1);
              %          0, 0, 0,       1       ];
              %q = LWR.ikine(T_new, q_seed);
              q_seed = q;
              LWR.plot(q);
              %[x, y, z] = ellipsoid(init_pose(1,1), init_pose(2,1), init_pose(3,1), 0.4, 0.4, 0.4); 
              set(h, 'XData', sim_result(1, :), 'YData', sim_result(2, :), 'ZData', sim_result(3, :));
              if i < 160
                 %obs_points(1,:) = obs_points(1,:) + 0.005;
                 obs_points(2,:) = obs_points(2,:) + 0.008;
              end
              set(obs, 'XData', obs_points(1, :), 'YData', obs_points(2, :), 'ZData', obs_points(3, :));
              %set(sur_area, 'XData', x, 'YData', y, 'ZData', z);
              drawnow
              %if bSaveAVI
              %    frame = getframe;            %// 把图像存入视频文件中  
              %    %frame.cdata = imresize(frame.cdata, [300 300]); %// 设置视频宽高：H为行数(高)，W为列数(宽)  
              %   writeVideo(writerObj,frame); %// 将帧写入视频
              %end
              pause(0.002);
          end
          i = i + 1; 
      end
      %if bSaveAVI
      %    close(writerObj); %// 关闭视频文件句柄 
      %end
  end
end
function plot = plotResult(x, y)
  hold on;
  plot(x, y,'ok','markersize',2,'linewidth',7.5);
end
function xd = globallyStableDS(init_p, target_p)
 K = 5;
 if (target_p - init_p)<0.01
     xd = 0;
 else
     xd = 10*(target_p - init_p);%/norm(target_p - init_p);%
 end   
end
function [] = plotCircle(center_x, center_y, radius)

end
