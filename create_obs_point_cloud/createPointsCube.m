function points_cub = createPointsCube(corner_point, width, height, length)
%CREATEPOINTSCUBE Summary of this function goes here
%   Detailed explanation goes here
  m = 1;
  for k = 0:0.01:length
        step = 0.01;
        for i = corner_point(1,1):step:(corner_point(2,1) + height) 
              point_squ1(1,1) = corner_point(1,1);
              point_squ1(2,1) = i;
              point_squ1(3,1) = corner_point(3,1) + k;
              points_cub(:, m) = point_squ1;
              m = m + 1;
              point_squ2(1,1) = corner_point(1,1)+width;
              point_squ2(2,1) = i;
              point_squ2(3,1) = corner_point(3,1) + k;
              points_cub(:, m) = point_squ2;
              m = m + 1;
        end
        for i = (corner_point(1,1) + step):step:(corner_point(1,1) + width)
              point_squ3(1,1) = i;
              point_squ3(2,1) = corner_point(2,1);
              point_squ3(3,1) = corner_point(3,1) + k;
              points_cub(:, m) = point_squ3;
              m = m + 1;
              point_squ4(1,1) = i;
              point_squ4(2,1) = corner_point(2,1) + height;
              point_squ4(3,1) = corner_point(3,1) + k;
              points_cub(:, m) = point_squ4;
              m = m + 1;
        end
  end
  for i = (corner_point(1,1) + step):step:(corner_point(1,1) + width - step)
     for j = (corner_point(2,1) + step):step:(corner_point(2,1) + height - step)
       point_squ5(1,1) = i;
       point_squ5(2,1) = j;
       point_squ5(3,1) = corner_point(3,1) + length;
       points_cub(:, m) = point_squ5;
       m = m + 1;
     end
  end
end
