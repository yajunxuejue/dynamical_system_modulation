function points_squ = createPointsSquare(corner_point, width, height)
    step = 0.05;
    j = 0;
    for i = corner_point(1,1):step:(corner_point(2,1) + height) 
          j = j + 1;
          point_squ1(1,j) = corner_point(1,1);
          point_squ1(2,j) = i;
          point_squ2(1,j) = corner_point(1,1)+width;
          point_squ2(2,j) = i;
    end
    j = 0;
    for i = (corner_point(1,1) + step):step:(corner_point(1,1) + width)
          j = j + 1;
          point_squ3(1,j) = i;
          point_squ3(2,j) = corner_point(2,1);
          point_squ4(1,j) = i;
          point_squ4(2,j) = corner_point(2,1) + height;
    end
    points_squ = [point_squ1, point_squ2, point_squ3, point_squ4];
end
