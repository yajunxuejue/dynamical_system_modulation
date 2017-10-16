function points_cyl = createPointsCylinder(centre_p, r, length)
    step = 0.02;
    j = 0;
    for i = 0:step:length
        for phi = 0:0.1:2*pi
            j = j + 1;
            x = r*cos(phi) + centre_p(1,1);
            y = r*sin(phi) + centre_p(2,1);
            z = i;
            points_cyl(:,j) = [x; y; z];
        end
    end
end
