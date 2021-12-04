currentDir = fileparts(mfilename('fullpath'));

hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));
figure();
%disp(hebilog.position);

dhcap = [0, 90, 56.05, 0;
      381, 0, 100, 0;
      317.5, 0, -65, 0;
      0, 90, 91, 0;
      0, 0, 55, 0;];
  
dof = size(dhcap, 1);

H0ij = zeros(4,4,dof);
thetas = hebilog.position;
disp("print");
disp(size(thetas,1));
for i = 1:size(thetas(:,1),1);
    for j = 1:size(thetas,2)
        dh = dhcap(j,:);
        % update thetas as variables
        dh(4) = rad2deg(thetas(i,j));
        if j == 1
            H0ij(:,:,j) = [cosd(dh(4)), -sind(dh(4))*cosd(dh(2)), sind(dh(4))*sind(dh(2)), dh(1)*cosd(dh(4));
                                sind(dh(4)), cosd(dh(4))*cosd(dh(2)), -cosd(dh(4))*sind(dh(2)), dh(1)*sind(dh(4));
                                0, sind(dh(2)), cosd(dh(2)), dh(3);
                                0, 0, 0, 1];
        else
            H0ij(:,:,j) = H0ij(:,:,j-1) * [cosd(dh(4)), -sind(dh(4))*cosd(dh(2)), sind(dh(4))*sind(dh(2)), dh(1)*cosd(dh(4));
                                sind(dh(4)), cosd(dh(4))*cosd(dh(2)), -cosd(dh(4))*sind(dh(2)), dh(1)*sind(dh(4));
                                0, sind(dh(2)), cosd(dh(2)), dh(3);
                                0, 0, 0, 1];
        end
     end
end

Hee = H0ij(:,:,end);
x = Hee(1,4);
y = Hee(2,4);
z = Hee(3,4);

plot3(x,y,z);