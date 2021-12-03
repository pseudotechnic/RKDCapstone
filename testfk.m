% dh parameters (a, alpha, d, theta)

dha = [0, 90, 56.05, 0;
      330.30, 0, 103.55, 0;
      254.50, 0, -74.05, 0;
      0, 90, 91.00, 0;
      0, 0, 213.75, 0;];
  
  
dhcap = [0, 90, 56.05, 0;
      381, 0, 100, 0;
      317.5, 0, -65, 0;
      0, 90, 91, 0;
      0, 0, 55, 0;];
  

      % for rotating
      %0, 90, 0, 90;
      %0, 0, 0, 90];

% last two rows make x axis point along end effector, such that x is roll,
% y is pitch, and z is yaw
  
dof = size(dhcap, 1);
%disp(dof);

frames = zeros(4,4,dof);

for i = 1:dof
    dh = dhcap(i,:);
    
    frames(:,:,i) = [cosd(dh(4)), -sind(dh(4))*cosd(dh(2)), sind(dh(4))*sind(dh(2)), dh(1)*cosd(dh(4));
                            sind(dh(4)), cosd(dh(4))*cosd(dh(2)), -cosd(dh(4))*sind(dh(2)), dh(1)*sind(dh(4));
                            0, sind(dh(2)), cosd(dh(2)), dh(3);
                            0, 0, 0, 1];
                        
    disp(frames(:,:,i));
end

frame_ee = eye(4,4);

for i = 1:dof
    frame_ee = frame_ee * frames(:,:,i);
end

disp("ee");
disp(frame_ee);

H0i = zeros(4,4,dof);

for i = 1:dof
    dh = dhcap(i,:);
    % update thetas as variables
    %dh(4) = rad2deg(thetas(i));
    if i == 1
        H0i(:,:,i) = [cosd(dh(4)), -sind(dh(4))*cosd(dh(2)), sind(dh(4))*sind(dh(2)), dh(1)*cosd(dh(4));
                            sind(dh(4)), cosd(dh(4))*cosd(dh(2)), -cosd(dh(4))*sind(dh(2)), dh(1)*sind(dh(4));
                            0, sind(dh(2)), cosd(dh(2)), dh(3);
                            0, 0, 0, 1];
    else
        H0i(:,:,i) = H0i(:,:,i-1) * [cosd(dh(4)), -sind(dh(4))*cosd(dh(2)), sind(dh(4))*sind(dh(2)), dh(1)*cosd(dh(4));
                            sind(dh(4)), cosd(dh(4))*cosd(dh(2)), -cosd(dh(4))*sind(dh(2)), dh(1)*sind(dh(4));
                            0, sind(dh(2)), cosd(dh(2)), dh(3);
                            0, 0, 0, 1];
    end
        
    disp("H to frame");
    disp(i);
    disp(H0i(:,:,i));
end






Hee = H0i(:,:,end);
x = Hee(1,4);
y = Hee(2,4);
z = Hee(3,4);

psi = rad2deg(atan2(Hee(3,2), Hee(3,3)));
theta = rad2deg(atan2(-Hee(3,1), sqrt(Hee(3,2)^2 + Hee(3,3)^2)));
phi = rad2deg(atan2(Hee(2,1),Hee(1,1)));

%disp("psi");
%disp(psi);

%disp("theta");
%disp(theta);

%disp("phi");
%disp(phi);

%------------------------------------------------Jacobians
dha = [0, 90, 56.05, 0;
      330.30, 0, 103.55, 0;
      254.50, 0, -74.05, 0;
      0, 90, 91.00, 0;
      0, 0, 213.75, 0;];
H0ij = zeros(4,4,dof);
thetas = [20;20;30;45;60];

for i = 1:dof
    dh = dha(i,:);
    % update thetas as variables
    dh(4) = rad2deg(thetas(i));
    if i == 1
        H0ij(:,:,i) = [cosd(dh(4)), -sind(dh(4))*cosd(dh(2)), sind(dh(4))*sind(dh(2)), dh(1)*cosd(dh(4));
                            sind(dh(4)), cosd(dh(4))*cosd(dh(2)), -cosd(dh(4))*sind(dh(2)), dh(1)*sind(dh(4));
                            0, sind(dh(2)), cosd(dh(2)), dh(3);
                            0, 0, 0, 1];
    else
        H0ij(:,:,i) = H0ij(:,:,i-1) * [cosd(dh(4)), -sind(dh(4))*cosd(dh(2)), sind(dh(4))*sind(dh(2)), dh(1)*cosd(dh(4));
                            sind(dh(4)), cosd(dh(4))*cosd(dh(2)), -cosd(dh(4))*sind(dh(2)), dh(1)*sind(dh(4));
                            0, sind(dh(2)), cosd(dh(2)), dh(3);
                            0, 0, 0, 1];
    end
        
    disp("H to frame");
    disp(i);
    disp(H0ij(:,:,i));
end  
  
epsilon = 0.3;

%jacobians = zeros(6, size(thetas));

% for j=1:size(theta)
%     thetap = thetas;
%     thetap(j) = thetap(j) + epsilon;
%     thetam = thetas;
%     thetam(j) = thetam(j) + epsilon;
%                
%     framep = 0;
%     framepx = framep(1,6,:);
%     framepy = framep(2,6,:);
%     framepz = framep(3,6,:);
%     frameppsi = framep(4,6,:);
%     frameptheta = framep(5,6,:);
%     framepphi = framep(6,6,:);
%                
%     framem = fk(thetam);
%     framemx = framem(1,6,:);
%     framemy = framem(2,6,:);
%     framemz = framem(3,6,:);
%     framempsi = framem(4,6,:);
%     framemtheta = framem(5,6,:);
%     framemphi = framem(6,6,:);
%                
%     for i=1:size(framep,6)
%         jacobians(1, j, i) = (framepx(frame) - framemx(frame)) / (2 * epsilon);
%         disp(jacobians(1, j, i));
%         jacobians(2, j, i) = (framepy(frame) - framemy(frame)) / (2 * epsilon);
%         disp(jacobians(2, j, i));
%         jacobians(3, j, i) = (framepz(frame) - framemz(frame)) / (2 * epsilon);
%         disp(jacobians(3, j, i));
%         jacobians(4, j, i) = (frameppsi(frame) - framempsi(frame)) / (2 * epsilon);
%         disp(jacobians(4, j, i));
%         jacobians(5, j, i) = (frameptheta(frame) - framemtheta(frame)) / (2 * epsilon);
%         disp(jacobians(5, j, i));
%         jacobians(6, j, i) = (framepphi(frame) - framemphi(frame)) / (2 * epsilon);
%         disp(jacobians(6, j, i));
%     end
% end