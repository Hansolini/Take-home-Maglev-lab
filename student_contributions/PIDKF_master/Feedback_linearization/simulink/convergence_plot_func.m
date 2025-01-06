e_tol = 0.004;  %Error tolerance in m
partitions = 16;  %Circumference partitions
parallels = 9; %(not very) Parallell lines to test
Parallels_starts = zeros(2,parallels);
Parallels_ends = [0.01 0.02 0.03 0.04 0.05 0.06 0.07 0.08 0.09;0.09 0.08 0.07 0.06 0.05 0.04 0.03 0.02 0.01];

x_stable = zeros(1,parallels);
y_stable = zeros(1,parallels);

for n = 1:parallels
    convergence = 0;
    iteration = 1;
    point = Parallels_ends(:,n).*2;  %multiplied by two because of lazy programmer
    while  convergence == 0
        point = point./2
        stable = 0
        for i = 1:partitions
            ang = 2*pi*(5-i)/partitions;
            x = cos(ang)*point(1);
            y = sin(ang)*point(1);
            z = point(2);
            x0 = xLp + [x,y,z, zeros(1,9)]';
            sim fl
            x = ans.states(1:3,:,:);
            
            if x(1,end) < (xLp(1)+e_tol) && x(1,end) > (xLp(1)-e_tol) && x(2,end) < (xLp(2)+e_tol) && x(2,end) > (xLp(2)-e_tol) && x(3,end) < (xLp(3)+e_tol) && x(3,end) > (xLp(3)-e_tol) 
                stable = stable+1
            else
                break
            end
            if i == 5
                break;
            end
        end
        if stable == 5
            convergence = 1
            x_stable(n) = point(1)   %This is actually delta r
            y_stable(n) = point(2)   %This is actually delta z
        end
        iteration = iteration+1;
    end
end
% 
% add_points = [0.01 0.005 0.0075 0.0075 0.01 0.01 0.01 ; 
%               0.05 0.01  0.02   0.03   0.04 0.05 0.07 ];
% 
% filler = [0 0     0.011  0.011 0.008 0.007;
%           0 0.095 0.0925 0.03  0.02  0];
% 
% failed = [0.02 0.015 0.015 0.015 0.01 0.01 0.0125 0.0125 0.0125;
%           0.09 0.09 0.05   0.01  0.01 0.02 0.04   0.05 0.07];
% % %  
% %% ROC, original, u_sat = 255/s, 6 partitions, etol = 0.004
% figure
% scatter(x_stable,y_stable,"filled");
% grid on
% hold on
% fill(filler(1,:),filler(2,:),'r','FaceAlpha',0.3)
% scatter(add_points(1,:),add_points(2,:),"filled","r")
% scatter(failed(1,:),failed(2,:),"x","r")
% xlim([0 0.1])
% ylim([0 0.1])


% point = [0.0125;0.07];
% stable = 0;
%  for i = 1:partitions
%     ang = 2*pi*(5-i)/partitions;
%     x = cos(ang)*point(1);
%     y = sin(ang)*point(1);
%     z = point(2);
%     x0 = xLp + [x,y,z, zeros(1,9)]';
%     sim simulink_lqr
%     x = ans.states(1:3,:,:);
%     
%     if x(1,end) < (xLp(1)+e_tol) && x(1,end) > (xLp(1)-e_tol) && x(2,end) < (xLp(2)+e_tol) && x(2,end) > (xLp(2)-e_tol) && x(3,end) < (xLp(3)+e_tol) && x(3,end) > (xLp(3)-e_tol) 
%         stable = stable+1
%     else
%         break
%     end
%     if i == 5
%         break
%     end
%  end