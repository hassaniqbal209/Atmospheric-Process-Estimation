%%
clc;
clear all;
close all;
%dimensions
F = 100;
Lx = F;
Ly = F;
Lz = 50;
Nx = 101;
Ny = 101;
Nz = 51;
dx = Lx/(Nx-1);
dy = Ly/(Ny-1);
dz = Lz/(Nz-1);
Nt  = 100; %267; %200000; %amount of iterations

%CFL conditions, determines how big dt should be for the equation to converge
c = 1;
C = 0.075; %C<1
dt = 2; %dx*C/c; %2;

%v = VideoWriter('C:\Users\caee-hi2369\OneDrive - The University of Texas at Austin\Documents\MATLAB\newfile.avi');
%open(v);

%field variables
cn=zeros(Nx,Ny,Nz); %concentration matrix
x=linspace(0,Lx,Nx); %xdistance
y=linspace(0,Ly,Ny); %ydistance
z=linspace(0,Lz,Nz); %zdistance
[X,Y,Z]=meshgrid(x,y,z); %defining 3D grid

%True puff parameters
theta = [1000,12,2,5,0,0];

%initial condition
t=0;

%Data generation
truedata = zeros(Nt,Nx,Ny,Nz);
measurements = zeros(Nt,Nx,Ny,Nz);
SNR = 1000;
noise = zeros(Nt,Nx,Ny,Nz);
W_diag = zeros(Nt,Nx,Ny,Nz);

for n=1:Nt
    
    %Update time
    t=t+dt;
    
    for i=1:Nx
        for j=1:Ny
            for k=1:Nz
                cn(i,j,k) = ADEquation(theta,i,j,k,t);
            end
        end
    end
    
    %     %Applying the source term
    %     if (t<3600)
    %         cn(10,10,10)= cn(10,10,10)+ 20*dt;
    %     elseif (t<2*3600)
    %         cn(10,10,5)= cn(10,10,5) - 20*dt;
    %     end
    
    %Visualization
    if(mod(n,1) == 0) %updates image every 1 second (0.25 minutes), this has been done to speed up computation
        slice(X,Y,Z,cn,20,50,2);
        colormap(flipud(hsv(256)));
        colorbar; 
        caxis([0 1.5*10^-5]); %1.5*10^-10
        title(sprintf('time = %f seconds', t));
        xlabel('x[m]') ;
        ylabel('y[m]');
        zlabel('z[m]');
        pause(0.05);
        %print -dmeta;
        %frame = getframe;
        %writeVideo(v,frame);
    end
    
    truedata(n,:,:,:)=cn;
    for i=1:Nx
        for j=1:Ny
            for k=1:Nz
                variance = (cn(i,j,k)^2)/SNR;
                noise(n,i,j,k) = sqrt(variance).*randn(1);
                W_diag(n,i,j,k) =  1/sqrt(variance);
                measurements(n,i,j,k)= cn(i,j,k) + noise(n,i,j,k);
            end
        end
    end
end

time = 0:dt:t;
%close(v);
save 3DDiffusionTrueValue time truedata 
save 3DDiffusionMeasurements time measurements W_diag

%% Parameter Estimation
close all;
clear all;
clc;
load 3DDiffusionMeasurements;
load 3DDiffusionTrueValue;

%Parameters
F = 100;
Nt = 100;
t0 = 40;
dt = 2;
duration = 50;
time = linspace(0,Nt*dt,101);

nw = 4; %waypoints per optimization
nr = 3; %number of robots
Lx = F;
Ly = F;
Lz = 50;
Nx = 101;
Ny = 101;
Nz = 51;
x=linspace(0,Lx,Nx); %xdistance
y=linspace(0,Ly,Ny); %ydistance
z=linspace(0,Lz,Nz); %zdistance

%dynamics constraints
vmax = 10; %max x/y/z velocity
umax = 3; %max x/y/z acceleration

%initial estimate
theta0 = [700 20 40 25 1 30];
x10 = [0 50 5];
x20 = [0 0 5];
x30 = [25 75 5];
u10 = [umax/3 0 umax/3];
u20 = [umax/3 umax/3 umax/3];
u30 = [umax/3 umax/3 umax/3];
v10 = [vmax/2 0 vmax/5];
v20 = [vmax/2 vmax/2 vmax/5];
v30 = [vmax/2 vmax/2 vmax/5];

v = zeros(nr*(nw+2),1);
pos = zeros(nr*(nw+2),4);
alpha = zeros(nr*(nw+2),1);

t = t0;

theta = optimvar('theta',6,1,'LowerBound',[0 0 0 0 0 0],'UpperBound',[2000 50 F F 50 Nt*dt]);
c_theta = optimexpr(nr*(nw+2));
xtheta.theta = theta0;
thetaf = theta0;
%Intialization
x1= x10;
x2= x20;
x3= x30;
v1= v10;
v2= v20;
v3= v30;
u1= u10;
u2= u20;
u3= u30;
xqr1 = [x1 v1];
xqr2 = [x2 v2];
xqr3 = [x3 v3];

for w=1:nr:nr*(nw+2)
    
    pos(w,:) = [x1(1),x1(2),x1(3),t];
    pos(w+1,:) = [x2(1),x2(2),x2(3),t];
    pos(w+2,:) = [x3(1),x3(2),x3(3),t];
    
    %dyanmics robot
    xqr1 = xqr1 + dt * [v1 u1];
    xqr2 = xqr2 + dt * [v2 u2];
    xqr3 = xqr3 + dt * [v3 u3];

    x1 =  xqr1(1:3); 
    x2 =  xqr2(1:3); 
    x3 =  xqr3(1:3); 
    
    if (x1(1) > (F-30))
        v1(1) = -v1(1);
        u1(1) = -u1(1);
    end
    if (x1(2) > (F-30))
        v1(2) = -v1(2);
        u1(2) = -u1(2);
    end
    if (x1(3) > (50-10))
        v1(3) = -v1(3);
        u1(3) = -u1(3);
    end
    
     if (x2(1) > (F-30))
        v2(1) = -v2(1);
        u2(1) = -u2(1);
    end
    if (x2(2) > (F-30))
        v2(2) = -v2(2);
        u2(2) = -u2(2);
    end
    if (x2(3) > (50-10))
        v2(3) = -v2(3);
        u2(3) = -u2(3);
    end
    
     if (x3(1) > (F-30))
        v3(1) = -v3(1);
        u3(1) = -u3(1);
    end
    if (x3(2) > (F-30))
        v3(2) = -v3(2);
        u3(2) = -u3(2);
    end
    if (x3(3) > (50-10))
        v3(3) = -v3(3);
        u3(3) = -u3(3);
    end
    
    
    if (x1 < 4)
        v1 = -v1;
        u1 = -u1;
    end
    if (x2 < 4)
        v2 = -v2;
        u2 = -u2;
    end
    if (x3 < 4)
        v3 = -v3;
        u3 = -u3;
    end
   t = t+ dt;
end
 
initialv = zeros(nr*(nw+2),3); 
initialu = zeros(nr*(nw+2),3);
t = t0;

for w = 1:nr:nr*(nw+2)
    initialv(w,:) = v10;
    initialv(w+1,:) = v20;
    initialv(w+2,:) = v30;
    
    initialu(w,:) = u10;
    initialu(w+1,:) = u20;
    initialu(w+2,:) = u30;
end

optimpos0 = [pos(:,1:4),initialv, initialu];
optimpos = optimvar('optimpos',nr*(nw+2),10,'LowerBound',[zeros(nr*(nw+2),4),zeros(nr*(nw+2),3),...
        zeros(nr*(nw+2),3)],'UpperBound',[F*ones(nr*(nw+2),2),50*ones(nr*(nw+2),1),Nt*dt*ones(nr*(nw+2),1),...
        vmax*ones(nr*(nw+2),3),umax*ones(nr*(nw+2),3)]);
xoptimpos.optimpos = optimpos0;
Joc = optimexpr(nr*(nw+2),length(theta0));
H = zeros(nr*(nw+2),length(theta0));

%Figure plot data
robot1_pos = zeros(duration/dt,4);
robot2_pos = zeros(duration/dt,4);
robot3_pos = zeros(duration/dt,4);
robot1_index =1;
robot2_index =1;
robot3_index =1;
theta_history = zeros(ceil(duration/nw),6);
theta_index = 1;
n_index= 1:Nt;
kk = 1;
pos_database = zeros(18*5,4);
while (t<t0+duration)
    
    for w=1:nr:nr*(nw+2)
        
        n = find(time==t);
        pos(w,1) = round(pos(w,1));
        pos(w,2) = round(pos(w,2));
        pos(w,3) = round(pos(w,3));
        pos(w,4) = t;
        
        pos(w+1,1) = round(pos(w+1,1));
        pos(w+1,2) = round(pos(w+1,2));
        pos(w+1,3) = round(pos(w+1,3));
        pos(w+1,4) = t;
        
        pos(w+2,1) = round(pos(w+2,1));
        pos(w+2,2) = round(pos(w+2,2));
        pos(w+2,3) = round(pos(w+2,3));
        pos(w+2,4) = t;
        
        x1n = find(x==pos(w,1));
        y1n = find(y==pos(w,2));
        z1n = find(z==pos(w,3));
        x2n = find(x==pos(w+1,1));
        y2n = find(y==pos(w+1,2));
        z2n = find(z==pos(w+1,3));
        x3n = find(x==pos(w+2,1));
        y3n = find(y==pos(w+2,2));
        z3n = find(z==pos(w+2,3));
        
        % collect measurements 
        v(w) = measurements(n,x1n,y1n,z1n);
        v(w+1) = measurements(n,x2n,y2n,z2n);
        v(w+2) = measurements(n,x3n,y3n,z3n);
        
        % collect noise associated with measurements
        alpha(w) = W_diag(n,x1n,y1n,z1n);
        alpha(w+1) = W_diag(n,x2n,y2n,z2n);
        alpha(w+2) = W_diag(n,x3n,y3n,z3n);
        
        % collect expectations from model
        c_theta(w) = ADEquation(theta,pos(w,1),pos(w,2),pos(w,3),t);
        c_theta(w+1) = ADEquation(theta,pos(w+1,1),pos(w+1,2),pos(w+1,3),t);
        c_theta(w+2) = ADEquation(theta,pos(w+2,1),pos(w+2,2),pos(w+2,3),t);
        
        robot1_pos(robot1_index,:) = [pos(w,1) pos(w,2) pos(w,3) t];
        robot2_pos(robot2_index,:) = [pos(w+1,1) pos(w+1,2) pos(w+1,3) t];
        robot3_pos(robot3_index,:) = [pos(w+2,1) pos(w+2,2) pos(w+2,3) t];

        t = t + dt;
        robot1_index = robot1_index +1;
        robot2_index = robot2_index +1;
        robot3_index = robot3_index +1;
    end

    % Optimize non linear least squares problem 
    W = diag(alpha);
    obj = 1/2*sum((W*(v - c_theta)).^2);
    %obj = 1/2*sum(((v - c_theta)).^2);
    lsqproblem = optimproblem("Objective",obj,'ObjectiveSense','min');
    %show(lsqproblem);
    options = optimoptions(lsqproblem,'MaxFunctionEvaluations',1e5, 'MaxIterations',1e4);
    [sol,fval] = solve(lsqproblem,xtheta,'Options',options);
    xtheta.theta = sol.theta;
    thetaf = sol.theta
    theta_history(theta_index,:) = sol.theta;
    theta_index = theta_index +1;
    
    % Evaluating Next informative M waypoints
    for w = 1:nr*(nw+2)
        H(w,:) = JacobianAD(thetaf,pos(w,1), pos(w,2),pos(w,3),pos(w,4)); 
        Joc(w,:) = JacobianAD(thetaf,optimpos(w,1), optimpos(w,2),optimpos(w,3),optimpos(w,4));  
    end
    
    ttt = t-2*dt;
    realpha1 = W_diag(find(time==ttt),find(x==pos(end-2*nr-2,1)),find(x==pos(end-2*nr-2,2)),find(x==pos(end-2*nr-2,3)));
    realpha2 = W_diag(find(time==ttt),find(x==pos(end-2*nr-1,1)),find(x==pos(end-2*nr-1,2)),find(x==pos(end-2*nr-1,3)));
    realpha3 = W_diag(find(time==ttt),find(x==pos(end-2*nr,1)),find(x==pos(end-2*nr,2)),find(x==pos(end-2*nr,3)));

    %M = transpose(H)*H + transpose(Joc)*Joc;
    M = transpose(H)*W*H + transpose(Joc)*diag(repmat([realpha1,realpha2,realpha3],1,length(Joc)/3))*Joc;
    mu = 0.2;
    obj2 = trace(M+mu*eye(length(M)));
    %obj2 = -log(det(M+mu*eye(length(M))));
    %obj2 = sqrt(sum(M.^2,'all'));
    lsqproblem2 = optimproblem("Objective",obj2,'ObjectiveSense','max');
    
    
    %Reinitializing future position data set from last position of the robots
    for w = 1:nr:nr*(nw+2)-2
        pos(w,1:4)= [pos(end-2*nr-2,1:3) ttt];
        pos(w+1,1:4)= [pos(end-2*nr-1,1:3) ttt];
        pos(w+2,1:4)= [pos(end-2*nr,1:3) ttt];
    end
    for w = 1:nr:nr*(nw+2)-2
        initialv(w,:)= initialv(end-2*nr-2,:);
        initialv(w+1,:)= initialv(end-2*nr-1,:);
        initialv(w+2,:)= initialv(end-2*nr,:);
    end
    for w = 1:nr:nr*(nw+2)-2
        initialu(w,:)= initialu(end-2*nr-2,:);
        initialu(w+1,:)= initialu(end-2*nr-1,:);
        initialu(w+2,:)= initialu(end-2*nr,:);
    end
    
    xoptimpos.optimpos =[pos,initialv, initialu];
    for w=1:nr:nr*(nw+2)-nr-2
        
        x1qrk = [optimpos(w,1) optimpos(w,2) optimpos(w,3) optimpos(w,5) optimpos(w,6) optimpos(w,7)];
        x1qrkp1 = [optimpos(w+nr,1) optimpos(w+nr,2) optimpos(w+nr,3) optimpos(w+nr,5) optimpos(w+nr,6) optimpos(w+nr,7)];
        
        x2qrk = [optimpos(w+1,1) optimpos(w+1,2) optimpos(w+1,3) optimpos(w+1,5) optimpos(w+1,6) optimpos(w+1,7)];
        x2qrkp1 = [optimpos(w+1+nr,1) optimpos(w+1+nr,2) optimpos(w+1+nr,3) optimpos(w+1+nr,5) optimpos(w+1+nr,6) optimpos(w+1+nr,7)];
        
        x3qrk = [optimpos(w+2,1) optimpos(w+2,2) optimpos(w+2,3) optimpos(w+2,5) optimpos(w+2,6) optimpos(w+2,7)];
        x3qrkp1 = [optimpos(w+2+nr,1) optimpos(w+2+nr,2) optimpos(w+2+nr,3) optimpos(w+2+nr,5) optimpos(w+2+nr,6) optimpos(w+2+nr,7)];
        
        uk1 = [optimpos(w,5) optimpos(w,6) optimpos(w,7) optimpos(w,8) optimpos(w,9) optimpos(w,10)];
        uk2 = [optimpos(w+1,5) optimpos(w+1,6) optimpos(w+1,7) optimpos(w+1,8) optimpos(w+1,9) optimpos(w+1,10)];
        uk3 = [optimpos(w+2,5) optimpos(w+2,6) optimpos(w+2,7) optimpos(w+2,8) optimpos(w+2,9) optimpos(w+2,10)];
        
        lsqproblem2.Constraints.(['robot1_' num2str(w)]) = x1qrkp1 == x1qrk + dt * uk1;
        lsqproblem2.Constraints.(['robot2_' num2str(w+1)]) = x2qrkp1 == x2qrk + dt * uk2;
        lsqproblem2.Constraints.(['robot3_' num2str(w+2)]) = x3qrkp1 == x3qrk + dt * uk3;
        lsqproblem2.Constraints.(['robot1_time_' num2str(w)]) = optimpos(w+nr,4)== optimpos(w,4) + dt;
        lsqproblem2.Constraints.(['robot2_time_' num2str(w+1)]) = optimpos(w+1+nr,4)== optimpos(w+1,4) + dt;
        lsqproblem2.Constraints.(['robot3_time_' num2str(w+2)]) = optimpos(w+2+nr,4)== optimpos(w+2,4) + dt;
        lsqproblem2.Constraints.(['time_' num2str(w)]) = optimpos(w,4) >= t-dt;
        lsqproblem2.Constraints.(['time_' num2str(w+1)]) = optimpos(w+1,4) >= t-dt;
        lsqproblem2.Constraints.(['time_' num2str(w+2)]) = optimpos(w+2,4) >= t-dt;
        if (w==1)
            lsqproblem2.Constraints.(['robot1_initial_' num2str(w)]) = x1qrk == [pos(w,1:3) initialv(w,1:3)];
            lsqproblem2.Constraints.(['robot2__initial_' num2str(w+1)]) =  x2qrk == [pos(w+1,1:3) initialv(w+1,1:3)];
            lsqproblem2.Constraints.(['robot3__initial_' num2str(w+2)]) = x3qrk == [pos(w+2,1:3) initialv(w+2,1:3)];
        end
        
    end
    %show(lsqproblem2);
    options = optimoptions(lsqproblem2,'MaxFunctionEvaluations',1e4, 'MaxIterations',1e4);
    [sol1,fval] = solve(lsqproblem2,xoptimpos,'Options',options);
    xoptimpos.optimpos = sol1.optimpos;
    pos(:,1:4) = sol1.optimpos(:,1:4);
    pos_database(kk:kk+17) = pos(:,1:4);
    initialv = sol1.optimpos(:,5:7);
    initialu = sol1.optimpos(:,8:10);
    kk = kk + 18;
    %to remove last 2 way points 
    t = t - 2*dt; 
    robot1_index = robot1_index -2;
    robot2_index = robot2_index -2;
    robot3_index = robot3_index -2;
end

%% Combined Theta Estimation
clc;
theta0 = [700 20 40 25 1 30];
nr = 3;
cum_theta_index =1;
cum_theta_history = zeros(5,6);
pos_database = zeros(18*5,4);
i = 1;
theta = optimvar('theta',6,1,'LowerBound',[0 0 0 0 0 0],'UpperBound',[2000 50 F F 50 Nt*dt]);
c_theta = optimexpr(5*18,1);
v = zeros(5*18,1);
alpha = zeros(5*18,1);
xtheta.theta = theta0;
for l = 1:nr:5*18
    pos_database(l,:) = robot1_pos(i,:);
    pos_database(l+1,:) = robot2_pos(i,:);
    pos_database(l+2,:) = robot3_pos(i,:);
    i = i +1;
end

for kk =18:18:length(pos_database)
    measurements_so_far = pos_database(1:kk,:);
    for w=1:nr:length(measurements_so_far)-2
       
        x1n = find(x==measurements_so_far(w,1));
        y1n = find(y==measurements_so_far(w,2));
        z1n = find(z==measurements_so_far(w,3));
        x2n = find(x==measurements_so_far(w+1,1));
        y2n = find(y==measurements_so_far(w+1,2));
        z2n = find(z==measurements_so_far(w+1,3));
        x3n = find(x==measurements_so_far(w+2,1));
        y3n = find(y==measurements_so_far(w+2,2));
        z3n = find(z==measurements_so_far(w+2,3));
        
        % collect measurements 
        v(w) = measurements(n,x1n,y1n,z1n);
        v(w+1) = measurements(n,x2n,y2n,z2n);
        v(w+2) = measurements(n,x3n,y3n,z3n);
        
        % collect noise associated with measurements
        alpha(w) = W_diag(n,x1n,y1n,z1n);
        alpha(w+1) = W_diag(n,x2n,y2n,z2n);
        alpha(w+2) = W_diag(n,x3n,y3n,z3n);
        
        % collect expectations from model
        c_theta(w) = ADEquation(theta,measurements_so_far(w,1),measurements_so_far(w,2),measurements_so_far(w,3),t);
        c_theta(w+1) = ADEquation(theta,measurements_so_far(w+1,1),measurements_so_far(w+1,2),measurements_so_far(w+1,3),t);
        c_theta(w+2) = ADEquation(theta,measurements_so_far(w+2,1),measurements_so_far(w+2,2),measurements_so_far(w+2,3),t);
        
        t = t + dt;

    end

    % Optimize non linear least squares problem 
    W = diag(alpha((1:length(measurements_so_far))));
    %obj = 1/2*sum((W*(v(1:length(measurements_so_far)) - c_theta(1:length(measurements_so_far)))).^2);
    obj = 1/2*sum(((v(1:length(measurements_so_far)) - c_theta(1:length(measurements_so_far)))).^2);
    
    %obj = 1/2*sum(((v(1:length(measurements_so_far)) - c_theta(1:length(measurements_so_far)))).^2);
    lsqproblem = optimproblem("Objective",obj,'ObjectiveSense','min');
    %show(lsqproblem);
    options = optimoptions(lsqproblem,'MaxFunctionEvaluations',1e5, 'MaxIterations',1e4);
    [sol,fval] = solve(lsqproblem,xtheta,'Options',options);
    xtheta.theta = sol.theta;
    cum_theta_history(cum_theta_index,:) = sol.theta;
    cum_theta_index = cum_theta_index +1;
end
cum_theta_history
%% Plots and Analysis
close all;

%Robot locations with time
ttime = 1:nr*(nw+2);
plot3(robot1_pos(:,1),robot1_pos(:,2),robot1_pos(:,3));
hold on
plot3(robot2_pos(:,1),robot2_pos(:,2),robot2_pos(:,3));
plot3(robot3_pos(:,1),robot3_pos(:,2),robot3_pos(:,3));
scatter3(robot1_pos(:,1),robot1_pos(:,2),robot1_pos(:,3))
scatter3(robot2_pos(:,1),robot2_pos(:,2),robot2_pos(:,3))
scatter3(robot3_pos(:,1),robot3_pos(:,2),robot3_pos(:,3))
grid on;
xlabel('x[m]');
ylabel('y[m]');
zlabel('z[m]');
print -dmeta

%% RMSE
close all;
theta_history2 = theta_history(1:5,:);
theta_true = [1000 12 2 5 0 0];
for i = 1:5
    RMSE(i)=sqrt(1/60*norm(theta_history2(i,:)-theta_true));
end
plot(RMSE)
ylabel('Root Mean Square Error');
xlabel('Iterations');
print -dmeta
%% Advection Diffusion 3D Equation
function [c] = ADEquation(theta,x,y,z,t)
Q = theta(1);
kx = theta(2);
ky = kx;
kz = 0.2113; %a*(z-z0);

x0 = theta(3);
y0 = theta(4);
z0 = theta(5);
t0 =  theta(6);

%wind velocity
u = 0.5; %datasample([0.5,0,-0.5,1],1); %(U,0,0)

c = Q/(8*(pi^1.5)*sqrt(kx*ky*kz)*(t-t0)^1.5) * ...
    exp(-((x-x0-u*(t-t0))^2)/(4*kx*(t-t0))-((y-y0)^2)/(4*ky*(t-t0)))*...
    (exp(-((z-z0)^2)/(4*kz*(t-t0)))+exp(-((z+z0)^2)/(4*kz*(t-t0))));
end


%% Jacobian 
function [Jc] = JacobianAD(theta,xx,yy,zz,tt)

Jc = [ (70368744177664*exp((theta(4) - yy)^2/(4*theta(2)*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)*(theta(6) - tt)))*(exp((theta(5) - zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500)) + exp((theta(5) + zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500))))/(3134689986450997*((2113*theta(2)^2)/10000)^(1/2)*(tt - theta(6))^(3/2)), - (9293072277962752*theta(1)*theta(2)*exp((theta(4) - yy)^2/(4*theta(2)*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)*(theta(6) - tt)))*(exp((theta(5) - zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500)) + exp((theta(5) + zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500))))/(1959181241531873125*((2113*theta(2)^2)/10000)^(3/2)*(tt - theta(6))^(3/2)) - (70368744177664*theta(1)*exp((theta(4) - yy)^2/(4*theta(2)*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)*(theta(6) - tt)))*(exp((theta(5) - zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500)) + exp((theta(5) + zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500)))*((theta(4) - yy)^2/(4*theta(2)^2*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)^2*(theta(6) - tt))))/(3134689986450997*((2113*theta(2)^2)/10000)^(1/2)*(tt - theta(6))^(3/2)), -(17592186044416*theta(1)*exp((theta(4) - yy)^2/(4*theta(2)*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)*(theta(6) - tt)))*(exp((theta(5) - zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500)) + exp((theta(5) + zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500)))*(2*theta(3) - theta(6) + tt - 2*xx))/(3134689986450997*theta(2)*((2113*theta(2)^2)/10000)^(1/2)*(tt - theta(6))^(5/2)), -(17592186044416*theta(1)*exp((theta(4) - yy)^2/(4*theta(2)*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)*(theta(6) - tt)))*(2*theta(4) - 2*yy)*(exp((theta(5) - zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500)) + exp((theta(5) + zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500))))/(3134689986450997*theta(2)*((2113*theta(2)^2)/10000)^(1/2)*(tt - theta(6))^(5/2)), (70368744177664*theta(1)*exp((theta(4) - yy)^2/(4*theta(2)*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)*(theta(6) - tt)))*((exp((theta(5) - zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500))*(2*theta(5) - 2*zz))/((2113*theta(6))/2500 - (2113*tt)/2500) + (exp((theta(5) + zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500))*(2*theta(5) + 2*zz))/((2113*theta(6))/2500 - (2113*tt)/2500)))/(3134689986450997*((2113*theta(2)^2)/10000)^(1/2)*(tt - theta(6))^(3/2)), (105553116266496*theta(1)*exp((theta(4) - yy)^2/(4*theta(2)*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)*(theta(6) - tt)))*(exp((theta(5) - zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500)) + exp((theta(5) + zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500))))/(3134689986450997*((2113*theta(2)^2)/10000)^(1/2)*(tt - theta(6))^(5/2)) - (70368744177664*theta(1)*exp((theta(4) - yy)^2/(4*theta(2)*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)*(theta(6) - tt)))*((2113*exp((theta(5) - zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500))*(theta(5) - zz)^2)/(2500*((2113*theta(6))/2500 - (2113*tt)/2500)^2) + (2113*exp((theta(5) + zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500))*(theta(5) + zz)^2)/(2500*((2113*theta(6))/2500 - (2113*tt)/2500)^2)))/(3134689986450997*((2113*theta(2)^2)/10000)^(1/2)*(tt - theta(6))^(3/2)) - (70368744177664*theta(1)*exp((theta(4) - yy)^2/(4*theta(2)*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)*(theta(6) - tt)))*(exp((theta(5) - zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500)) + exp((theta(5) + zz)^2/((2113*theta(6))/2500 - (2113*tt)/2500)))*((theta(4) - yy)^2/(4*theta(2)*(theta(6) - tt)^2) + (theta(3) - theta(6)/2 + tt/2 - xx)/(4*theta(2)*(theta(6) - tt)) + (theta(3) - theta(6)/2 + tt/2 - xx)^2/(4*theta(2)*(theta(6) - tt)^2)))/(3134689986450997*((2113*theta(2)^2)/10000)^(1/2)*(tt - theta(6))^(3/2))];
 
end
