close all

addpath('util/')


mass = 1.535;
Ix = 0.029125;
Iy = 0.029125;
Iz = 0.055225;
I_diag = [Ix; Iy; Iz];

params = [mass; I_diag];

n_max = (1100)^2;
l_arm = 0.28;

kf = 5.84*1e-6;
thrust_max = 4.0 * kf * n_max;

km = 0.06*kf;

torque_max_x = abs(kf * l_arm * (sin(pi/4) + sin(3*pi/4) ) * n_max );
torque_max_y = abs(-kf * l_arm * (cos(3*pi/4) + cos(5*pi/4) ) * n_max );
torque_max_z = 2.0 * km * n_max;


Nsim = 4000;
delta = 0.005;

p0 = [0 0 0];
v0 = [0 0 0];
q0 = [1 0 0 0];
om0 = [0 0 0];
xk = [p0 v0 q0 om0]';

x_hst = zeros(13, Nsim+1);
x_hst(:,1) = xk;
u_hst = zeros(4, Nsim);
control_states = zeros(12,1);

vel = 1;

% y_traj = 0:vel*delta:Nsim*vel*delta;
% y_vel = 0.4*ones(1, Nsim+1);
% y_acc = 0*ones(1, Nsim+1);
y_traj = -cos(0:vel*delta:Nsim*delta) + ones(1, Nsim+1);
y_vel = sin(0:vel*delta:Nsim*delta);
y_acc = cos(0:vel*delta:Nsim*delta);



NED2FLU = [1 0 0; 0 -1 0; 0 0 -1]; % and vice versa

ref.p = NED2FLU*[0;0;0];
ref.v = NED2FLU*[0;0;0];
ref.a = NED2FLU*[0;0;0];
ref.yaw = -0; % this is rotated in NED


for i=1:Nsim

    ref.p = NED2FLU*[0; y_traj(i); 0];
    ref.v = NED2FLU*[0; y_vel(i); 0];
    ref.a = NED2FLU*[0; y_acc(i); 0];

    xk_ned = xk;
    xk_ned(1:3) = NED2FLU*xk(1:3);
    xk_ned(4:6) = NED2FLU*xk(4:6);
    xk_ned(8:10) = NED2FLU*xk(8:10);
    xk_ned(11:13) = NED2FLU*xk(11:13);
    [uk, control_states_next] = controller(xk_ned, control_states, ref);


    uk(1) = -uk(1)*thrust_max;
    uk(2:4) = NED2FLU*uk(2:4).*[torque_max_x; torque_max_y; torque_max_z];


    [~, xint] = ode45(@(t, states) dynamics(t, states, uk, params),[0 delta], xk);

    xk = xint(end,:)';


    control_states = control_states_next;

    x_hst(:,i+1) = xk;
    u_hst(:,i) = uk;


end

%%
close all
figure()
plot(u_hst(1, :))
title("Thrust")


figure()
plot(u_hst(2:end, :)')
title("Torques")

figure()
% plot(x_hst(3, :))
% title("z coordinate (FLU)")

plot3(x_hst(1, :),x_hst(2, :), x_hst(3, :))
axis equal
grid on



%%

start = 1;
skip = 10;

X = x_hst;

larm = 0.05;

% dimvideo = [1200 900];
% figure('units','pixels','Position', [100 50 dimvideo])
f = figure('units','pixels','Position', [0 0 1440 1080], Resize='off');

t = tiledlayout(f, 1,1);
t.TileSpacing = 'tight';
t.Padding = 'tight';
set(gcf,'color','w');

nexttile

% camerapanx = [linspace(80,90,200), 90*ones(1,length(X)-200)];
% camerapanx = [linspace(80,65,150), 65*ones(1,length(X)-150)];



v = VideoWriter("quad_anim");
v.FrameRate = 1/delta/skip;
open(v);



for n=start:skip:length(X)

    clf


    set(gcf,'color','w');


    
    fontSize_more=26;

    axesHandle=gca;
    set(axesHandle,'FontSize',fontSize_more);
    set(axesHandle,'TickLabelInterpreter','latex')

    xlabel('$x$ [m]','interpreter','latex','FontSize',fontSize_more)
    ylabel('$y$ [m]','interpreter','latex','FontSize',fontSize_more)
    zlabel('$z$ [m]','interpreter','latex','FontSize',fontSize_more)


    hold on
    grid on
    axis equal


%     view([-camerapanx(n), 15])

%     view([-camerapanx(n), 22.5])
    view([-70, 25])


    Rk = quat2rotm(X(7:10,n));

    pos_k = [X(1,n); X(2,n); X(3,n)];

    A = (pos_k + Rk*[larm;0;0]);
    B = (pos_k + Rk*[-larm;0;0]);
    C = (pos_k + Rk*[0;larm;0]);
    D =(pos_k + Rk*[0;-larm;0]);


    pos_b = (pos_k);




    line([A(1) B(1)], [A(2) B(2)], [A(3) B(3)], 'LineWidth',2.0, 'Color','k')
    line([C(1) D(1)], [C(2) D(2)], [C(3) D(3)], 'LineWidth',2.0, 'Color','k')

    plot3(X(1,start:skip:n), X(2,start:skip:n), X(3,start:skip:n),'LineWidth',2.0, 'Color', 'b')

    plot3(0*(max(1, n-10*skip):skip:n), y_traj(max(1, n-10*skip):skip:n), 0*(max(1, n-10*skip):skip:n),'LineWidth',2.0, 'Color', 'r')



    xlim([-0.5, 0.5])
    ylim([-0.5, 2.5])
% %     zlim([-2.3 0.1])
    zlim([-0.5, 0.5])


%     plot3(ref.p(1),ref.p(2),ref.p(3), 'Marker', "pentagram", "Color",'k', 'MarkerSize',10, 'MarkerFaceColor','k')



%     set(gcf,'units','pixels','position',[100 50 dimvideo])
    FF = getframe(gcf);
    writeVideo(v,FF);
    
    pause(delta)



end


close(v);
