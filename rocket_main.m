function rocket_main(varargin)

params.verbose = 1;
params.save = false;
params.sweep = false;
params.target_apogee = 200000;
params.filename = 'rocket_solution';

for i = 1:2:length(varargin)
    switch lower(varargin{i})
        case 'verbose', params.verbose = varargin{i+1};
        case 'save', params.save = varargin{i+1};
        case 'sweep', params.sweep = varargin{i+1};
        case 'apogee', params.target_apogee = varargin{i+1};
        case 'filename', params.filename = varargin{i+1};
    end
end

if params.verbose
    fprintf('\n6-DOF Rocket Trajectory Optimization\n');
    fprintf('Target apogee: %.1f km\n', params.target_apogee/1000);
end

p = getRocketParameters();

guidance0 = [85*pi/180; 0.1; -0.001];

options = optimset('Display','off','TolX',1e-6,'MaxFunEvals',100);
guidance_sol = fsolve(@(g) apogeeError(g,p,params), guidance0, options);

if params.verbose
    fprintf('a=%.2f deg, b=%.4f deg/s, c=%.6f deg/s^2\n', ...
        guidance_sol(1)*180/pi, guidance_sol(2)*180/pi, guidance_sol(3)*180/pi);
end

[t,y] = ode45(@(t,y) rocketDynamics(t,y,p,guidance_sol), [0 300], getInitialConditions(p));

plotResults(t,y,guidance_sol,p,params);

if params.save
    save(['outputs/' params.filename '.mat'],'t','y','guidance_sol');
end

end


function miss = apogeeError(guidance,p,params)
[t,y] = ode45(@(t,y) rocketDynamics(t,y,p,guidance), [0 300], getInitialConditions(p));
miss = max(y(:,3)) - params.target_apogee;
end


function dy = rocketDynamics(t,y,p,guidance)

r = y(1:3);
v = y(4:6);
q = y(7:10);
omega = y(11:13);
m = y(14);

q = q / norm(q);

h = r(3);
V = norm(v);

rho = p.rho0 * exp(-h/p.H);

v_body = quatRotate(quatConj(q), v);
drag_body = -0.5 * rho * V^2 * p.Cd * p.Aref * (v_body/V);
drag = quatRotate(q, drag_body);

g_mag = 3.986e14 / (norm(r)^2);
g = -g_mag * (r/norm(r));

thrust = quatRotate(q, [p.Thrust;0;0]);

if t < 200
    theta_cmd = guidance(1) + guidance(2)*t + guidance(3)*t^2;
else
    theta_cmd = guidance(1) + guidance(2)*200 + guidance(3)*200^2;
end

pitch = 2*atan2(q(2), q(1));
tau = [0; p.Kp*(theta_cmd - pitch) - p.Kd*omega(2); 0];

dy = zeros(14,1);
dy(1:3) = v;
dy(4:6) = (thrust + drag)/m + g;

Omega = [0, -omega(1), -omega(2), -omega(3);
         omega(1), 0, omega(3), -omega(2);
         omega(2), -omega(3), 0, omega(1);
         omega(3), omega(2), -omega(1), 0];

dy(7:10) = 0.5 * Omega * q;
dy(11:13) = p.I \ (tau - cross(omega, p.I*omega));
dy(14) = -p.mdot;

end


function y0 = getInitialConditions(p)
y0 = zeros(14,1);
y0(7) = 1;
y0(14) = p.m0;
end


function v_rot = quatRotate(q,v)
qv = [0;v];
q_conj = [q(1); -q(2:4)];
qv_rot = quatMultiply(quatMultiply(q,qv), q_conj);
v_rot = qv_rot(2:4);
end


function q_conj = quatConj(q)
q_conj = [q(1); -q(2:4)];
end


function q_out = quatMultiply(q1,q2)
q_out = [q1(1)*q2(1) - dot(q1(2:4),q2(2:4));
         q1(1)*q2(2:4) + q2(1)*q1(2:4) + cross(q1(2:4),q2(2:4))];
end


function p = getRocketParameters()

p.m0 = 50000;
p.mdot = 200;
p.Thrust = 1e6;
p.Isp = 300;

p.Cd = 0.5;
p.Aref = 10;
p.rho0 = 1.225;
p.H = 8500;

p.I = diag([1e6 1e6 1e5]);

p.Kp = 1e6;
p.Kd = 1e5;

end


function plotResults(t,y,~,~,params)

figure(1)
plot(t, y(:,3)/1000,'LineWidth',2)
xlabel('Time (s)'), ylabel('Altitude (km)'), grid on

figure(2)
V = sqrt(sum(y(:,4:6).^2,2));
plot(t, V/1000,'LineWidth',2)
xlabel('Time (s)'), ylabel('Velocity (km/s)'), grid on

figure(3)
pitch = 2*atan2(y(:,8), y(:,7));
plot(t, pitch*180/pi,'LineWidth',2)
xlabel('Time (s)'), ylabel('Pitch (deg)'), grid on

figure(4)
plot3(y(:,1)/1000, y(:,2)/1000, y(:,3)/1000,'LineWidth',2)
xlabel('X (km)'), ylabel('Y (km)'), zlabel('Z (km)')
grid on, view(45,30)

if params.save
    for i = 1:4
        saveas(i, ['outputs/figure' num2str(i) '.png']);
    end
end

end