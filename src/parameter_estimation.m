
run('../settings.m');
data = csvread(strcat('../output/',output_filename));
delta_omegas = data(:,2:5); % dw1,dw2,dw3,dw4
rates = data(:,6:9); % pdot,qdot,rdot,wdot

[n,~] = size(data);
dts = zeros(n-1,1);
for i = 2:n
    dts(i,1) = data(i,1) - data(i-1,1);
end
dt = mean(dts); clear dts;

% initial values
theta1_est = l*kt/(sqrt(2)*Ix0);
theta2_est = kq/Iz0;
theta3_est = kt/m0;
theta4_est = tau0;
Ix_est = (l*kt)/(sqrt(2)*theta1_est);
Iz_est = kq/theta2_est;
m_est = kt/theta3_est;
tau_est = theta4_est;

% iteration parameters

estimation_parameters = zeros(4,niter+1);
estimation_parameters(:,1) = [Ix_est;Iz_est;m_est;tau_est];

for j = 1:niter % iterate using same log data
    
    xs = zeros(8,n);
    xs(5,1) = theta1_est;
    xs(6,1) = theta2_est;
    xs(7,1) = theta3_est;
    xs(8,1) = theta4_est;

    P0 = 100.*diag([0.01,0.01,0.01,0.01,theta1_est^2,theta2_est^2,theta3_est^2,theta4_est^2]);
    P = P0;
    for i = 2:n
        %if rem(i-2,floor(n/4))==0
        %    P = P0;
        %end

        % control input and measurements
        dw = delta_omegas(i,:)'; % 4x1 matrix
        rate = rates(i,:)'; % 4x1 matrix

        % full state variables
        x = xs(:,i-1); % pdot,qdot,rdot,wdot,a,b,kz,kt,tau
        pdot = x(1);
        qdot = x(2);
        rdot = x(3);
        wdot = x(4);
        theta1 = x(5);
        theta2 = x(6);
        theta3 = x(7);
        theta4 = x(8);

        omega1bar = [1 -1 -1 1]*dw;
        omega2bar = [1 -1 1 -1]*dw;
        omega3bar = [-1 -1 1 1]*dw;
        omega4bar = [-1 -1 -1 -1]*dw;

        pdotcmd = theta1*omega1bar;
        qdotcmd = theta1*omega2bar;
        rdotcmd = theta2*omega3bar;
        wdotcmd = theta3*omega4bar;

        % time update
        dxdt = zeros(8,1);
        dxdt(1) = (pdotcmd-pdot)/theta4; % d/dt pdot
        dxdt(2) = (qdotcmd-qdot)/theta4; % d/dt qdot
        dxdt(3) = (rdotcmd-rdot)/theta4; % d/dt rdot
        dxdt(4) = (wdotcmd-wdot)/theta4; % d/dt wdot
        x = x + dt*dxdt;
        %A_pdot_theta = [ ...
        %    kt/(sqrt(2)*tau)*(dw(1)-dw(2)-dw(3)+dw(4)) 0 0 a/(sqrt(2)*tau)*(dw(1)-dw(2)-dw(3)+dw(4)) -(pdotcmd-pdot)/tau^2; ...
        %    kt/(sqrt(2)*tau)*(dw(1)-dw(2)+dw(3)-dw(4)) 0 0 a/(sqrt(2)*tau)*(dw(1)-dw(2)+dw(3)-dw(4)) -(qdotcmd-qdot)/tau^2; ...
        %    0 0 1/tau*(-dw(1)-dw(2)+dw(3)+dw(4)) 0 -(rdotcmd-rdot)/tau^2; ...
        %    0 kt/tau*(-dw(1)-dw(2)-dw(3)-dw(4)) 0 b/tau*(-dw(1)-dw(2)-dw(3)-dw(4)) -(wdotcmd-wdot)/tau^2];
        %A_pdot_pdot = eye(4).*(-1/tau);

        A11 = eye(4).*(-1/theta4);
        A12 = [ ...
            omega1bar/theta4 0 0 -(pdotcmd-pdot)/theta4^2; ...
            omega2bar/theta4 0 0 -(qdotcmd-qdot)/theta4^2; ...
            0 omega3bar/theta4 0 -(rdotcmd-rdot)/theta4^2; ...
            0 0 omega4bar/theta4 -(wdotcmd-wdot)/theta4^2];
        A = [A11 A12; ...
            zeros(4) zeros(4)];

        Phi = eye(8)+dt.*A;
        B = [ ...
            theta1/theta4.*[1 -1 -1 1]; ...
            theta1/theta4.*[1 -1 1 -1]; ...
            theta2/theta4.*[-1 -1 1 1]; ...
            theta3/theta4.*[-1 -1 -1 -1]; ...
            zeros(4)];
        Gamma = B.*dt;
        Q = eye(4).*(0.01^2);
        P = Phi*P*Phi'+Gamma*Q*Gamma';

        % measurement update
        z_meas = rate; % pdot,qdot,rdot,wdot
        z_pred = x(1:4);
        dz = z_meas - z_pred;
        H = [eye(4) zeros(4)];
        R = diag([1^2,1^2,1^2,5^2]);
        K = P*H'*inv(H*P*H'+R);
        x = x + K*dz;
        P = (eye(8)-K*H)*P;
        xs(:,i) = x;
    end

    theta1_est = xs(5,end);
    theta2_est = xs(6,end);
    theta3_est = xs(7,end);
    theta4_est = xs(8,end);
    
    Ix_est = (l*kt)/(sqrt(2)*theta1_est);
    Iz_est = kq/theta2_est;
    m_est = kt/theta3_est;
    tau_est = theta4_est;

    estimation_parameters(:,j+1) = [Ix_est;Iz_est;m_est;tau_est];
end % end of iteration

if show_plot_model_estimation
    figure;
    subplot(1,2,1);
    semilogy(estimation_parameters');
    grid();
    title('Estimation Parameters');
    legend('Ix','Iz','mass','tau');
    
    subplot(1,2,2)
    plot(diff(estimation_parameters'));
    grid();
    title('Convergence Rate');
    legend('Ix','Iz','mass','tau');
end

ActuationMatrix = [theta1_est.*[1 -1 -1 1];theta1_est.*[1 -1 1 -1]; ...
    theta2_est.*[-1 -1 1 1]; theta3_est.*[-1 -1 -1 -1];];
ActuationInverse = inv(ActuationMatrix);

disp('Estimated mass (kg):');
disp(m_est);
disp('Estimated moment of inertia (kg m^2):');
disp([Ix_est Ix_est Iz_est]);
disp('Estimated motor time constant (s):');
disp(tau_est);
disp('Estimated Actuation Inverse Matrix:');
disp(ActuationInverse);
