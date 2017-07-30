% compute quaternion and world acceleration in body frame

[n,~] = size(accelerometer);
quat = [ones(n,1) zeros(n,3)];
world_acceleration_bodyframe = [accelerometer(1,:); zeros(n-1,3)];
P = eye(3).*0.01;
for i = 2:n
    q = quat(i-1,:)'; % quaternion in previous timestep, 4x1 matrix
    w = gyro(i,:)'; % gyro in current timestep, 3x1 matrix
    a = accelerometer(i,:)'; % accelerometer in current timestep, 3x1 matrix
    
    % time update
    q = q + (dt*0.5) .* qmult2(q,[0;w]);
    Phi = eye(3) + dt.*[0 w(3) -w(2); -w(3) 0 w(1); w(2) -w(1) 0];
    Q = eye(3).*(sigma_gyro^2);
    Gamma = -eye(3).*dt;
    P = P + Phi*P*Phi' + Gamma*Q*Gamma';
    
    % measurement update
    a_pred = qcoordinatetransform([0;0;-1],q); % prediction of accelerometer measurement
    dz = a - a_pred; % measurement residual
    H = [0 -a_pred(3) a_pred(2); -a_pred(2) 0 -a_pred(1); -a_pred(2) a_pred(1) 0];
    R = eye(3).*(sigma_accelerometer^2);
    K = P*H'*inv(H*P*H'+R);
    alpha = K*dz;
    dq = 0.5 .* [-q(2) -q(3) -q(4); q(1) -q(4) q(3); q(4) q(1) -q(2); -q(3) q(2) q(1)]*alpha;
    q = q + dq;
    q = q / norm(q);
    P = (eye(3)-K*H)*P;

    quat(i,:) = q'; % estimated quaternion in current timestep, 1x4 matrix
    world_acceleration_bodyframe(i,:) = a + qcoordinatetransform([0;0;1],q);
end
