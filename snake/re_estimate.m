% re_estimate.m -- estimate Re from the commanded wave, no simulation
L = 0.15; d = 0.006; nu = 1e-6; n_nodes = 21; l_seg = L/20;
A = 0.465; lambda = 1.2; f = 1.825; loco = 2;      % gait to estimate
D10 = 1.353;                                        % displacement in 10 s (m)
U = D10/10;

n_bends = n_nodes - 2; s = (1:n_bends)/(n_bends + 1); k = 2*pi/lambda;
if loco == 1, A_s = A*ones(1, n_bends); else, A_s = A*(1 + 0.25*s)/1.25; end
nt = 400; dt = 1/(f*nt);
X = zeros(n_nodes, nt); Y = X;
for m = 1:nt
    kappa = A_s .* sin(2*pi*(m-1)/nt - k*s);
    ang = [0, cumsum(2*atan(kappa/2))];
    x = [0, cumsum(l_seg*cos(ang))]; y = [0, cumsum(l_seg*sin(ang))];
    x = x - mean(x); y = y - mean(y);                       % remove rigid translation
    th = atan2(y(end) - y(1), x(end) - x(1));               % rotate head-tail line onto x
    p = [cos(-th) -sin(-th); sin(-th) cos(-th)]*[x; y];
    X(:,m) = p(1,:)'; Y(:,m) = p(2,:)';
end
Vx = (circshift(X, -1, 2) - X)/dt; Vy = (circshift(Y, -1, 2) - Y)/dt;
Tx = X(3:end,:) - X(1:end-2,:); Ty = Y(3:end,:) - Y(1:end-2,:);
Tn = hypot(Tx, Ty); Tx = Tx./Tn; Ty = Ty./Tn;
vn = -Vx(2:end-1,:).*Ty + Vy(2:end-1,:).*Tx;                % sideways speed at interior nodes
Un = max(abs(vn(:)));
fprintf('U_n = %.3f m/s  ->  Re_d = %.0f\nU   = %.3f m/s  ->  Re_L = %.0f\n', Un, Un*d/nu, U, U*L/nu);