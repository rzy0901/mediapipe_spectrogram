function [X, Y, Z] = ellipsoid2P_transformed(P1, P2, a, b, c, N)
Cntr = (P1+P2)/2;  % ellipsoid center
Lc = norm(P2-P1);

% the axis defined by: P1+V*[0:Lc]
V = (P1-P2)/Lc;   %normalized cylinder's axis-vector;
U = rand(1,3);     %linear independent vector
U = V-U/(U*V');    %orthogonal vector to V
U = U/sqrt(U*U');  %orthonormal vector to V
W = cross(V,U);    %vector orthonormal to V and U
W = W/sqrt(W*W');  %orthonormal vector to V and U 
R = [U.', W.', V.'];
T = eye(4);
T(1:3, 1:3) = R;
T(1:3, 4) = Cntr;

% generate the ellipsoid at (0,0,0)
[Xc,Yc,Zc] = ellipsoid(0,0,0,a,b,c,N);
X = zeros(size(Xc));
Y = zeros(size(Yc));
Z = zeros(size(Zc));
for ii = 1:size(Xc,1)
for jj = 1:size(Xc,2)
    local_coords = [Xc(ii,jj);Yc(ii,jj);Zc(ii,jj);1];
    global_coords = T*local_coords;
    X(ii,jj) = global_coords(1);
    Y(ii,jj) = global_coords(2);
    Z(ii,jj) = global_coords(3);
end
end
end