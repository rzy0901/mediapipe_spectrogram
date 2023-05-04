load("data.mat")
R_list = transformations(:,1:3,1:3);
for ii = 1:length(R_list)
    squeeze(R_list(ii,:,:))
end
T_list = transformations(:,1:3,4);

%% 卡尔曼滤波器
function xhat =  kalman(D, Q, R)
    xhat = zeros(size(D)); 
    P= zeros(size(D));
    xhatminus = zeros(size(D));
    Pminus = zeros(sz);
    K = zeros(sz);
    xhat(1,:) = D(1,:); 
    P(1,:) =0; % 误差方差为1

    for k = 2:length(D)
        % 时间更新（预测）
        % 用上一时刻的最优估计值来作为对当前时刻的预测
        xhatminus(k) = xhat(k-1);
        % 预测的方差为上一时刻最优估计值的方差与过程方差之和
        Pminus(k) = P(k-1)+Q;
        % 测量更新（校正）
        % 计算卡尔曼增益
        K(k) = Pminus(k)/( Pminus(k)+R );
        % 结合当前时刻的测量值，对上一时刻的预测进行校正，得到校正后的最优估计。该估计具有最小均方差
        xhat(k) = xhatminus(k)+K(k)*(D(k)-xhatminus(k));
        % 计算最终估计值的方差
        P(k) = (1-K(k))*Pminus(k);
    end
end