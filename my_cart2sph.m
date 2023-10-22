function [azimuth,elevation,r] = my_cart2sph(x,y,z)
% matlab自带的坐标转化有问题
% https://www.mathworks.com/help/releases/R2022b/matlab/ref/cart2sph.html
% matlab计算的elevation角以x轴为基准，正负pi/2, 如下
% azimuth = atan2(y,x)
% elevation = atan2(z,sqrt(x.^2 + y.^2))
% r = sqrt(x.^2 + y.^2 + z.^2)
% https://zhuanlan.zhihu.com/p/34485962
  r = sqrt(x^2 + y^2 + z^2);
  elevation = acos(z/r);
  azimuth = atan2(y,x);
end