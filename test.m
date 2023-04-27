
clc; close all; clear;
load('data.mat');
scatter3(transformations(:,1,4),transformations(:,2,4),transformations(:,3,4))
figure;
stem(transformations(:,1,4))
figure;
stem(transformations(:,2,4))
figure;
stem(transformations(:,3,4))