clc; clear; close all;
names = ["push_pull","beckoned","rub_finger"];
for ii = 1:1:length(names)
    files  = dir(fullfile('../data/all/'+names(ii),'*.jpg'));
    for jj = 1:1:length(files)
        [~,base,~] = fileparts(files(jj).name);
        jpg_path = fullfile(files(jj).folder,files(jj).name);
        img = imread(jpg_path);
        img = imresize(img,[60,80]);
%         imshow(img)
        save(fullfile(files(jj).folder,base+".mat"),'img','-v7.3')
        reszie_jpg_path = fullfile(files(jj).folder+"_resize",files(jj).name); 
        imwrite(img,reszie_jpg_path);
    end
end

