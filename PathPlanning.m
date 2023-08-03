clear all
clc;
close all

%% import maps
M20 = readmatrix('20.txt');
M100 = readmatrix('100.txt');
M1000 = readmatrix('1000.txt');


%% Variables
M = M1000;
N = 1000;
t = 1000;
T = 1000;
start = [randi([1,N],1),randi([1,N],1)];

tic 
%% Initialize data matrix and alg_time checker
path_drone = [start M(start(1),start(2))];
pos = start;
T = T/1000;
run_time=0;
alg_time=toc;
M_OG=M;
recharge = [];
while run_time<t
    tic
    
    %% Apply convolution for to determine regions of interest
    convsize = floor(sqrt(N));
    if rem(convsize,2)==0
        convsize = convsize +1;
    end
    convM = conv2(M,(floor(sqrt(N))^(-2))*ones(floor(sqrt(N))),'same');


    %% obtain top percentage regions of interest for long path focus
    maxVal = max(convM,[],"all");
    [row,col] = find(convM>(1-1/N)*maxVal);

    ROI = [row,col];

    %% find nearest ROI for for main target
    dist_ROI = max(abs(ROI-ones(length(ROI),1)*pos),[],2);
    dist_ROI(dist_ROI~=0);
    idx = find(dist_ROI==min(dist_ROI));
    idx=idx(1);
    tar = ROI(idx,:);
       
    %obtain direction vector
    tar_vec = tar-pos;
    tar_vec_dir = tar_vec./abs(tar_vec);
    tar_vec_dir(isnan(tar_vec_dir))=0;
    
     
    % check time
    iteration_time=0;
    alg_time=alg_time+toc;
    if alg_time>=T || alg_time+iteration_time>=T
        break
    end
    %% MODE ROI FOCUS 
    %Fly to region of interest untill the 
    while sqrt(tar_vec(1)^2+tar_vec(2)^2)>=(convsize/2)
        tic
        %if vector is diagonal allow for three movements e.g. vector is
        %'NE' allow for 'NE', 'N' and 'E' step take the highest value.
        if tar_vec_dir(1)~=0 && tar_vec_dir(2)~=0
            possib = [M(pos(1)+tar_vec_dir(1),pos(2)) M(pos(1)+tar_vec_dir(1),pos(2)+tar_vec_dir(2)) M(pos(1),pos(2)+tar_vec_dir(2))];
            [~,best_posib] = max(possib);
            switch best_posib
                case 1
                    new_pos = [pos(1)+tar_vec_dir(1) pos(2)];
                case 2
                    new_pos = [pos(1)+tar_vec_dir(1) pos(2)+tar_vec_dir(2)];
                case 3
                    new_pos = [pos(1) pos(2)+tar_vec_dir(2)];
            end
        %if vector is straight 
        %e.g. vector is 'N' allow for 'NE', 'N' and 'NWE' step and take the highest value.
        else
            tar_vec_dir_left = tar_vec_dir;
            tar_vec_dir_right = tar_vec_dir;
            tar_vec_dir_left(tar_vec_dir==0)=-1;
            tar_vec_dir_right(tar_vec_dir==0)=1;
            possib = [M(pos(1)+tar_vec_dir_left(1),pos(2)+tar_vec_dir_left(2)) M(pos(1)+tar_vec_dir(1),pos(2)+tar_vec_dir(2)) M(pos(1)+tar_vec_dir_right(1),pos(2)+tar_vec_dir_right(2))];
            [~,best_posib] = max(possib);
            switch best_posib
                case 1
                    new_pos = [pos(1)+tar_vec_dir_left(1) pos(2)+tar_vec_dir_left(2)];
                case 2
                    new_pos = [pos(1)+tar_vec_dir(1) pos(2)+tar_vec_dir(2)];
                case 3
                    new_pos = [pos(1)+tar_vec_dir_right(1) pos(2)+tar_vec_dir_right(2)];
            end
        end

        % update position 
        recharge = [recharge sub2ind(size(M),pos(1),pos(2))];
        pos= new_pos;
        path_drone= [path_drone; pos M(pos(1),pos(2))];
        
        %update score on map
        M(pos(1),pos(2))=0;
        recharge(M(recharge)>=((1-1/N)*M_OG(recharge)))=[];
        M(recharge)=M(recharge)+(2/N)*M_OG(recharge);        
        
        run_time= run_time+1;
        
        %obtain new direction vector
        tar_vec = tar-pos;
        tar_vec_dir = tar_vec./abs(tar_vec);
        tar_vec_dir(isnan(tar_vec_dir))=0;
        
        if run_time>=t
            break
        end
        
        %algorithm time calculation and prediction
        iteration_time = toc;
        alg_time = alg_time+iteration_time;
        if alg_time>=T || alg_time+iteration_time>=T
            break
        end
        
    end
    
    
    %% MODE SCAN
    
    %select possibilities around the drone
    possib = [M(pos(1)+1,pos(2)-1) M(pos(1)+1,pos(2)) M(pos(1)+1,pos(2)+1); M(pos(1),pos(2)-1) M(pos(1),pos(2)) M(pos(1),pos(2)+1); M(pos(1)-1,pos(2)-1) M(pos(1)-1,pos(2)) M(pos(1)-1,pos(2)+1)];
    [best_val,best_posib] = max(possib,[],'all','linear');
    
    %Scan region of interest untill drone can not increase its mean
    while best_val>=mean(path_drone(:,3))
        tic
        switch best_posib
            case 1
                new_pos = [pos(1)+1 pos(2)-1];
            case 2
                new_pos = [pos(1) pos(2)-1];
            case 3
                new_pos = [pos(1)-1 pos(2)-1];
            case 4
                new_pos = [pos(1)+1 pos(2)];
            case 5
                "Drone thinks current location is best location";
            case 6
                new_pos = [pos(1)-1 pos(2)];
            case 7
                new_pos = [pos(1)+1 pos(2)+1];
            case 8
                new_pos = [pos(1) pos(2)+1];
            case 9
                new_pos = [pos(1)-1 pos(2)+1];
        end
        
        % update position 
        recharge = [recharge sub2ind(size(M),pos(1),pos(2))];
        pos= new_pos;
        path_drone= [path_drone; pos M(pos(1),pos(2))];
        
        %update score on map
        M(pos(1),pos(2))=0;
        recharge(M(recharge)>=((1-1/N)*M_OG(recharge)))=[];
        M(recharge)=M(recharge)+(2/N)*M_OG(recharge);        
        
        run_time= run_time+1;
        
        %dont fall of the grid
        xl = min(pos(1)-1,1);
        if pos(1)>=N
            xu = 0;
        else 
            xu = 1;
        end
        yl = min(pos(2)-1,1);
        if pos(2)>=N
            yu = 0;
        else 
            yu = 1;
        end
        
        %select possibilities around the drone
        possib = [M(pos(1)+xu,pos(2)-yl) M(pos(1)+xu,pos(2)) M(pos(1)+xu,pos(2)+yu); M(pos(1),pos(2)-yl) M(pos(1),pos(2)) M(pos(1),pos(2)+yu); M(pos(1)-xl,pos(2)-yl) M(pos(1)-xl,pos(2)) M(pos(1)-xl,pos(2)+yu)];
        [best_val,best_posib] = max(possib,[],'all','linear');
        
        if run_time>=t
            break
        end
        
        %algorithm time calculation and prediction
        iteration_time = toc;
        alg_time = alg_time+iteration_time;
        if alg_time>=T || alg_time+iteration_time>=T
            break
        end
    end
    
    
end
%%
figure 
plot(path_drone(:,1)',path_drone(:,2)','x')
axis([0 N 0 N])
grid on

"Total points:"+sum(path_drone(:,3))
"Mean of points:"+sum(path_drone(:,3))/run_time
"Mean of points/t:"+sum(path_drone(:,3))/t
"Mean of map:"+mean(M,"all")

