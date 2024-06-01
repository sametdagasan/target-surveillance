function connProb = connections(probs)
load('connections.mat');
n = nchoosek(4,2);  
arr = (1:2^n)-1;
arr2 = dec2base(arr,2);
links = zeros(64,6);
for i = 1:size(arr2,1)
    for j = 1:size(arr2,2)
        links(i,j) = str2double(arr2(i,j));
    end
end

connProb = 0;
% temp_prob = 1;
% for i = 1:size(arr2,1)
%    for j = 1:size(arr2,2)
%        if (array(i,j) == 1)
%            temp_prob = temp_prob*x(j);
%        else
%            temp_prob = temp_prob*x_inv(j);
%        end
%        p = p + temp_prob;
%    end
% end
for i = 1:size(links,1)
    res(i) = prod(probs(links(i,:) ~= 0))*prod(1-probs(links(i,:) ~= 1));
    connProb = connProb + res(i)*results(i);
end
end