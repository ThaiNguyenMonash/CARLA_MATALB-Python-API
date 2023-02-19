                
% 
% A = [1 2 3 4 5 6 7];
% if min(A) > 1
%     next_id = 1;
% else
%     index = find(histcounts(A)== 0);
%     if isempty(index)
%         next_id = max(A) + 1;
%     else
%         next_id = index(1);
%     end
% end
%         
% disp(next_id)
% 
% B = [0 1; 2 3];
% C = ["Id" "Vel"];
% fig = uifigure;
% for i = 1:100
%     B = B +1 ;
%     uitable(fig,'Data',B,'ColumnName',C);
%     pause(1) 
% end

% M = [1 2; 4 5;1 2; 1 3; 6 7];
% [X,ia,ic] = unique(M(:,1),'rows');
% 
% disp("X")
% disp(X)
% 
% 
% Y = M(:,2);
% Y = Y(ia);
% 
% disp("Y")
% disp(Y)
% 
% f = figure(1); 
% h1 = subplot(2,1,1);
% s = dir('targets.txt');
% Column_names = ["ID" "X" "Y" "Velocity"];
% if s.bytes ~= 0
%     targets= csvread('targets.txt');
%     uitable('Data',targets,'ColumnName',Column_names,'Units',h1.Units,"Position",h1.Position);
%     disp("Hi")
% else
%     uitable('Data',[],'ColumnName',Column_names,'Units',h1.Units,"Position",h1.Position);
% end
% h1.Visible = 'Off'; 

% stringy = "(" + num2str(255) + "," + num2str(255) + "," + num2str(255) + ")"

% 
% x = [-100 -50 -20 -30 -90];
% v = [13 54 52 63 89]; 
% output = zeros(length(x),3);
% output(:,1) = (x == min(x));
% 
% lead_pos_id = zeros(1,length(x));
% for i = 1:length(x)
%     if output(i,1)
%         lead_pos_id(i) = i;
%     else
%         y = (x(i)-x);
%         y(y<=0) = NaN;
%         [~,ind] = min(y);
%         lead_pos_id(i) = ind;
%     end
% end
% 
% output(:,2) = x(lead_pos_id);
% output(:,3) = v(lead_pos_id);
% 
% myList = pyrun("l = ['A', 'new', 'list']", "l");

a = [0 0 0 0 NaN]
size(a)




