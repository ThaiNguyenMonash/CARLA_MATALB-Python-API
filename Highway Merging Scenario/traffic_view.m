classdef traffic_view < matlab.System
    % Untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties

    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            f = figure(1); 
            clf(f);
        end

        function [front_gap,back_gap ] = stepImpl(~,pos,waypoints)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states. 
            f = figure(1);
            
            x = pos(1);
            y = pos(2);
            
            h1 = subplot(2,2,1);
            plot(-1*waypoints(:,1),waypoints(:,2),'g--');
            hold on
            plot(-x,y,'bo','MarkerSize',2);
            axis([-120 20 -5 10]);
            grid on;
            title('Route');
            ylabel('y position');
            xlabel('x position');
            hold on

%             s = dir('targets.txt');
%             if s.bytes ~= 0
%                 targets= csvread('targets.txt');
%                 x_bar = targets(:,1);
%                 
%                 [x_bar,ia,~] = unique(x_bar,'rows');
%                 
%                 y_bar = targets(:,4);
%                 y_bar = y_bar(ia);
%                 
%                 bar(x_bar,y_bar);
%             else
%                 bar([],[]);
%             end
%             title('Velocity of Vehicle');
%             ylim([0 120])
%             ylabel('Velocity');
%             xlabel('Vehicle ID');

%             s = dir('targets.txt');
%             Column_names = ["ID" "X" "Y" "Velocity"];
%             if s.bytes ~= 0
%                 targets= csvread('targets.txt');
%                 uitable(f,'Data',targets,'ColumnName',Column_names,'Units',h1.Units,"Position",h1.Position);
%             else
%                 uitable(f,'Data',[],'ColumnName',Column_names,'Units',h1.Units,"Position",h1.Position);
%             end
%             h1.Visible = 'Off'; 

%             cla(h1);
%             delete(findall(gcf,'type','annotation'))
%             
%             s = dir('targets.txt');
%             if s.bytes ~= 0
%                 targets= csvread('targets.txt');
%                 ID = targets(:,1);
%                 
%                 [ID,ia,~] = unique(ID,'rows');
%                 
%                 X = targets(:,2);
%                 X = X(ia);
%                 Y = targets(:,3);
%                 Y = Y(ia);
%                 Velocity = targets(:,4);
%                 Velocity = Velocity(ia);
%                 
%             else
%                 ID = [];
%                 X = [];
%                 Y = [];
%                 Velocity = [];
%             end
%                 
%             T = table(ID,X,Y,Velocity);           
%             % Get the table in string form.
%             TString = evalc('disp(T)');
%             % Use TeX Markup for bold formatting and underscores.
%             TString = strrep(TString,'<strong>','\bf');
%             TString = strrep(TString,'</strong>','\rm');
%             TString = strrep(TString,'_','\_');
%             % Get a fixed-width font.
%             FixedWidth = get(0,'FixedWidthFontName');
%             % Output the table using the annotation command.
%             annotation(gcf,'Textbox','String',TString,'Interpreter','Tex',...
%                 'FontName',FixedWidth,'Units','Normalized','Position',h1.Position);
%             h1.Visible = 'Off'; 
            
            
            h2 = subplot(2,2,[3 4]); 
            cla(h2);
                        
            front_gap = 50;
            back_gap = 50;
                
            s = dir('targets.txt');
            if s.bytes ~= 0
                targets= csvread('targets.txt');
                
                for i=1:length(targets(:,2))
                    plot(-1*targets(i,2),targets(i,3),'o','MarkerFaceColor', 'g');
                    text(-1*targets(i,2),targets(i,3), string(targets(i,1)));
                    hold on
                end
                
                
                targets_x = -1*targets(:,2);
                targets_x = targets_x(targets(:,3)<20);
                                
                back_car = max(targets_x(targets_x<-x));
                front_car = min(targets_x(targets_x>-x));
                
                if ~isempty(back_car)
                    back_gap = min(back_gap,-x-back_car);
                end
                
                if ~isempty(front_car)
                    front_gap = min(front_gap,front_car + x);
                end
            end  

            txt = ['Front Gap: ' num2str(front_gap) ' Back Gap: ' num2str(back_gap)];
            plot(-x,y,'ro','MarkerFaceColor','r');
            hold on
            xline(-x + front_gap,'r','LineWidth',1);
            hold on
            xline(-x - back_gap,'b','LineWidth',1);
            hold on
            axis([-x-100 -x+100 -10 30]);
            pbaspect([6 1 1]);
            grid on;
            title(txt);
            ylabel('y position');
            xlabel('x position');
            
            h3 = subplot(2,2,2); 
            cla(h3);
            delete(findall(gcf,'type','annotation'))
            
            s = dir('targets.txt');
            if s.bytes ~= 0
                targets= csvread('targets.txt');
                ID = targets(:,1);
                
                [ID,ia,~] = unique(ID,'rows');
                
                X = targets(:,2);
                X = X(ia);
                Y = targets(:,3);
                Y = Y(ia);
                Velocity = targets(:,4);
                Velocity = Velocity(ia);
                
            else
                ID = [];
                X = [];
                Y = [];
                Velocity = [];
            end
                
            T = table(ID,X,Y,Velocity);           
            % Get the table in string form.
            TString = evalc('disp(T)');
            % Use TeX Markup for bold formatting and underscores.
            TString = strrep(TString,'<strong>','\bf');
            TString = strrep(TString,'</strong>','\rm');
            TString = strrep(TString,'_','\_');
            % Get a fixed-width font.
            FixedWidth = get(0,'FixedWidthFontName');
            % Output the table using the annotation command.
            annotation(gcf,'Textbox','String',TString,'Interpreter','Tex',...
                'FontName',FixedWidth,'Units','Normalized','Position',h3.Position);
            h3.Visible = 'Off'; 
        end
        
        function [front_gap,back_gap] = isOutputComplexImpl(~)
            
            front_gap = false;
            back_gap = false;

        end
        
        function [front_gap,back_gap] = getOutputSizeImpl(~)
            front_gap = [1 1];
            back_gap = [1 1];
        end
        
        function [front_gap,back_gap] = getOutputDataTypeImpl(~)
            front_gap = 'double';
            back_gap = 'double';
        end

        function [front_gap,back_gap] = isOutputFixedSizeImpl(~)
            front_gap = true;
            back_gap = true;
        end

        function resetImpl(~)
            % Initialize / reset discrete-state properties
        end
    end
end