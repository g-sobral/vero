% encapsulates messages to control staubli robot
%
% SockStaubli properties:
%otcp - internal matlab socket class (opened as client)
%
% SockStaubli methods:
%SockStaubli(ip,port,timeout) - contructor
%connect (obj) - try to open the connection. return status string. 
%send(obj,v) - sends velocity commands
%rcv(obj) - receives velocity and position
%send_rcv_reconnect(o,v) - sends, receives, and infinitely tries to reconnect
%close(obj) - closes the socket (must close before reconnecting)
classdef SockStaubli < handle
    properties
       otcp;
    end

    methods
        function SS = SockStaubli(ip,port,timeout)
            % timeout in seconds
            % ip and port of the remote robot host
            if nargin < 3
                timeout = 2;
            end
            SS.otcp = tcpip(ip,port);
            SS.otcp.Timeout = timeout;
            %SS.otcp.Terminator = ';'; % comment to use default
        end % function
        
        function ok = connect (obj)
            % function ok = connect (obj)
            % constructor does not open the connection.
            % return the same status string as tcpip socket class.
            %See also: tcpip
            fopen(obj.otcp);
            ok= obj.otcp.Status;
        end % function
        
        function send(obj,v)           
            % input v: flange velocities in mm/s and deg/s, vx,vy,vz,rx,ry,rz
            % does not run any checks.
            % the robot should answer with a data message (see rcv)
            fprintf(obj.otcp,'%.3f %.3f %.3f %.3f %.3f %.3f \n',v);
        end % function
        
        function [vels,here,empty,ok] = rcv_noblk(obj)
            % checks if there bytes in the reception buffers.
            % if yes, receives them
            % if not, return with empty = true and ok = true
            if obj.otcp.BytesAvailable <= 0             
                empty = true;
                ok = true;
                vels = zeros(6,1);
                here = zeros(6,1);
                return
            end % if
            while obj.otcp.BytesAvailable > 0             
                [vels,here,ok] = obj.rcv();
            end % while
            empty = false;   
        end % function
        
        function [vels,here,ok] = rcv(obj)
            % receives data. 
            % blocks until terminator character is received
            % then counts the numbers received. 
            % returns:
            % ok is true only if it has received the correct amount of data
            % vels: flange velocities in mm/s and deg/s, vx,vy,vz,rx,ry,rz
            % here: flange pose xyz, rot xyz, in mm and deg
            d = fscanf(obj.otcp,'%f');
            if length(d) == 12 
                ok = true;
                vels = reshape(d(1:6),6,1);
                here = reshape(d(7:12),6,1);
            elseif length(d) == 6
                ok = true;
                vels = zeros(6,1);
                here = reshape(d(1:6),6,1);
            else
                ok = false;
                vels = zeros(6,1);
                here = zeros(6,1);
            end
        end % function
        
        function [vels,here,ok] = send_rcv_reconnect(o,v)
        %function [vels,here,ok] = send_rcv_reconnect(o,v)
        %sends commands and then receives data. 
        % if it timeouts waiting for the data, it will try to reconnect,
        % and then to send again the command and receive data again.
        % it tries again forever, so do not use this if your program has
        % something else to do.
            ok = false;
            while ~ok
                o.send(v);
                [vels,here,ok] = o.rcv();
                if ~ok
                    o.close();
                    disp('rcv timeout: attempting to reconnect')
                    disp(o.connect());
                end
            end % while
        end % function
        
        function close(obj)           
            % close the socket
            fclose(obj.otcp);
        end % function
        
    end %methods
    
end % classdef
