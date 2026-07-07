classdef CarlaSimulinkBridge < matlab.System
    % CARLA plant interface for Simulink co-simulation
    %
    % Inputs:
    %   steer    normalized steering command [-1, 1]
    %   throttle normalized throttle [0, 1]
    %   brake    normalized brake [0, 1]
    %
    % Outputs:
    %   x         longitudinal position, origin-zeroed [m]
    %   y         lateral position, origin-zeroed [m]
    %   psi       yaw angle [rad]
    %   vx        longitudinal speed [m/s]
    %   Xd        reference longitudinal position [m]
    %   Yd        reference lateral position (smoothstep lane change) [m]
    %   psi_d     reference heading [rad]
    %   psi_dot_d reference yaw rate [rad/s]
    %   psi_dd_d  reference yaw acceleration [rad/s^2]
    %   psi_dot   measured yaw rate from CARLA angular velocity [rad/s]
    %   Y_dot     measured global lateral velocity [m/s]

    properties (Nontunable)
        PythonFolder = 'G:\Control_Research\Autonomy_Mining_Dump_Truck\src\sim'
    end

    properties (Access = private)
        bridge
    end

    methods (Access = protected)
       
        
        function setupImpl(obj)
        
            disp("SETUP RUNNING");

            py_folder   = char(obj.PythonFolder);
            bridge_file = fullfile(py_folder, 'carla_bridge.py');
            disp(['Bridge file: ' bridge_file]);

            % Python accepts forward slashes on Windows; avoids escape issues
            bf_py = string(strrep(bridge_file, '\', '/'));

            % Load the exact .py file by absolute path, bypassing sys.path
            % and any stale __pycache__ bytecode.
            pyrun( ...
                "import importlib.util, sys; " + ...
                "sys.modules.pop('carla_bridge', None); " + ...
                "spec = importlib.util.spec_from_file_location('carla_bridge', '" + bf_py + "'); " + ...
                "m = importlib.util.module_from_spec(spec); " + ...
                "sys.modules['carla_bridge'] = m; " + ...
                "spec.loader.exec_module(m)");

            % Retrieve from sys.modules (path search bypassed)
            mod = py.importlib.import_module('carla_bridge');
        
            obj.bridge = mod.CarlaBridge();
        
            disp("CARLA bridge created");
        
        end
        
        function [x, y, psi, vx, Xd, Yd, psi_d, psi_dot_d, psi_dd_d, psi_dot, Y_dot] = stepImpl(obj, steer, throttle, brake)
        
            out  = obj.bridge.step(double(steer), ...
                                   double(throttle), ...
                                   double(brake));
            vals = double(py.array.array('d', out));

            x         = vals(1);  y         = vals(2);  psi       = vals(3);
            vx        = vals(4);  Xd        = vals(5);  Yd        = vals(6);
            psi_d     = vals(7);  psi_dot_d = vals(8);  psi_dd_d  = vals(9);
            psi_dot   = vals(10); Y_dot     = vals(11);
        
        end



        function resetImpl(obj)
            % Optional reset.
        end

        function flag = isInputDirectFeedthroughImpl(~, ~)
            % CARLA applies the input and ticks the world inside step(),
            % so the returned state is k+1, not a function of the current
            % input at time k. This breaks the algebraic loop.
            flag = false;
        end

        function sts = getSampleTimeImpl(obj)
            % Lock the block to CARLA's fixed simulation timestep.
            % Without this, Simulink may call step() at its own fundamental
            % rate (e.g. 1 ms), ticking CARLA far faster than intended.
            sts = createSampleTime(obj, 'Type', 'Discrete', ...
                                        'SampleTime', 0.01);
        end
        
        function releaseImpl(obj)
        
            if ~isempty(obj.bridge)
                try
                    obj.bridge.close();
                    disp("CARLA vehicle destroyed");
                catch
                    disp("Error destroying CARLA vehicle");
                end
            end
        
        end

        %% ----- Port definitions -----

        function num = getNumInputsImpl(~)
            num = 3;
        end

        function num = getNumOutputsImpl(~)
            num = 11;
        end

        function [name1, name2, name3] = getInputNamesImpl(~)
            name1 = 'steer';
            name2 = 'throttle';
            name3 = 'brake';
        end

        
        function [name1,name2,name3,name4,name5,name6,name7,name8,name9,name10,name11] = getOutputNamesImpl(~)
        
            name1  = 'x';
            name2  = 'y';
            name3  = 'psi';
            name4  = 'vx';
            name5  = 'Xd';
            name6  = 'Yd';
            name7  = 'psi_d';
            name8  = 'psi_dot_d';
            name9  = 'psi_dd_d';
            name10 = 'psi_dot';
            name11 = 'Y_dot';
        
        end

        %% ----- Output propagation methods -----
        % These prevent underspecified signal dimension errors.

        function [o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11] = getOutputSizeImpl(~)
            o1=[1 1];o2=[1 1];o3=[1 1];o4=[1 1];o5=[1 1];
            o6=[1 1];o7=[1 1];o8=[1 1];o9=[1 1];o10=[1 1];o11=[1 1];
        end

        function [o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11] = getOutputDataTypeImpl(~)
            o1='double';o2='double';o3='double';o4='double';o5='double';
            o6='double';o7='double';o8='double';o9='double';o10='double';o11='double';
        end

        function [o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11] = isOutputComplexImpl(~)
            o1=false;o2=false;o3=false;o4=false;o5=false;
            o6=false;o7=false;o8=false;o9=false;o10=false;o11=false;
        end

        function [o1,o2,o3,o4,o5,o6,o7,o8,o9,o10,o11] = isOutputFixedSizeImpl(~)
            o1=true;o2=true;o3=true;o4=true;o5=true;
            o6=true;o7=true;o8=true;o9=true;o10=true;o11=true;
        end
        
        %% ----- Force interpreted execution -----
        % Required because Python/CARLA API is not code-generation compatible.

        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end
    end
end