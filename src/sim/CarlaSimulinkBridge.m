classdef CarlaSimulinkBridge < matlab.System
    % CARLA plant interface for Simulink co-simulation
    %
    % Inputs:
    %   steer    normalized steering command [-1, 1]
    %   throttle normalized throttle [0, 1]
    %   brake    normalized brake [0, 1]
    %
    % Outputs:
    %   y        lateral/global Y position from CARLA [m]
    %   psi      yaw/heading angle [rad]
    %   vx       longitudinal/global X velocity [m/s]

    properties (Nontunable)
        PythonFolder = 'G:\SIL_projects\carla_bridge'
    end

    properties (Access = private)
        bridge
    end

    methods (Access = protected)
       
        
        function setupImpl(obj)
        
            disp("SETUP RUNNING");
        
            py_folder = char(obj.PythonFolder);
        
            if count(py.sys.path, py_folder) == 0
                insert(py.sys.path, int32(0), py_folder);
            end
        
            py.importlib.invalidate_caches();
        
            mod = py.importlib.import_module('carla_bridge');
        
            obj.bridge = mod.CarlaBridge();
        
            disp("CARLA bridge created");
        
        end
    
    
        function [y, psi, vx] = stepImpl(obj, steer, throttle, brake)
        
            % Call Python bridge
            out = obj.bridge.step(double(steer), double(throttle), double(brake));
        
            % Unpack outputs
            y   = double(out{2});
            psi = double(out{3});
            vx  = double(out{4});
        end


        function resetImpl(obj)
            % Optional reset. Leave empty for now.
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
            num = 3;
        end

        function [name1, name2, name3] = getInputNamesImpl(~)
            name1 = 'steer';
            name2 = 'throttle';
            name3 = 'brake';
        end

        function [name1, name2, name3] = getOutputNamesImpl(~)
            name1 = 'y';
            name2 = 'psi';
            name3 = 'vx';
        end


        %% ----- Output propagation methods -----
        % These prevent underspecified signal dimension errors.

        function [o1, o2, o3] = getOutputSizeImpl(~)
            o1 = [1 1];
            o2 = [1 1];
            o3 = [1 1];
        end

        function [o1, o2, o3] = getOutputDataTypeImpl(~)
            o1 = 'double';
            o2 = 'double';
            o3 = 'double';
        end

        function [o1, o2, o3] = isOutputComplexImpl(~)
            o1 = false;
            o2 = false;
            o3 = false;
        end

        function [o1, o2, o3] = isOutputFixedSizeImpl(~)
            o1 = true;
            o2 = true;
            o3 = true;
        end


        %% ----- Force interpreted execution -----
        % Required because Python/CARLA API is not code-generation compatible.

        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end
    end
end