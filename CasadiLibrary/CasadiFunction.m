classdef CasadiFunction < handle
  %FUNCTION Summary of this class goes here
  %   Detailed explanation goes here
  
  properties (Access = private)
    fun
    casadiFun
    numericOutputIndizes
    numericOutputValues
    
    compiled
    name = 'test'
  end
  
  methods
    
    function self = CasadiFunction(inputFunction,jit)
      % CasadiFunction(function,jit)
      % CasadiFunction(userFunction,jit)
      
      self.fun = inputFunction;
      
      if nargin ==1
        jit = false;
      end
      
      nInputs = length(inputFunction.inputs);
      if isa(inputFunction,'UserFunction')
        for k=1:nInputs
          CasadiLib.setSX(inputFunction.inputs{k});
        end
        inputs = inputFunction.inputs;
      else
        inputs = cell(1,nInputs);
        for k=1:nInputs
          inputs{k} = casadi.SX.sym('in',inputFunction.inputs{k}.size);
        end
      end
      
      outputs = cell(1,inputFunction.nOutputs);
      [outputs{:}] = inputFunction.functionHandle(inputs{:});
      
      if isa(inputFunction,'UserFunction')
        for k=1:nInputs
          inputs{k} = inputs{k}.flat;
        end

        outputsCasadi = cell(1,inputFunction.nOutputs);
        for k=1:inputFunction.nOutputs
          outputsCasadi{k} = outputs{k}.flat;
        end
        outputs = outputsCasadi;
      end
      
      % check for numieric/constant outputs
      self.numericOutputIndizes = logical(cellfun(@isnumeric,outputs));
      self.numericOutputValues = outputs(self.numericOutputIndizes);
      
      self.casadiFun = casadi.Function('fun',inputs,outputs,struct('jit',jit));
      self.casadiFun.expand();
      if jit
        delete jit_tmp.c
      end
    end
    
    function varargout = evaluate(self,varargin)
      
      global exportDir
      
      if self.compiled
        cFilePath = fullfile(exportDir,[self.name]);
        
        opts = struct;
        opts.flags = '-O3';
%         funImporter = casadi.Importer(cFilePath, 'clang',opts);
        externalCasadiFun = casadi.external('fun', [cFilePath '.mexw64']);
        
        varargout = cell(1,self.fun.nOutputs);
        [varargout{:}] = externalCasadiFun(varargin{:});

        % replace numerical outputs
        varargout(self.numericOutputIndizes) = self.numericOutputValues;
        
      else
        % evaluate casadi function
        varargout = cell(1,self.fun.nOutputs);
        [varargout{:}] = self.casadiFun(varargin{:});

        % replace numerical outputs
        varargout(self.numericOutputIndizes) = self.numericOutputValues;
      end
    end
    
    function compile(self,name)
      
      self.name = name;
      
      global exportDir
      currentDir = pwd;
      cd(exportDir)
      
      codeGenerator = casadi.CodeGenerator(self.name,struct('mex',true));
      codeGenerator.add(self.casadiFun);
      
      MAX_NUM_SEEDS = 1;
      
      for i=1:MAX_NUM_SEEDS
        fwd    = self.casadiFun.forward(i);
        adj    = self.casadiFun.reverse(i);
        fwd_over_reverse = adj.forward(i);

        codeGenerator.add(fwd)
        codeGenerator.add(adj)
        codeGenerator.add(fwd_over_reverse)
      end
      
      cFilePath = fullfile(exportDir,[self.name '.c']);
      codeGenerator.generate;
      cd(currentDir)
      
      % compile generated code as mex file
      mex(cFilePath, '-largeArrayDims')
      self.compiled = true;
    end

%     function compile(self)
%       global exportDir
%       currentDir = pwd;
%       cd(exportDir)
%       opts = struct;
%       opts.mex = true;
%       cFilePath = fullfile(exportDir,[self.name '.c']);
%       self.fun.generate([self.name '.c'],opts)
%       cd(currentDir)
%       
%       % compile generated code as mex file
%       mex(cFilePath,'-largeArrayDims')
%       self.compiled = true;
%     end
    
  end
end

