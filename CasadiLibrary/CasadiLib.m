classdef CasadiLib
  %CASADILIB Casadi functionality for Vars and Models
  
  properties

  end
  
  methods(Static)
    
    function nlpStruct = loadNLP(name)
      global exportDir
      
      curDir = pwd;
      cd(fullfile(exportDir,name));
      
      rmpath(genpath(exportDir));
      addpath(exportDir);
      addpath(fullfile(exportDir,name));
      
      load('nlpStruct','nlpStruct');
      
      cd(curDir);
      
    end
    
    function exportSolver(name,solver)
      global exportDir
      
      curDir = pwd;
      cd(fullfile(exportDir,name));
      
      solver.generate_dependencies('nlp.c',struct('mex',true));
      mex('nlp.c', '-largeArrayDims');
      
      gSparsity = nlp('nlp_jac_g');
      gSparsity(find(gSparsity)) = 1;
      
      HSparsity = nlp('nlp_hess_l');
      HSparsity(find(HSparsity)) = 1;
      
      cd(curDir);
      
      nlpStruct = struct;
      nlpStruct.name = name;
      nlpStruct.f = @(x) nlp('nlp_f',x);
      nlpStruct.g = @(x) nlp('nlp_g',x);
      nlpStruct.fGrad = @(x) nlp('nlp_grad_f',x);
      nlpStruct.gJac = @(x) nlp('nlp_jac_g',x);
      nlpStruct.H = @(x,lam) nlp('nlp_hess_l',x,lam);
      nlpStruct.gSparsity = gSparsity;
      nlpStruct.HSparsity = HSparsity;
      
      save('nlpStruct','nlpStruct');
      cd(curDir);
      
      
    end
    
    function exportNLP(name,x,f,g,lbg,ubg)
      global exportDir
      
      fGrad = gradient(f,x);
      gJac = jacobian(g,x);
      lam = casadi.MX.sym('lam', g.sparsity());
      H = hessian(f+dot(g,lam),x);
      
      fFun = casadi.Function('f',{x},{f});
      gFun = casadi.Function('g',{x},{g});
      fGradFun = casadi.Function('fGrad',{x},{fGrad});
      gJacFun = casadi.Function('gJac',{x},{gJac});
      hessFun = casadi.Function('H',{x,lam},{H});
      
      [crsRP,crsCI]=gJac.sparsity().get_crs; % returns row pointer and column index
      crsRP = crsRP+1; crsCI = crsCI+1;   % 0 index shift to 1 for matlab
      val = ones(1,length(crsCI));        % one vector with number of non-zeros
      gSparsity = csr_to_sparse(crsRP,crsCI,val);

      [crsRP,crsCI]=H.sparsity().get_crs; % returns row pointer and column index
      crsRP = crsRP+1; crsCI = crsCI+1;   % 0 index shift to 1 for matlab
      val = ones(1,length(crsCI));        % one vector with number of non-zeros
      HSparsity = csr_to_sparse(crsRP,crsCI,val);
    
      curDir = pwd;
      [~,~] = mkdir(fullfile(exportDir,name));
      cd(fullfile(exportDir,name));  
      
      fFun.generate(    'fFunMEX.c',    struct('mex',true));
      gFun.generate(    'gFunMEX.c',    struct('mex',true));
      fGradFun.generate('fGradFunMEX.c',struct('mex',true));
      gJacFun.generate( 'gJacFunMEX.c', struct('mex',true));
      hessFun.generate( 'hessFunMEX.c', struct('mex',true));
      
      mex('fFunMEX.c',    '-largeArrayDims');
      mex('gFunMEX.c',    '-largeArrayDims');
      mex('fGradFunMEX.c','-largeArrayDims');
      mex('gJacFunMEX.c', '-largeArrayDims');
      mex('hessFunMEX.c', '-largeArrayDims');
      
      nlpStruct = struct;
      nlpStruct.name = name;
      nlpStruct.f = @(x) fFunMEX('f',x);
      nlpStruct.g = @(x) gFunMEX('g',x);
      nlpStruct.fGrad = @(x) fGradFunMEX('fGrad',x);
      nlpStruct.gJac = @(x) gJacFunMEX('gJac',x);
      nlpStruct.H = @(x,lam) hessFunMEX('H',x,lam);
      nlpStruct.gSparsity = gSparsity;
      nlpStruct.HSparsity = HSparsity;
      nlpStruct.lbg = lbg;
      nlpStruct.ubg = ubg;
      
      save('nlpStruct','nlpStruct');
      cd(curDir);
      
    end
    
    function setSXModel(model)
      CasadiLib.setSX(model.state)
      CasadiLib.setSX(model.algState)
      CasadiLib.setSX(model.controls)
      CasadiLib.setSX(model.parameters)
    end
    
    function setMXModel(model)
      CasadiLib.setMX(model.state)
      CasadiLib.setMX(model.algState)
      CasadiLib.setMX(model.controls)
      CasadiLib.setMX(model.parameters)
    end
    
    
    function setSX(var)
      %setSX()
      CasadiLib.setSym(var,'SX')
    end
    
    function setMX(var)
      %setSX()
      CasadiLib.setSym(var,'MX')
    end
    

    function setSym(var,type,varargin)
      % setSX(type)
      %   type is either 'SX' or 'MX'
      % setSX(type,varIndex)
      %   not public only for recursion
      
      assert(isa(var,'Var'), 'Input has to be a Var.');
      
      % variable index is used in the naming of the symbolic variable
      varPrefix = '';
      if nargin==4
        varPrefix = varargin{1};
        varIndex = varargin{2};
        
        delimiter = '';
        if ~strcmp(varPrefix,'')
          delimiter = '_';
        end

        varPrefix = [varPrefix delimiter var.id varIndex];
        
      end
      
      
      % create symbolic variable and assign as value if var has no subvars
      if isempty(var.subVars)
        
        if prod(var.size) == 0
          return
        end
        
        if strcmp(type,'MX')
          var.set(casadi.MX.sym(varPrefix,var.size));
        else
          var.set(casadi.SX.sym(varPrefix,var.size));
        end
        return
      end
      
      % go recursively through subvars
      for i = 1:length(var.subVars)
        subVar = var.subVars(i);
        indizes = var.varIds.get(subVar.id);
        
        % find i in indizes
        subIndex = num2str(find(indizes==i));
        if length(indizes)==1
          subIndex = '';
        end
        
        CasadiLib.setSym(subVar,type,varPrefix,subIndex);
      end
      
    end % setSym
    
    
  end
  
end


function [nzi,nzj,nzv] = csr_to_sparse(rp,ci,ai,ncols)
% CSR_TO_SPARSE Convert from compressed row arrays to a sparse matrix
%
% A = csr_to_sparse(rp,ci,ai) returns the sparse matrix represented by the
% compressed sparse row representation rp, ci, and ai.  The number of
% columns of the output sparse matrix is max(max(ci),nrows).  See the call
% below.
%
% A = csr_to_sparse(rp,ci,ai,ncol) While we can infer the number of rows 
% in the matrix from this expression, you may want a
% different number of 
%
% [nzi,nzj,nzv] = csr_to_sparse(...) returns the arrays that feed the
% sparse call in matlab.  You can use this to avoid the sparse call and
% customize the behavior.
%
% This command "inverts" the behavior of sparse_to_csr.
% Repeated entries in the matrix are summed, just like sparse does.  
%
% See also SPARSE SPARSE_TO_CSR
% 
% Example:
%   A=sparse(6,6); A(1,1)=5; A(1,5)=2; A(2,3)=-1; A(4,1)=1; A(5,6)=1; 
%   [rp ci ai]=sparse_to_csr(A); 
%   A2 = csr_to_sparse(rp,ci,ai)

% David F. Gleich
% Copyright, Stanford University, 2008-2009

%  History
%  2009-05-01: Initial version
%  2009-05-16: Documentation and example

nrows = length(rp)-1;
nzi = zeros(length(ci),1);
for i=1:nrows
    for j=rp(i):rp(i+1)-1
        nzi(j) = i;
    end
end

if nargout<2,
    if nargin>3,
        nzi = sparse(nzi,ci,ai,nrows,ncols);
    else
        % we make the matrix square unless there are more columns
        ncols = max(max(ci),nrows);
        if isempty(ncols), ncols=0; end
        nzi = sparse(nzi,ci,ai,nrows,ncols);
    end
else
    nzj = ci;
    nzv = ai;
end 

end
