classdef (Abstract) System < handle
  
  properties(Constant)
    DOT_PREFIX = 'D';
  end  
  
  properties
    state
    algVars
    controls
    parameters
    ode
    alg
    outputs
    
    initialConditions
    
    systemFun
    outputFun
  end
  
  methods (Abstract)
   setupVariables(self)
   setupEquation(self)
  end
  
  methods
    
    function self = System(parameters)
      self.state      = Var('state');
      self.algVars    = Var('algVars');
      self.controls   = Var('controls');
      
      if nargin == 0
        self.parameters = Parameters;
      else
        self.parameters  = parameters;
      end
      
      self.initialConditions = [];
      

      self.ode         = Var('ode');
      self.alg         = Var('alg');
      self.outputs     = Var('outputs');
      
      self.setupVariables;
            
      self.state.compile;

      self.controls.compile;
      self.parameters.compile;
      
      self.systemFun = UserFunction(@self.evaluate,{self.state,self.algVars,self.controls,self.parameters},2);
      self.outputFun = UserFunction(@self.getOutputs,{self.state,self.algVars,self.controls,self.parameters},1);
    end
    
    function initialCondition(~,~,~)
    end
    
    function setupOutputs(self,state,algVars,controls,parameters)
    end
    
    function outputs = getOutputs(self,state,algVars,controls,parameters)
      self.outputs     = Var('outputs');
      self.setupOutputs(state,algVars,controls,parameters)
      outputs = self.outputs;
    end
    
    function [ode,alg] = evaluate(self,state,algVars,controls,parameters)
      % evaluate the system equations for the assigned 
      
      self.alg         = Var('alg');
      
      self.setupEquation(state,algVars,controls,parameters);
      
      self.ode.compile;
      self.alg.compile;
      
      ode = self.ode;
      alg = self.alg;
    end
    
    function addState(self,id,size)
      self.state.add(id,size);
      self.ode.add([System.DOT_PREFIX id],size)
    end
    function addAlgVar(self,id,size)
      self.algVars.add(id,size);
    end
    function addControl(self,id,size)
      self.controls.add(id,size);
    end
    function addParameter(self,id,size)
      self.parameters.add(id,size);
    end
    
    function state = getState(self,id)
      state = self.state.get(id).value;
    end
    function algState = getAlgVar(self,id)
      algState = self.algVars.get(id).value;
    end
    function control = getControl(self,id)
      control = self.controls.get(id).value;
    end
    function param = getParameter(self,id)
      param = self.parameters.get(id).value;
    end

    function setODE(self,id,equation)
      %
      self.ode.get([System.DOT_PREFIX id]).set(equation);
    end
    
    function setAlgEquation(self,equation)
      self.alg.add(Var(equation,'algEq'));
    end
    
    function setOutput(self,name,equation)
      self.outputs.add(equation,name);
    end
    
    function setInitialCondition(self,value)
      self.initialConditions = [self.initialConditions; value];      
    end
    
    function  ic = getInitialCondition(self,state,parameters)
      self.initialConditions = [];
      self.initialCondition(state,parameters)
      ic = self.initialConditions;
    end
    
    function controls = callIterationCallback(self,state,algVars,parameters)
      controls = self.controls;
      controls.set(0);
      self.simulationCallback(state,algVars,controls,parameters);
    end
    
    function solutionCallback(self,solution)
      
      N = solution.get('state').getNumberOfVars;
      params = solution.get('parameters');
      
      for k=1:N-1
        thisState = solution.get('state',k+1);
        thisAlgVars = solution.get('integratorVars',k).get('algVars',3);
        self.callIterationCallback(thisState,thisAlgVars,params);
      end
      
    end
    
  end
  
end

