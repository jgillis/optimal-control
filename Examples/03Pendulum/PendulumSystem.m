classdef PendulumSystem < System
  methods
    
    function self = PendulumSystem()
      self = self@System();
    end
    
    function setupVariables(self)
      self.addState('p',[2,1]);
      self.addState('v',[2,1]);
      self.addControl('F',[1,1]);
      self.addAlgVar('lambda',[1,1]);
      
      self.addParameter('m',[1 1]);
      self.addParameter('l',[1 1]);
      
    end
    function setupEquation(self,state,algVars,controls,parameters)
      p       = state.p;
      v       = state.v;
      F       = controls.F;
      lambda  = algVars.lambda;
      m       = parameters.m;

      ddp     = - 1/m * lambda*p - [0;9.81] + [F;0];

      self.setODE('p',v); 
      self.setODE('v',ddp);

      self.setAlgEquation(dot(ddp,p)+v(1)^2+v(2)^2);
    end
    function initialCondition(self,state,parameters)
      l = parameters.l.value;
      p = state.p.value;
      v = state.v.value;
      
      self.setInitialCondition(p(1)^2+p(2)^2-l^2);
      self.setInitialCondition(dot(p,v));
    end
    function simulationCallback(self,states,algVars,controls,parameters)
      p = states.p.value;
      l = parameters.l.value;
      
      plot(0,0,'ob')
      hold on
      plot([0,p(1)],[0,p(2)],'-k')
      plot(p(1),p(2),'or')
      xlim([-l,l])
      ylim([-l,l])
      
      pause(0.1);
      
    end
  end
end

