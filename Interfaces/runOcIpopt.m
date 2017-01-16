name = 'example';
nlpStruct = CasadiLib.loadNLP(name);

funcs.objective         = nlpStruct.f;
funcs.constraints       = nlpStruct.g;
funcs.gradient          = nlpStruct.fGrad;
funcs.jacobian          = nlpStruct.gJac;
funcs.jacobianstructure = @() nlpStruct.gSparsity;
funcs.hessian           = @(x,sig,lam) nlpStruct.H(x,lam);
funcs.hessianstructure  = @() nlpStruct.HSparsity;


x0         = [0 0];  % The starting point.
options.lb = [-inf -inf];  % Lower bound on the variables.
options.ub = [inf inf];  % Upper bound on the variables.
options.cl = 0;   % Lower bounds on the constraint functions.
options.cu = inf;   % Upper bounds on the constraint functions.

tic; [x info] = ipopt(x0,funcsMEX,options); toc;