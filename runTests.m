function runTests(version,changeMessage)
  % runTests(version,changeMessage)
  % Run all tests with version (e.g. 1.01a) and a message that describes
  % the changes of this test with respect to the previous version.
  % Indicate wether examples or tests were added because than the runtime
  % of test will differ from preious versions.
  
  StartupOC
  global testDir
  
  if nargin == 0
    version       = 'undefined';
    changeMessage = 'undefined';
  end

  close all
  set(0,'DefaultFigureVisible','off');

  coreTestsPassed     = true;
  casadiTestsPassed   = true;
  exmapleTestsPassed  = true;

  %% run core class tests
  coreTestsResult = runTest('Core',@TestVar);

  %% run casadi tests
  casadiTestsResult = runTest('Casadi',@TestCasadiVar);

  %% run examples
  exampleTestsResult = runTest('Example',@Example);

  %% display and save results
  testOuput = {};
  testOuput{end+1} = sprintf('Test on %s\nVersion: %s\nChange message: %s\n\n',datestr(now),version,changeMessage);
  testOuput{end+1} = getTestOutput(coreTestsResult);
  testOuput{end+1} = getTestOutput(casadiTestsResult);
  testOuput{end+1} = getTestOutput(exampleTestsResult);
  fprintf([testOuput{:}]);
  
  prompt = input('Save result to file? (y,n)','s');
  if strcmp(prompt,'y') || strcmp(prompt,'yes')
    fileName = [datestr(now,'yyyy-mm-dd_HHMMSS') '.txt'];
    filePath = fullfile(testDir,fileName);
    resultsFile = fopen(filePath,'w');
    fprintf(resultsFile,[testOuput{:}]);
    fclose(resultsFile);
  end

  %% local functions 
  function testResult = runTest(testName,scriptHandle)
    testResult = struct;
    testResult.name = testName;
    testResult.passed = true;
    testResult.runtime = 0;
    testResult.exception = '';
    try
      tic;
      scriptHandle();
      testResult.runtime = toc;
    catch exception
      testResult.passed = false;
      testResult.exception = exception;
    end
  end

  function outputString = getTestOutput(testResult)
    if testResult.passed
      s1 = sprintf('%s Tests passed\n',testResult.name);
      s2 = sprintf('It took %.4f seconds.\n\n',testResult.runtime);
      outputString = [s1,s2];
    else
      outputString = sprintf('%s Tests failed\n',testResult.name);
      disp(getReport(testResult.exception))
    end
  end
end




