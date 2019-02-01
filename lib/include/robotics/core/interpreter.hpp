#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linenoise.h>

namespace Robotics
{
  class Interpreter
  {
  public:
    Interpreter(std::unordered_map<std::string, std::function<std::string(std::vector<double>)>> commands)
      : commands(commands)
    {
      linenoiseSetCompletionCallback(completion);
      linenoiseSetHintsCallback(hints);
      linenoiseHistoryLoad("history.txt");
    
    }
    
    static void completion(const char *buf, linenoiseCompletions *lc) {
      /*
      if (buf[0] == 'h') {
        linenoiseAddCompletion(lc,"hello");
        linenoiseAddCompletion(lc,"hello there");
      }
      */
    }

    static char *hints(const char *buf, int *color, int *bold) {
      if (!strcasecmp(buf,"hello")) {
        *color = 35;
        *bold = 0;
        return "World";
      }
      return NULL;
    }

    void run()
    {
      char *line;
      
      while((line = linenoise("hello> ")) != NULL) {
        if (line[0] != '\0' && line[0] != '/') {   /* Do something with the string. */
	  // split with "("
	  std::vector<std::string> v1;
	  std::stringstream ss1{std::string(line)};
	  std::string buf1;
	  while (std::getline(ss1, buf1, '(')) {
	    v1.push_back(buf1);
	  }

	  // split with ")"
	  std::vector<std::string> v2;
	  std::stringstream ss2{std::string(v1.at(1))};
	  std::string buf2;
	  while (std::getline(ss2, buf2, ')')) {
	    v2.push_back(buf2);
	  }

	  // split with ","
	  std::vector<std::string> v3;
	  std::stringstream ss3{std::string(v2.at(0))};
	  std::string buf3;
	  while (std::getline(ss3, buf3, ',')) {
	    v3.push_back(buf3);
	  }

	  // decomposite args
	  std::vector<double> args;
	  for (int i =  0; i < v3.size(); i++) {
	    args.push_back(std::stod(v3.at(i)));
	  }

	  // execute command
	  commands.at(v1.at(0))(args);
	  
	  linenoiseHistoryAdd(line);
	  linenoiseHistorySave("history.txt");
        } else if (!strncmp(line,"/historylen",11)) {   /* The "/historylen" command will change the history len. */
	  int len = atoi(line+11);
	  linenoiseHistorySetMaxLen(len);
        } else if (line[0] == '/') {
	  printf("Unreconized command: %s\n", line);
        }
        free(line);
      }
    }

  private:
    std::unordered_map<std::string, std::function<std::string(std::vector<double>)>> commands;
  };
}   // namespace Robotics
