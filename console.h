#ifndef CONSOLE_H
#define CONSOLE_H

#define MAX_COMMAND_LENGTH	32

typedef char (*CommandFunc)(char *);

typedef struct {
  char *name;
  CommandFunc func;
} CommandArray;

char get_command(char *command, int maxlength);
char execute_command(char *cmdstr, CommandArray *commands);
void dispatch_console(void);

#endif
