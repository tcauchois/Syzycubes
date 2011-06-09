#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <avr/pgmspace.h>

#include "main.h"
#include "console.h"
#include "ADXL345.h"
#include "TLC5947DAP.h"

#define MAX_COMMAND_LENGTH	32

typedef char (*CommandFunc)(char *);

typedef struct {
  char *name;
  CommandFunc func;
} CommandArray;

char get_command(char *command, int maxlength);
char execute_command(char *cmdstr, CommandArray *commands);
char helpCmd(char *);
char ledCmdRgb(char *);
char ledCmdHsb(char *);
char accelCmd(char *);
char aboutCmd(char *);

CommandArray mainMenu[] =
{
  { "help", helpCmd },
  { "rgb", ledCmdRgb },
  { "hsb", ledCmdHsb },
  { "accel", accelCmd },
  { "about", aboutCmd },
  { NULL, NULL },
};

char aboutCmd(char *arg)
{
  printf_P(PSTR("This awesome piece of awesomeness designed by Ardent Heavy Industries!\n"));
  printf_P(PSTR("Hardware by Nathan Hunsperger\n"));
  printf_P(PSTR("Software by Tom Cauchois\n"));
  return 0;
}

char helpCmd(char *arg)
{
  int i;

  printf_P(PSTR("Commands:\n"));
  for(i = 0; mainMenu[i].name != NULL; ++i)
    printf_P(PSTR("  %s\n"), mainMenu[i].name);

  return 0;
}

char ledCmdRgb(char *arg)
{
  unsigned long r,g,b;
  r = strtoul(arg, &arg, 16);
  g = strtoul(arg, &arg, 16);
  b = strtoul(arg, &arg, 16);

  printf_P(PSTR("Setting LEDs (0x%lx, 0x%lx, 0x%lx)\n"), r, g, b);
  send_rgb((uint16_t)r, (uint16_t)g, (uint16_t)b);
  return 0;
}

char ledCmdHsb(char *arg)
{
  unsigned long h,s,b;
  h = strtoul(arg, &arg, 16);
  s = strtoul(arg, &arg, 16);
  b = strtoul(arg, &arg, 16);

  printf_P(PSTR("Setting LEDs HSB (0x%lx, 0x%lx, 0x%lx)\n"), h, s, b);
  send_hsb((uint16_t)h, (uint16_t)s, (uint16_t)b);
  return 0;
}

char accelCmd(char *arg)
{
  int16_t x,y,z;
  adxl345_getxyz(&x, &y, &z);
  printf_P(PSTR("Accel: (%d, %d, %d)\n"), x, y, z);
  return 0;
}

void dispatch_console(void)
{
  static char command[MAX_COMMAND_LENGTH] = {0};
  static char init = 0;
  int length = strlen(command);

  if(init == 0)
  {
    printf_P(PSTR("> "));
    init = 1;
  }

  if(get_command(command+length, MAX_COMMAND_LENGTH-length) == 0)
  {
    execute_command(command, mainMenu);
    memset(command, 0, sizeof(command));
    printf_P(PSTR("> "));
  }
}

char get_command(char *command, int maxlength)
{
  char input=0, length=0;

  if(uart_charwaiting() != 0)
    return 1;
  input = uart_getchar();

  while(input != '\r' && input != '\n')
  {
    *command = input;
    printf_P(PSTR("%c"), *command++);
    if(++length == maxlength - 1)
      break;
    if(uart_charwaiting() != 0)
      return 1;
    input = uart_getchar();
  }
  printf_P(PSTR("\n"));

  return 0;
}

char execute_command(char *cmdstr, CommandArray *commands)
{
  int i;

  while(*cmdstr == ' ') ++cmdstr;

  if(*cmdstr == 0)
    return 0;

  for(i = 0; commands[i].name != NULL; ++i)
  {
    if(!strncmp(cmdstr, commands[i].name, strlen(commands[i].name)))
    {
      return commands[i].func(cmdstr + strlen(commands[i].name));
    }
  }

  printf_P(PSTR("Unknown command! Type 'help' for a list of commands\n"));
  return 1;
}
