#pragma once

#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <stdio.h>


void init_keyboard();
void close_keyboard();
int _kbhit();
int _getch();
int _putch(int c);
