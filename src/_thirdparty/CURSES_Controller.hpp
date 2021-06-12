#ifdef NCURSES_ENABLE
#include <curses.h>
#include <signal.h>

static int row, col;
static WINDOW *NCURSES_WINDOWS;
static char mesg[] = "Just a string";

void resizeHandler(int);

inline void WindowInit()
{
	NCURSES_WINDOWS = initscr();
	signal(SIGWINCH, resizeHandler);
	cbreak();
	noecho();
	getmaxyx(NCURSES_WINDOWS, row, col);
	mvprintw(row / 2, (col - strlen(mesg)) / 2, "%s", mesg);
	refresh();
	while (true)
	{

		usleep(50000);
	}
}

void resizeHandler(int sig)
{
	endwin();
	refresh();
	clear();
	getmaxyx(NCURSES_WINDOWS, row, col);
	mvprintw(row / 2, (col - strlen(mesg)) / 2, "%s", mesg);
	refresh();
}
#endif