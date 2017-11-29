#include <includes.h>
#include <stdarg.h>


/* we use this so that we can do without the ctype library */
#define is_digit(c)     ((c) >= '0' && (c) <= '9')

extern void serial_puts (const char *s);

void printf (const char *fmt, ...)
{
	va_list args;
	uint i;
	char printbuffer[256];

	va_start (args, fmt);

	/* For this to work, printbuffer must be larger than
	 *          * anything we ever want to print.
	 *                   */
	i = vsprintf (printbuffer, fmt, args);
	va_end (args);

	/* Print the string */
	serial_puts (printbuffer);
}

