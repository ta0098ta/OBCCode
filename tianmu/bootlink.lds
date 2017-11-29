/* 
 * file: ramlnk.dld
 *
 * Author: Ernie price
 * eprice@bitwise.net
 *
 */

SECTIONS
{
	/*
	 * The first GROUP contains code and constant data.
	 */

	. = 0x00000;
	. = ALIGN(4); 
	.init : { *(.init*) }

	.text : { *(.text) }
	. = ALIGN(4); 
	.sdata2 	: { *(.sdata2) }   /* small CONST area */
	. = ALIGN(4); 
	.data 	: { *(.data) }
	. = ALIGN(4); 
	.sdata 	: { *(.sdata) }
	. = ALIGN(4); 
	. = ALIGN(4096);
	__init_begin = .;
	.text.init : { *(.text.init) }
	.data.init : { *(.data.init) }
	. = ALIGN(4096);
	__init_end = .;

	. = ALIGN(4); 
	__bss_start = .;
	.sbss 	: { *(.sbss) }
	.dynbss : {*(.dynbss)}
	.bss  	: { *(.bss) }
	. = ALIGN(4); 
	_end = .;

}
Sbase   = __bss_start;
/*Send   = ADDR(.sbss) + SIZEOF(.sbss) + SIZEOF(.bss);*/
Send   = _end;

