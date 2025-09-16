// asm_termios_proxy.h
#ifndef ASM_TERMIOS_PROXY_H_
#define ASM_TERMIOS_PROXY_H_

#define winsize asmwinsize
#define termio asmtermio
#define termios asmtermios
#include <asm/termios.h>
#undef winsize
#undef termio
#undef termios

#endif  // ASM_TERMIOS_PROXY_H_