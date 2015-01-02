/* $Id: c018_reloc.c,v 1.1 2006/03/22 17:05:18 akhe Exp $ */

/* Copyright (c)1999 Microchip Technology */

/* MPLAB-C18 startup code */

/* external reference to the user's main routine */
extern void main (void);
/* prototype for the startup function */
void _entry (void);
void _startup (void);
extern near char __FPFLAGS;
#define RND 6

// Added by Ghole. To make sure code runs when not remapped
#ifdef RUNWITHOUTBOOTLOADER
#pragma code _resetremap =0x000
void
_resetremap (void)
{
    _asm goto 0x800 _endasm
}


#ifdef RELOCATE     //DEBUG & RELOCATE means need to remap the IRQs too.

#pragma code _lowirqremap =0x008
void
_lowirqremap (void)
{
    _asm goto 0x808 _endasm
}
#pragma code _highirqremap =0x018
void
_highirqremap (void)
{
    _asm goto 0x818 _endasm
}
#endif
#endif

#pragma code _entry_scn=0x000800
void
_entry (void)
{
_asm goto _startup _endasm

}

#pragma code _startup_scn
void
_startup (void)
{
  _asm
    // Initialize the stack pointer
    lfsr 1, _stack
    lfsr 2, _stack

    clrf TBLPTRU, 0 // 1st silicon doesn't do this on POR

    bcf __FPFLAGS,RND,0 // Initalize rounding flag for floating point libs
_endasm loop:

  // Call the user's main routine
  main ();

  goto loop;
}                               /* end _startup() */
