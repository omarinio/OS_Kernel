/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

extern void     main_console();
extern uint32_t tos_user;
extern void     main_P3();
extern void     main_P4();
extern void     main_P5();
extern void     main_dinner();

pcb_t procTab[ MAX_PROCS ];
pcb_t* executing = NULL;

pipe_t pipes[ MAX_PIPES ];
int num_pipes = 0;

uint16_t fb[ 600 ][ 800 ];
int mousePos[2] = {0,0};

void dispatch( ctx_t* ctx, pcb_t* prev, pcb_t* next ) {
  char prev_pid = '?', next_pid = '?';

  if( NULL != prev ) {
    memcpy( &prev->ctx, ctx, sizeof( ctx_t ) ); // preserve execution context of P_{prev}
    prev_pid = '0' + prev->pid;
  }
  if( NULL != next ) {
    memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  execution context of P_{next}
    next_pid = '0' + next->pid;
  }

    PL011_putc( UART0, '[',      true );
    PL011_putc( UART0, prev_pid, true );
    PL011_putc( UART0, '-',      true );
    PL011_putc( UART0, '>',      true );
    PL011_putc( UART0, next_pid, true );
    PL011_putc( UART0, ']',      true );

    executing = next;

    // update current process on display
    drawRectangle(fb,280,432,240,48,BLACK);
    writeString(fb,"[",280,432,WHITE,5);
    writeChar(fb,prev_pid,280,440,WHITE,5);
    writeString(fb,"->",280,448,WHITE,5);
    writeChar(fb,next_pid,280,464,WHITE,5);
    writeString(fb,"]",280,472,WHITE,5);

  return;
}
// finds free pipe index to be populated
int getFreePipe() {
  for (int i = 0; i < MAX_PIPES; i++) {
    if(pipes[i].status == STATUS_TERMINATED) {
      return i;
    }
  }
}
// finds free process
pcb_t* getChildProc() {
  for (int i = 0; i < MAX_PROCS; i++) {
    if (procTab[i].status == STATUS_TERMINATED) {
      return &procTab[i];
    }
  }
  return NULL;
}

int getMaxPriority() {
  int maxPriority = 0;
  int temp;
  // loops through all processes
  for(int i = 0; i < MAX_PROCS; i++) {
    int tempPriority = 0;
    // checks if process is not currently being executed and if it is initalised
      if (procTab[i].pid != executing->pid && procTab[i].status != STATUS_TERMINATED) {
        // sets temp priority to initial priority plus how long its had to wait
        tempPriority = procTab[i].priority + procTab[i].age;
      }
      // if temp priority is greater than max prioritym assign new to max
      if (maxPriority <= tempPriority && procTab[i].status != STATUS_TERMINATED) {
        maxPriority = tempPriority;
        temp = i;
      }

  }
  // increment age of each process
  for(int i = 0; i < MAX_PROCS; i++) {
    if (procTab[i].status != STATUS_TERMINATED) {
      procTab[i].age = procTab[i].age + 1;
    }
  }
  // set the age of the executing process to 0
  procTab[temp].age = 0;

  return temp;

}

void schedule( ctx_t* ctx ) {

  int process = getMaxPriority();

  dispatch(ctx, executing, &procTab[process]);

  executing->status = STATUS_READY;
  procTab[process].status = STATUS_EXECUTING;
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////KEY PRESSES////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Scan codes found here
// https://www.win.tue.nl/~aeb/linux/kbd/scancodes-1.html
char keyPress(unsigned char key) {
    switch(key) {
        case 0x04: // 3 was pressed
          return 'T';
        case 0x05: // 4 was pressed
          return 'F';
        case 0x06: // 5 was pressed
          return 'I';
        case 0x19: // P was pressed
          return 'P';
        case 0x25: // K was pressed
          return 'K';
        default:
          return ' ';
    }
}

void lcdExec(ctx_t *ctx, uint32_t process) {
  pcb_t* child = getChildProc();

  if (child == NULL) { //no empty process?
    return;
  }

  //copy context onto child
  memcpy(&child->ctx, ctx, sizeof(ctx_t));

  //update status
  child->status = STATUS_CREATED;

  //copy stack
  uint32_t offset = (uint32_t) &executing->tos - ctx->sp;
  child->ctx.sp = child->tos - offset;
  // memcpy((void *)child->ctx.sp, (void *)ctx->sp, offset);
  memcpy((void *) (child->tos - 0x00001000), (void *) (executing->tos - 0x00001000), 0x00001000);

  child->ctx.pc = process;
}

void killAll(ctx_t *ctx) {
    for(int i = 0; i < MAX_PROCS; i++) {
      procTab[i].status = STATUS_TERMINATED;
    }
}

///////////////////////////////////////////////////////////////////////////////
////////////////////////////////MOUSE FUNCT////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// 0 empty, 1 black, 2 white
int mouse[18][12] = {
    {1,1,0,0,0,0,0,0,0,0,0,0},
    {1,2,1,0,0,0,0,0,0,0,0,0},
    {1,2,2,1,0,0,0,0,0,0,0,0},
    {1,2,2,2,1,0,0,0,0,0,0,0},
    {1,2,2,2,2,1,0,0,0,0,0,0},
    {1,2,2,2,2,2,1,0,0,0,0,0},
    {1,2,2,2,2,2,2,1,0,0,0,0},
    {1,2,2,2,2,2,2,2,1,0,0,0},
    {1,2,2,2,2,2,2,2,2,1,0,0},
    {1,2,2,2,2,2,2,2,2,2,1,0},
    {1,2,2,2,2,2,2,2,2,2,2,1},
    {1,2,2,2,2,2,2,1,1,1,1,1},
    {1,2,2,2,1,2,2,1,0,0,0,0},
    {1,2,2,1,0,1,2,2,1,0,0,0},
    {1,2,1,0,0,1,2,2,1,0,0,0},
    {1,1,0,0,0,0,1,2,2,1,0,0},
    {0,0,0,0,0,0,1,2,2,1,0,0},
    {0,0,0,0,0,0,0,1,1,0,0,0},

};
// old image of mouse, array set to all 0s initally
int mouseOld[18][12] = {0};

void drawMouse(int xPos, int yPos) {
  // uses the mouse array to draw it on the screen
  // 0 leaves the picture unchanged
  // 1 draws a black pixel
  // 2 draws a white pixel
  for(int i = 0; i < 18; i++) {
    for(int j = 0; j < 12; j++) {
      if(mouse[i][j] == 0) {

      } else if(mouse[i][j] == 1) {
        fb[i+xPos][j+yPos] = BLACK;
      } else {
        fb[i+xPos][j+yPos] = WHITE;
      }
    }
  }
}

void moveMouse(int xPos, int yPos) {
   // initalise empty array and copy the current position of the mouse onto that array
   int prevMouse[2];
   memcpy(&prevMouse, &mousePos, sizeof(mousePos));

   for(int i = 0; i < 18; i++) {
     for(int j = 0; j < 12; j++) {
        // fill the fb with the image that had been there before the mouse
        fb[mousePos[0] + i][mousePos[1] + j] = mouseOld[i][j];
     }
   }
   // check if mouse is inside boundaries so it can be redrawn
   if(mousePos[0] - xPos >=0 && mousePos[0] - xPos <= 580) {
     mousePos[0] -= xPos;
   }
   if(mousePos[1] + yPos >=0 && mousePos[1] + yPos <= 780) {
     mousePos[1] += yPos;
   }

   for(int i = 0; i < 18; i++) {
     for(int j = 0; j < 12; j++) {
        // save the image under where the mouse will travel to
        mouseOld[i][j] = fb[mousePos[0] + i][mousePos[1] + j];
     }
   }

   drawMouse(mousePos[0], mousePos[1]);

}


void hilevel_handler_rst( ctx_t* ctx              ) {
  /* Invalidate all entries in the process table, so it's clear they are not
   * representing valid (i.e., active) processes.
   */

   PL011_putc( UART0, '[', true);
   PL011_putc( UART0, 'R', true);
   PL011_putc( UART0, 'E', true);
   PL011_putc( UART0, 'S', true);
   PL011_putc( UART0, 'E', true);
   PL011_putc( UART0, 'T', true);
   PL011_putc( UART0, ']', true);

  memset( &procTab[ 0 ], 0, sizeof( pcb_t ) ); // initialise 0-th PCB = P_1
  procTab[ 0 ].pid      = 1;
  procTab[ 0 ].status   = STATUS_READY;
  procTab[ 0 ].tos      = ( uint32_t )( &tos_user  );
  procTab[ 0 ].ctx.cpsr = 0x50;
  procTab[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
  procTab[ 0 ].ctx.sp   = procTab[ 0 ].tos;
  procTab[ 0 ].priority = 2;
  procTab[ 0 ].age      = 0;

  for (int i = 1; i < MAX_PROCS; i++) {
    memset( &procTab[ i ], 0, sizeof( pcb_t ) );
    procTab[ i ].pid      = i+1;
    procTab[ i ].status   = STATUS_TERMINATED;
    procTab[ i ].ctx.cpsr = 0x50;
    procTab[ i ].ctx.sp   = procTab[ 0 ].tos - (i*0x00001000);
    procTab[ i ].tos      = procTab[ i ].ctx.sp;
    procTab[ i ].priority = 1;
    procTab[ i ].age      = 0;
  }

  // Initalising pipes
  for (int i = 0; i < MAX_PIPES; i++) {
    memset( &pipes[ i ], 0, sizeof( pipe_t ) );
    pipes[i].status = STATUS_TERMINATED;
  }

  /* Automatically execute the user programs P1 and P2 by setting the fields
   * in two associated PCBs.  Note in each case that
   *
   * - the CPSR value of 0x50 means the processor is switched into USR mode,
   *   with IRQ interrupts enabled, and
   * - the PC and SP values match the entry point and top of stack.
   */

   TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
   TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
   TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
   TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
   TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

   GICC0->PMR          = 0x000000F0; // unmask all            interrupts
   GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
   GICC0->CTLR         = 0x00000001; // enable GIC interface
   GICD0->CTLR         = 0x00000001; // enable GIC distributor

   // LCD

   SYSCONF->CLCD      = 0x2CAC;     // per per Table 4.3 of datasheet
   LCD->LCDTiming0    = 0x1313A4C4; // per per Table 4.3 of datasheet
   LCD->LCDTiming1    = 0x0505F657; // per per Table 4.3 of datasheet
   LCD->LCDTiming2    = 0x071F1800; // per per Table 4.3 of datasheet

   LCD->LCDUPBASE     = ( uint32_t )( &fb );

   LCD->LCDControl    = 0x00000020; // select TFT   display type
   LCD->LCDControl   |= 0x00000008; // select 16BPP display mode
   LCD->LCDControl   |= 0x00000800; // power-on LCD controller
   LCD->LCDControl   |= 0x00000001; // enable   LCD controller

        /* Configure the mechanism for interrupt handling by
    *
    * - configuring then enabling PS/2 controllers st. an interrupt is
    *   raised every time a byte is subsequently received,
    * - configuring GIC st. the selected interrupts are forwarded to the
    *   processor via the IRQ interrupt signal, then
    * - enabling IRQ interrupts.
    */
    PS20->CR           = 0x00000010; // enable PS/2    (Rx) interrupt
    PS20->CR          |= 0x00000004; // enable PS/2 (Tx+Rx)
    PS21->CR           = 0x00000010; // enable PS/2    (Rx) interrupt
    PS21->CR          |= 0x00000004; // enable PS/2 (Tx+Rx)

    uint8_t ack;

          PL050_putc( PS20, 0xF4 );  // transmit PS/2 enable command
    ack = PL050_getc( PS20       );  // receive  PS/2 acknowledgement
          PL050_putc( PS21, 0xF4 );  // transmit PS/2 enable command
    ack = PL050_getc( PS21       );  // receive  PS/2 acknowledgement

    GICC0->PMR         = 0x000000F0; // unmask all          interrupts
    GICD0->ISENABLER1 |= 0x00300000; // enable PS2          interrupts
    GICC0->CTLR        = 0x00000001; // enable GIC interface
    GICD0->CTLR        = 0x00000001; // enable GIC distributor

    // info text
    writeString(fb,"KERNEL",0,376,WHITE,6);
    writeString(fb,"PRESS 3 TO RUN P3",90,336,WHITE,2);
    writeString(fb,"PRESS 4 TO RUN P4",120,336,WHITE,2);
    writeString(fb,"PRESS 5 TO RUN P5",150,336,WHITE,2);
    writeString(fb,"PRESS P TO RUN PHILOSOPHER",180,336,WHITE,2);
    // create buttons
    drawRectangle(fb,400,30,80,80,BLUE);
    drawRectangle(fb,400,250,80,80,BLUE);
    drawRectangle(fb,400,470,80,80,BLUE);
    drawRectangle(fb,400,690,80,80,BLUE);
    // status if process is running
    drawRectangle(fb,500,30,80,20,RED);
    drawRectangle(fb,500,250,80,20,RED);
    drawRectangle(fb,500,470,80,20,RED);
    drawRectangle(fb,500,690,80,20,RED);
    // button text
    writeString(fb,"P3",420,60,BLACK,5);
    writeString(fb,"P4",420,280,BLACK,5);
    writeString(fb,"P5",420,500,BLACK,5);
    writeString(fb,"PHILO",420,720,BLACK,5);

    writeString(fb,"NOW RUNNING:",280,336,BLUE,5);
    writeString(fb,"PRESS K TO KILL ALL PROGRAMS",550,288,WHITE,3);
    writeString(fb,"BUTTONS",340,376,WHITE,5);
    // initalise mouse
    drawMouse(mousePos[0], mousePos[1]);
    dispatch( ctx, NULL, &procTab[ 0 ] );
    int_enable_irq();

  return;
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
  /* Based on the identifier (i.e., the immediate operand) extracted from the
   * svc instruction,
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call, then
   * - write any return value back to preserved usr mode registers.
   */

  switch( id ) {

    case 0x00 : { // 0x00 => yield()
      schedule( ctx );

      break;
    }

    case 0x01 : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        PL011_putc( UART0, *x++, true );
      }

      ctx->gpr[ 0 ] = n;

      break;
    }

    case 0x02 : {
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        PL011_getc( UART0, true );
      }

      ctx->gpr[ 0 ] = n;

      break;
    }

    case 0x03 : { // Fork
      PL011_putc( UART0, '[', true);
      PL011_putc( UART0, 'F', true);
      PL011_putc( UART0, 'O', true);
      PL011_putc( UART0, 'R', true);
      PL011_putc( UART0, 'K', true);
      PL011_putc( UART0, ']', true);

      pcb_t* child = getChildProc();

      if (child == NULL) { //no empty process?
        break;
      }

      //copy context onto child
      memcpy(&child->ctx, ctx, sizeof(ctx_t));

      //update status
      child->status = STATUS_CREATED;

      //copy stack
      uint32_t offset = (uint32_t) &executing->tos - ctx->sp;
      child->ctx.sp = child->tos - offset;
      // memcpy((void *)child->ctx.sp, (void *)ctx->sp, offset);
      memcpy((void *) (child->tos - 0x00001000), (void *) (executing->tos - 0x00001000), 0x00001000);

      //set return value to 0 for child and child pid for parent
      child->ctx.gpr[ 0 ] = 0;
      ctx->gpr[ 0 ] = child->pid;

      break;

    }

    case 0x04 : { // Exit
      PL011_putc( UART0, '[', true);
      PL011_putc( UART0, 'E', true);
      PL011_putc( UART0, 'X', true);
      PL011_putc( UART0, 'I', true);
      PL011_putc( UART0, 'T', true);
      PL011_putc( UART0, ']', true);

      executing->status = STATUS_TERMINATED;
      schedule(ctx);
      break;
    }

    case 0x05 : { // Exec
      PL011_putc( UART0, '[', true);
      PL011_putc( UART0, 'E', true);
      PL011_putc( UART0, 'X', true);
      PL011_putc( UART0, 'E', true);
      PL011_putc( UART0, 'C', true);
      PL011_putc( UART0, ']', true);

      ctx->pc = ctx->gpr[ 0 ];
      ctx->sp = executing->tos;
      break;
    }

    case 0x06 : { // Kill
      PL011_putc( UART0, '[', true);
      PL011_putc( UART0, 'K', true);
      PL011_putc( UART0, 'I', true);
      PL011_putc( UART0, 'L', true);
      PL011_putc( UART0, 'L', true);
      PL011_putc( UART0, ']', true);

      uint32_t kill = (uint32_t) (ctx->gpr[ 0 ]);
      procTab[kill].status = STATUS_TERMINATED;
      break;
    }

    case 0x07 : { //Nice
      PL011_putc( UART0, '[', true);
      PL011_putc( UART0, 'N', true);
      PL011_putc( UART0, 'I', true);
      PL011_putc( UART0, 'C', true);
      PL011_putc( UART0, 'E', true);
      PL011_putc( UART0, ']', true);
      break;
    }

    case 0x08 : { //SYS_CREATE_PIPE
      // get next free pipe
      int free = getFreePipe();
      // initalise pipe
      pipes[ free ].fd = free;
      pipes[ free ].status = STATUS_CREATED;
      pipes[ free ].send = ( pid_t ) ctx->gpr[0];
      pipes[ free ].recieve = ( pid_t ) ctx->gpr[1];
      pipes[ free ].content = EMPTY;
      ctx->gpr[ 0 ] = pipes[ free ].fd;

      num_pipes +=1;

      break;
    }

    case 0x09 : { //SYS_CLOSE_PIPE
      // get pipe id from gpr
      int pipeID = (int) ctx->gpr[0];
      // set status to terminated
      pipes[pipeID].status = STATUS_TERMINATED;
      num_pipes -= 1;
      break;
    }

    case 0x10 : { //SYS_READ_PIPE
      // get pipe id from gpr
      int pipeID = (int) ctx->gpr[0];
      // set return value to the content of the pipe
      ctx->gpr[0] = pipes[pipeID].content;

      break;
    }

    case 0x11 : { //SYS_WRITE_PIPE
      // get pipe id from gpr
      int pipeID = (int) ctx->gpr[0];
      // get new pipe content from gpr
      pipes[pipeID].content = ctx->gpr[1];

      break;
    }

    case 0x12 : { //SYS_PID
       // set return value to current process pid
       ctx->gpr[0] = executing->pid;
       break;
    }

    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}

void hilevel_handler_irq(ctx_t* ctx) {
  // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;

  if     ( id == GIC_SOURCE_PS20 ) { //keyboard interrupt
    uint8_t x = PL050_getc( PS20 );

    char key = x;

    switch(keyPress(key)) {
        case 'T': { // P3 to be executed
            lcdExec(ctx, (uint32_t) &main_P3); //H
            drawRectangle(fb,500,30,80,20,GREEN);
            break;
        }
        case 'F': { // P4 to be executed
            lcdExec(ctx, (uint32_t) &main_P4); //G
            drawRectangle(fb,500,250,80,20,GREEN);
            break;
        }
        case 'I': { // P5 to be executed
            lcdExec(ctx, (uint32_t) &main_P5); //Z
            drawRectangle(fb,500,470,80,20,GREEN);

            break;
        }
        case 'P': { // Philosophers to be executed
            lcdExec(ctx, (uint32_t) &main_dinner); //9
            drawRectangle(fb,500,690,80,20,GREEN);
            break;
        }
        case 'K': { // Kill all programs
            killAll(ctx); //L
        }
        default:
          break;
    }

  }
  else if( id == GIC_SOURCE_PS21 ) { //mouse interrupt
    //wiki.osdev.org/PS/2_Mouse
    uint8_t clicked = PL050_getc( PS21 );

    if ((clicked & 0x01) == 1) {
        if (mousePos[0] > 400 && mousePos[0] < 480) {
          if (mousePos[1] > 30 && mousePos[1] < 110) {
            // P3 button
            lcdExec(ctx, (uint32_t) &main_P3);
            drawRectangle(fb,500,30,80,20,GREEN);

          } else if (mousePos[1] > 250 && mousePos[1] < 330) {
            // P4 button
            lcdExec(ctx, (uint32_t) &main_P4);
            drawRectangle(fb,500,250,80,20,GREEN);

          } else if (mousePos[1] > 470 && mousePos[1] < 550) {
            // P5 button
            lcdExec(ctx, (uint32_t) &main_P5);
            drawRectangle(fb,500,470,80,20,GREEN);

          } else if (mousePos[1] > 690 && mousePos[1] < 570) {
            // Philosopher button
            lcdExec(ctx, (uint32_t) &main_dinner);
            drawRectangle(fb,500,690,80,20,GREEN);

          }
        }
    }
    // get other 2 bits from PS21
    uint8_t yPos    = PL050_getc( PS21 );
    uint8_t xPos    = PL050_getc( PS21 );
    // shift to check the new position
    int yChange = yPos - ((clicked << 4) & 0x100);
    int xChange = xPos - ((clicked << 3) & 0x100);

    moveMouse(xChange, yChange);

  }

  // Step 4: handle the interrupt, then clear (or reset) the source.

  if( id == GIC_SOURCE_TIMER0 ) {
    TIMER0->Timer1IntClr = 0x01;
    schedule(ctx);
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}
