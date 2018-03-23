
// RTOS Framework - Spring 2018
// J Losh

// Student Name:Siddaram Gireppa Kanni
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only one .c files in an e-mail to me (not in a compressed file):
// xx_rtos.c   Single-file with your project code
// (xx is a unique number that will be issued in class)
// Please do not include .intvecs section in your code submissions
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4))) // off-board red LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) // off-board orange LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) // off-board yellow LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4))) // off-board green LED
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED

#define PB1            (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))
#define PB2            (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))
#define PB3            (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))
#define PB4            (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();
int rtosScheduler();
void Setsp(uint32_t value);
uint32_t Getsp();
uint32_t PushPC();

uint32_t sysSP;

void idle();
void idle2();
void flash4Hz();
void oneshot();
void partOfLengthyFn();
void lengthyFn();
void readKeys();
void debounce();
void uncooperative();
void important();
void shell();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
  uint16_t count;
  uint16_t queueSize;
  uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
  char name_s[16];
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer
#define STATE_RUN        5 // Running state

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

#define MAX_CHARS 80
#define MAX_FIELDS 10
uint8_t field=0;
uint8_t type[MAX_FIELDS];
uint8_t pos[MAX_FIELDS];
char str[MAX_CHARS+1];
char* str2;
char s[20];
char s1[20];
uint16_t count=0;
uint8_t valid1=0;
struct semaphore *pSemaphore;
uint32_t time1;
uint32_t time2;
uint32_t totaltime;
uint8_t val=0;
bool prin=false;
bool preemptive=false;

struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t priority;              // 0=highest, 7=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint32_t ticks;                // ticks until sleep complete
  char name[16];                 // name of task used in ps command
  char name1[16];
  void *semaphore;               // pointer to the semaphore that is blocking the thread
  uint8_t skipCount;
  uint32_t b;
  uint32_t processTime;
  uint32_t taskTime;
  uint32_t cpuPercentage;
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{
  uint8_t i;
  // no tasks running
  taskCount = 0;
  val++;
  // clear out tcb records
  for (i = 0; i < MAX_TASKS; i++)
  {
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0;
  }
  // REQUIRED: initialize systick for 1ms system timer
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;


}

void rtosStart()
{
  // REQUIRED: add code to call the first task to be run
  _fn fn;

  taskCurrent = rtosScheduler();
  sysSP=Getsp();
  Setsp((uint32_t)tcb[taskCurrent].sp);
  tcb[taskCurrent].state= STATE_READY;
  fn= tcb[taskCurrent].pid;
   (*fn)();


  // Add code to initialize the SP with tcb[task_current].sp;
}

void Setsp(uint32_t value)
{
    __asm("              ADD  SP, #0x08");
    __asm("              MOV   SP, R0 ");
    __asm("              BX    LR     ");
}

uint32_t Getsp()
{
    __asm("              SUB  SP, #0x08");
    __asm("              MOV  R0, SP" );
    __asm("              BX    LR     ");
}

bool createThread(_fn fn, char name[], int priority)
{
  __asm("    SVC #19");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm("    SVC #18");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t x;
    for(x=0;x<MAX_TASKS;x++)
    {
        if(tcb[x].pid == fn)
        {
            tcb[x].priority=priority;
            tcb[x].currentPriority=priority;
        }
    }
}

struct semaphore* createSemaphore(char name_s[],uint8_t count)
{
  struct semaphore *pSemaphore = 0;
  if (semaphoreCount < MAX_SEMAPHORES)
  {
    pSemaphore = &semaphores[semaphoreCount++];
    pSemaphore->count = count;
    strcpy(pSemaphore->name_s, name_s);
  }
  return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
  // push registers, call scheduler, pop registers, return to new function
    __asm("    SVC #14");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
  // push registers, set state to delayed, store timeout, call scheduler, pop registers,
  // return to new function (separate unrun or ready processing)
    __asm("    SVC #15");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm("    SVC #16");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm("    SVC #17");
}

// REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{

  bool ok;
  static uint8_t task = 0xFF;
  ok = false;
  while (!ok)
  {
    task++;
    if (task >= MAX_TASKS)
      task = 0;
    ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    if(ok)
    {
        if( tcb[task].skipCount >= tcb[task].priority)
        {
            tcb[task].skipCount=0;
            ok=1;
        }

        else
        {
            tcb[task].skipCount++;
            ok=0;
        }
    }
  }
  return task;
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{

    uint8_t x,i,alpha=0.9;
    for(x=0;x<MAX_TASKS; x++)
    {
        if(tcb[x].state == STATE_DELAYED)
        {
            tcb[x].ticks--;
            if(tcb[x].ticks==0)
            tcb[x].state = STATE_READY;
        }
    }
    count++;
    if(count==1000)
    {
        count=0;
        for(i=0;i<MAX_TASKS;i++)
        {
            tcb[i].b=alpha*tcb[i].b+(1-alpha)*tcb[i].taskTime;
            tcb[i].cpuPercentage=(((float)tcb[i].b*100)/totaltime)*100;
            tcb[i].taskTime=0;
            tcb[i].processTime=0;
        }
        totaltime=0;
    }

    if(preemptive)
    {
        tcb[taskCurrent].state= STATE_READY;
        NVIC_INT_CTRL_R= NVIC_INT_CTRL_PEND_SV ;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    __asm("         ADD SP,#8");
    __asm("         PUSH {R4-R11}" );
    NVIC_INT_CTRL_R= NVIC_INT_CTRL_UNPEND_SV ;
    tcb[taskCurrent].sp=(void*) Getsp();


    Setsp(sysSP);
    time2=TIMER1_TAV_R;
    if(val==1)
    {
        time2=0;
        val++;
    }
    tcb[taskCurrent].processTime=time1-time2;
    tcb[taskCurrent].taskTime+=tcb[taskCurrent].processTime;
    totaltime+= tcb[taskCurrent].processTime;
    taskCurrent = rtosScheduler();
    time1=TIMER1_TAV_R;
    if(tcb[taskCurrent].state == STATE_READY)
    {
       Setsp((uint32_t)tcb[taskCurrent].sp);

        __asm("           POP {R4-R11}" );
        __asm("            POP {R1}" );
        __asm("            POP {R2}" );
        __asm("            POP {R3}" );
        __asm("            POP {LR}" );
        tcb[taskCurrent].state= STATE_RUN;

    }

    if(tcb[taskCurrent].state== STATE_UNRUN)
    {
        Setsp((uint32_t)tcb[taskCurrent].sp);
        tcb[taskCurrent].state= STATE_RUN;
        __asm("            MOV R0, #0x01000000" );
        __asm("            PUSH {R0}" );
        PushPC(tcb[taskCurrent].pid);
        __asm("            PUSH {R0}" );
        __asm("            PUSH {R12}" );
        __asm("            PUSH {R0-R3}" );


        __asm("    MOV R0, #0xFFFF");
        __asm("    MOV R1, #0xFFF9");
        __asm("    ROR R0, #0x10");
        __asm("     ORR R0,R0,R1");
        __asm("    MOV LR,R0");
        __asm("    BX    LR     ");

    }
    __asm("              BX    LR     ");

}

uint32_t PushPC(value)
{
    __asm("         ADD SP, #0x08");
    __asm("            PUSH {R0}" );
    __asm("       SUB SP, #0x08");

}

uint8_t getSVno()
{

}

uint32_t getReg0()
{

}
char* getReg1()
{
    __asm("    MOV R0, R1");
}
uint32_t getReg2()
{
    __asm("    MOV R0, R2");
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    uint32_t reg0,reg2;
    char* reg1;
    reg0=getReg0();
    reg1=getReg1();
    reg2=getReg2();
    uint8_t sv,x,k,i,j;
    __asm("    MOV R0, SP");
    __asm("    LDR R0, [R0, #64]");
    __asm("    LDRB R0, [R0, #-2]");

    sv=getSVno();
    bool ok = false;
    bool found = false;

    switch(sv)
    {
    case 14://yield
        tcb[taskCurrent].state= STATE_READY;
        NVIC_INT_CTRL_R= NVIC_INT_CTRL_PEND_SV ;
        break;
    case 15://sleep
        tcb[taskCurrent].state = STATE_DELAYED;
        tcb[taskCurrent].ticks = reg0;
        NVIC_INT_CTRL_R= NVIC_INT_CTRL_PEND_SV ;
        break;
    case 16://wait
           pSemaphore = (struct semaphore *) reg0;
           if(pSemaphore->count>0)
                   pSemaphore->count--;
           else
           {

                   pSemaphore->processQueue[pSemaphore->queueSize]=(uint32_t)tcb[taskCurrent].pid;
                   pSemaphore->queueSize++;
                   tcb[taskCurrent].state= STATE_BLOCKED;
                   tcb[taskCurrent].semaphore= pSemaphore;
                   if(prin)
                   {
                       for(x=0;x< MAX_TASKS; x++)
                       {
                           if(tcb[taskCurrent].semaphore==tcb[x].semaphore)
                           {
                              if (tcb[x].priority > tcb[taskCurrent].priority)
                              {
                                  tcb[x].priority = tcb[taskCurrent].priority;

                              }
                           }
                       }
                   }
                   else
                   {
                       for(k=0;k< MAX_TASKS; k++)
                       {
                           if(tcb[k].priority == tcb[taskCurrent].priority)
                               tcb[taskCurrent].priority = tcb[taskCurrent].currentPriority;
                       }

                   }
                   NVIC_INT_CTRL_R= NVIC_INT_CTRL_PEND_SV ;
                   break;
           }
           break;
    case 17: //post
        pSemaphore = (struct semaphore *) reg0;
        pSemaphore->count++;
        if((pSemaphore->count > 0) && (pSemaphore->queueSize > 0))
        {

            for(x=0;x<MAX_TASKS;x++)
            {
                tcb[x].priority=tcb[x].currentPriority;
                if(pSemaphore->processQueue[pSemaphore->queueSize-1]==(uint32_t)tcb[x].pid)
                {
                    tcb[x].state=STATE_READY;
                    pSemaphore->count--;
                    pSemaphore->queueSize--;
                    break;

                }
            }

        }
        tcb[taskCurrent].state= STATE_READY;
        NVIC_INT_CTRL_R= NVIC_INT_CTRL_PEND_SV ;
        break;
    case 18:// destroy thread
        for (x = 0;x < MAX_TASKS;x++)
        {
            if (tcb[x].pid ==(_fn)reg0)
            {
                pSemaphore=tcb[x].semaphore;
                for(j=0;j<MAX_QUEUE_SIZE;j++)
                {
                    for(i=0;i<taskCount;i++)
                    {
                        if(pSemaphore->processQueue[j]==tcb[i].pid)
                        {
                            pSemaphore->processQueue[j]=0;
                            tcb[i].state=STATE_READY;

                            pSemaphore->queueSize--;
                        }
                    }
                }


                tcb[x].state = STATE_INVALID;
                tcb[x].pid = 0;
                tcb[x].cpuPercentage=0;
                taskCount--;
            }
        }
        break;
    case 19: //create thread



         // REQUIRED: store the thread name
         // add task if room in task list
        if (taskCount < MAX_TASKS)
        {
            // make sure fn not already in list (prevent reentrancy)
            while (!found && (i < MAX_TASKS))
            {
                found = (tcb[i++].pid ==  (_fn)reg0);
            }
            if (!found)
            {
                // find first available tcb record
                i = 0;
                while (tcb[i].state != STATE_INVALID) {i++;}
                tcb[i].state = STATE_UNRUN;
                tcb[i].pid = (_fn)reg0;
                tcb[i].sp = &stack[i][255];
                tcb[i].priority = reg2;
                tcb[i].currentPriority = reg2;
                tcb[i].skipCount = reg2;
                strcpy(tcb[i].name, reg1);
             // increment task count
                taskCount++;
                ok = true;
            }
        }
         // REQUIRED: allow tasks switches again

        break;
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
  // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
  //           5 pushbuttons, and uart
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
       SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

       // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
       // Note UART on port A must use APB
       SYSCTL_GPIOHBCTL_R = 0;

       // Enable GPIO port A,F and C peripherals
       SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOF;

       // Configure LED and pushbutton pins
       GPIO_PORTA_DIR_R |= 0xE0;  // bits 1 and 3 are outputs, other pins are inputs
       GPIO_PORTA_DR2R_R |= 0xE0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
       GPIO_PORTA_DEN_R |= 0xFC;  // enable LEDs and pushbuttons
       GPIO_PORTA_PUR_R |= 0x1C;


       GPIO_PORTB_DIR_R |= 0x10;  // bits 1 and 3 are outputs, other pins are inputs
       GPIO_PORTB_DR2R_R |= 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
       GPIO_PORTB_DEN_R |= 0x50;  // enable LEDs and pushbuttons
       GPIO_PORTB_PUR_R |= 0x40;


       GPIO_PORTF_DEN_R |= 0x1E;  // enable LEDs and pushbuttons(red, blue, green)
       GPIO_PORTF_PUR_R |= 0x10;  // enable internal pull-up for push button

       GPIO_PORTF_DIR_R |= 0x0E;  // bits 1,2 and 3 are outputs, other pins are inputs
       GPIO_PORTF_DR2R_R |= 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)

       NVIC_ST_CTRL_R = 0;                  //Disable SysTick during configuration
       NVIC_ST_RELOAD_R = 0x00009C40;      //Reload value configured for 1ms
       NVIC_ST_CURRENT_R = 0;              //Current value is reset to 0

       // Configure GPIO pins for UART0
         SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
         GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
         GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
         GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

         // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
         UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
         UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
         UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
         UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
         UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
         UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

         // Configure Timer 1 as the time base
             SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
             TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
             TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
             TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down) TIMER_TAMR_TACDIR
             TIMER1_TAV_R= 0xFFFFFFFF;
             TIMER1_CTL_R |= TIMER_CTL_TAEN;

}


uint8_t strlen1(char *str)
{
    uint8_t i;
    for(i=0;str[i]!='\0';i++);
    return i;
}
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen1(str); i++)
      putcUart0(str[i]);
}
void putnUart0(uint16_t n)
{
    char l3,l2,l1,l0;
    uint16_t a;
    a= n%10;
    l3=a+48;
    a=(n/10)%10;
    l2=a+48;
    a=(n/100)%10;
    l1=a+48;
    a=(n/1000)%10;
    l0=a+48;
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = l0;
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = l1;
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = l2;
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = l3;

}

void floatUart0(uint32_t n)
{

    uint32_t x=n;
    char l3,l2,l1,l0;
    uint16_t a;
    a= x%10;
    l3=a+48;
    a=(x/10)%10;
    l2=a+48;
    a=(x/100)%10;
    l1=a+48;
    a=(x/1000)%10;
    l0=a+48;
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = l0;
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = l1;
    putsUart0(".");
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = l2;
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = l3;


}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
    {
        yield();
    }
    return UART0_DR_R & 0xFF;
}


// Function that returns string that is compatible with backspace, space and carriage return
void getsUart0()
{
    uint8_t count=0;
    char c;
    start:  c= getcUart0() ;
    if (c==8)
     {
         if(count>0)
         {
             count--;
           goto start ;
         }
         goto start ;
     }

     else if (c==13)
     {
         x: str[count++]=0;
          return;
     }
      if(c>=' ')
      {
          str[count++]=c;
      }
      else
      {
          goto start;
      }
      if(count> MAX_CHARS)
      {
          goto x;
      }
      else
      {
      goto start ;
      }

}

    //Function that defines the differentiates delimiters and alphanumeric characters and defines fields, position and type of string
void isstring(char* str)
{
    field=0;

    uint8_t i, flag=0;
    uint8_t l=strlen1(str);
    for(i=0; i<l; i++)
    {
        // Condition for Alphanumeric characters
        s[i]=str[i];
        if((str[i]>='0' && str[i]<= '9') || (str[i]>='A' && str[i]<='Z')||(str[i]>='a' && str[i]<='z'))
            {
            if((str[i]>='a' && str[i]<='z'))        //defining case insensitivity
            {
                    str[i]=str[i]-32;
            }

            flag++;
            if(flag==1)
                {
                field++;                                                        // Setting Field value
                pos[field-1]=i;                                                 // Setting position value

                // Defining the type of string
                if ((str[i]>='A' && str[i]<='Z')||(str[i]>='a' && str[i]<='z'))
                    type[field-1]='a';
                else
                    type[field-1]='n';
                }
            }
        // Condition for delimiters and converting them to zero
        else
            {
                    flag=0;
                    str[i]=0;

            }
    }

}

// Function for checking the entered data
bool iscommand(char strcmd[], uint8_t min_args)
{
    if((strcmp(&str[pos[0]], strcmd)==0) && (field>= min_args))
        return true;
    else
        return false;
}

// Function to get the entered string
char* getstring(uint8_t field)
{
        if (type[field]=='a')
            return &str[pos[field]];
        else
            valid1 =1;
        return 0;
}

char* getstring1(uint8_t field)
{
    return &s[pos[field]];
}
// Function to get the entered number
int16_t getnumber(uint8_t field)
{
    uint16_t x;
    if (type[field]=='n')
    {
        x= atoi(&str[pos[field]]);
        return x;
    }
    else
        valid1 =1;
    return valid1;
}

void command()
{
    uint8_t x,i,j,k;
    if (iscommand("PS", 0))
    {
        putsUart0(" PID \t PROCESS NAME \t CPU PERCENTAGE \t  PROCESS STATE \n\n\r");
        for(x=0;x<9;x++)
        {
            putsUart0(" ");
            putnUart0((uint32_t)tcb[x].pid);
            putcUart0('\t');
            putsUart0(tcb[x].name);
            for(i=0;i<16-strlen1(tcb[x].name);i++)
                putsUart0(" ");
            putcUart0('\t');

            floatUart0(tcb[x].cpuPercentage);
            for(i=0;i<14;i++)
                putsUart0(" ");
            putcUart0('\t');
            if(tcb[x].state==0)
                putsUart0("invalid");
            else if(tcb[x].state==1)
                putsUart0("unrun  ");
            else if(tcb[x].state==2)
                putsUart0("ready  ");
            else if(tcb[x].state==3)
                putsUart0("blocked");
            else if(tcb[x].state==4)
                putsUart0("delayed");
            else if(tcb[x].state==5)
                putsUart0("run    ");
            putcUart0('\n');

            putcUart0('\r');
        }
        putcUart0('\n');
    }

    if( iscommand("REBOOT",0))
    {
        putsUart0("restarting.... \n\r");
        NVIC_APINT_R= NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
    }

    if( iscommand("PIDOF",1))
    {
        uint8_t x,i;
        char* string;
        for(x=0;x<MAX_TASKS; x++)
        {
            string=getstring1(1);
            if(strcmp(string,tcb[x].name)==0)
            {
                putsUart0("The PID of the Task is..");
                putnUart0((uint32_t)tcb[x].pid);
                for(i=0;i<20;i++)
                    s[i]=0;
                putcUart0('\n');
                putcUart0('\r');
                break;
            }
        }
    }

    if (iscommand ("PI",1))
    {
        if(strcmp(getstring(1),"ON")==0)
        {
            putsUart0("pi on");
            putcUart0('\n');
            putcUart0('\r');
            prin=true;
        }
        else if(strcmp(getstring(1),"OFF")==0)
        {
            putsUart0("pi off");
            putcUart0('\n');
            putcUart0('\r');
            prin=false;
        }
        else
            putsUart0("Error in 'pi' command \n\r");

    }

    if (iscommand ("PREEMP",1))
          {
              if(strcmp(getstring(1),"ON")==0)
              {
                  putsUart0("Preemption on");
                  putcUart0('\n');
                  putcUart0('\r');
                  preemptive=true;
              }
              else if(strcmp(getstring(1),"OFF")==0)
              {
                  putsUart0("Preemption off");
                  putcUart0('\n');
                  putcUart0('\r');
                  preemptive=false;
              }
              else
                  putsUart0("Error in 'preemp' command \n\r");

          }


    if( iscommand("IPCS",0))
    {
        uint8_t x,i;
        putsUart0(" ");
        putsUart0("\n\r SEMAPHORE NAME \tCOUNT \t   QUEUE SIZE \t PROCESS QUEUE\n\n\r");
        for(x=0; x< semaphoreCount; x++)
        {
            putsUart0(" ");
            putsUart0(semaphores[x].name_s);
            for(i=0;i<19-strlen1(semaphores[x].name_s);i++)
                putsUart0(" ");
            putcUart0('\t');
            putnUart0(semaphores[x].count);
            for(i=0;i<5-semaphores[x].count;i++)
                putsUart0(" ");
            putcUart0('\t');
            putnUart0(semaphores[x].queueSize);
            putcUart0('\t');
            putcUart0('\t');
            for(i=0;i<semaphores[x].queueSize;i++)
                putnUart0(semaphores[x].processQueue[i]);
            putcUart0('\n');
            putcUart0('\r');
        }
        putcUart0('\n');
    }

    if( iscommand("KILL",0))
    {
        uint8_t x;
        for(x=0;x<taskCount;x++)
        {
            if(getnumber(1)==tcb[x].pid)
                destroyThread((_fn)tcb[x].pid);
        }
        for(i=0;i<20;i++)
            s[i]=0;

    }

    for(j=0;j<strlen1(s);j++)
    {

        if(s[j]==0x26)
        {
            s[strlen1(s)-1]=0;
            str2= getstring1(0);
            for(k=0;k<MAX_TASKS;k++)
            {
                if(strcmp(str2,tcb[k].name)==0)
                {
                    if(strcmp(tcb[k].name,"Idle")==0)
                    {
                        createThread(idle, tcb[k].name, tcb[k].priority);
                        taskCount--;
                    }
                    if(strcmp(tcb[k].name,"LengthyFn")==0)
                    {
                        createThread(lengthyFn, tcb[k].name, tcb[k].priority);
                        taskCount--;
                    }
                    if(strcmp(tcb[k].name,"OneShot")==0)
                    {
                        createThread(oneshot, tcb[k].name, tcb[k].priority);
                        post(flashReq);
                        taskCount--;
                    }
                    if(strcmp(tcb[k].name,"ReadKeys")==0)
                    {
                        createThread(readKeys, tcb[k].name, tcb[k].priority);
                        taskCount--;
                    }
                    if(strcmp(tcb[k].name,"Debounce")==0)
                    {
                        createThread(debounce, tcb[k].name, tcb[k].priority);
                        taskCount--;
                    }
                    if(strcmp(tcb[k].name,"Important")==0)
                    {
                        createThread(important, tcb[k].name, tcb[k].priority);
                        taskCount--;
                    }
                    if(strcmp(tcb[k].name,"Uncoop")==0)
                    {
                        createThread(uncooperative, tcb[k].name, tcb[k].priority);
                        taskCount--;
                    }
                    if(strcmp(tcb[k].name,"Shell")==0)
                    {
                        createThread(shell, tcb[k].name, tcb[k].priority);
                        taskCount--;
                    }
                    if(strcmp(tcb[k].name,"Flash4Hz")==0)
                    {
                        createThread(flash4Hz, tcb[k].name, tcb[k].priority);
                        taskCount--;
                    }
                    putsUart0("The thread is created ");
                    putcUart0('\n');
                    putcUart0('\r');
                }
            }
        }
    }
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                                // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    uint8_t rval=0;
    if(!PB1){rval= 1;}
    if(!PB2){rval=rval+2;}
    if(!PB3){rval=rval+4;}
    if(!PB4){rval=rval+ 8;}
    if(!PUSH_BUTTON){rval=rval+16;}
    return rval;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    //waitMicrosecond(1000000);
    yield();
  }
}

void idle2()
{
  while(true)
  {
    YELLOW_LED = 1;
    waitMicrosecond(1000);
    YELLOW_LED = 0;
    //waitMicrosecond(1000000);
    yield();
  }
}


void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);
  }
}

void oneshot()
{
  while(true)
  {
    wait(flashReq);
    YELLOW_LED = 1;
    sleep(1000);
    YELLOW_LED = 0;
  }
}

void partOfLengthyFn()
{
  // represent some lengthy operation
  waitMicrosecond(1000);
  // give another process a chance to run
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    wait(resource);
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
    post(resource);
  }
}

void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
      createThread(flash4Hz, "Flash4Hz", 0);
    }
    if ((buttons & 8) != 0)
    {
      destroyThread(flash4Hz);
    }
    if ((buttons & 16) != 0)
    {
      setThreadPriority(lengthyFn, 4);
    }
    yield();
  }
}

void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(keyPressed);
    count = 10;
    while (count != 0)
    {
      sleep(10);
      if (readPbs() == 0)
        count--;
      else
        count = 10;
    }
    post(keyReleased);
  }
}

void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {

    }
    yield();
  }
}

void important()
{
    while(true)
    {
      wait(resource);
      BLUE_LED = 1;
      sleep(1000);
      BLUE_LED = 0;
      post(resource);
    }
}

void shell()
{
  while (true)
  {
    // REQUIRED: add processing for the shell commands through the UART here

      getsUart0();
      putsUart0(str);
      putcUart0('\n');
      putcUart0('\r');
      isstring(str);
      command();

  }
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
  bool ok;

  // Initialize hardware
  initHw();
  rtosInit();

  // Power-up flash
  GREEN_LED = 1;
  waitMicrosecond(250000);
  GREEN_LED = 0;
  waitMicrosecond(250000);
  putsUart0("RTOS_Spring18 \n\r");
  // Initialize semaphores
  keyPressed = createSemaphore("keypressed",1);
  keyReleased = createSemaphore("keyreleased",0);
  flashReq = createSemaphore("flashreq",5);
  resource = createSemaphore("resource",1);

  // Add required idle process
  ok =  createThread(idle, "Idle", 7);
 // ok &=  createThread(idle2, "Idle2", 1);
  // Add other processes
  ok &= createThread(lengthyFn, "LengthyFn", 6);
  ok &= createThread(flash4Hz, "Flash4Hz", 2);
  ok &= createThread(oneshot, "OneShot", 2);
  ok &= createThread(readKeys, "ReadKeys", 6);
  ok &= createThread(debounce, "Debounce", 6);
   ok &= createThread(important, "Important", 0);
  ok &= createThread(uncooperative, "Uncoop", 5);
  ok &= createThread(shell, "Shell", 4);


  // Start up RTOS
  if (ok)
    rtosStart(); // never returns
  else
    RED_LED = 1;

return 0;
}
