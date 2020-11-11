#include "dinner.h"

extern void main_philosopher();

#define NUM_PHILOSOPHERS 16

int pipes[NUM_PHILOSOPHERS];
int philosophers[NUM_PHILOSOPHERS];

void initaliseProcs() {
  pid_t waiterPID = getPid();

  // create new processes for each philosopher
  for(int i = 0; i < NUM_PHILOSOPHERS; i++) {
      philosophers[i] = fork();
      // exec if return value is 0, showing fork executed properly
      if (philosophers[i] == 0) {
          exec(&main_philosopher);
      }

      pipes[i] = createPipe(waiterPID, philosophers[i]);
  }


}

void main_dinner() {
    pid_t waiterPID = getPid();
    initaliseProcs();

    // 2 groups of philosophers eating and thinking, efficient solution as
    // each round 8 eat and 8 think which also means no starvation in the solution

    // inital for loop to let them eat or think
    for(int i = 0; i < NUM_PHILOSOPHERS; i++) {
       if (i%2 == 0) {
         writePipe(i, EAT);
       }
       else {
         writePipe(i, THINK);
       }
    }
    yield();

    // infinite loop to determine what they should do depending on their previous action
    while(1) {
      for(int j = 0; j < NUM_PHILOSOPHERS; j++) {
        int message = readPipe(j);
        if(message == EAT) {
          writePipe(j, THINK);
        }
        else {
          writePipe(j, EAT);
        }
      }
      yield();
    }

    exit(EXIT_SUCCESS);
}
