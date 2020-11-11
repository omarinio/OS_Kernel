#include "dinner.h"

void main_philosopher() {
    int philosopher = getPid() - 2;
    char philo[2];
    // converting int to char to allow printing
    itoa(philo, philosopher);
    int status = 0;
    while(1) {
      // checking status of pipe to see if philosopher should eat or think
      status = readPipe(philosopher-1);
      if (status == EAT) {
        write(STDOUT_FILENO, "P", 1);
        write(STDOUT_FILENO, philo, 2);
        write(STDIN_FILENO, " EATS", 5);
        write( STDOUT_FILENO, "\n", 2 );
      }
      if (status == THINK) {
        write(STDOUT_FILENO, "P", 1);
        write(STDOUT_FILENO, philo, 2);
        write(STDIN_FILENO, " THINKS", 7);
        write( STDOUT_FILENO, "\n", 2 );
      }
      yield();
    }

    exit(EXIT_SUCCESS);
}
