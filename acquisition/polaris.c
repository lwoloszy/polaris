#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>

#include "udp.h"
#include "polaris.h"
#include "systemcrc.h"
#include "commandhandling.h"
#include "commandconstruction.h"

void sig_handler(int sig);

int main()
{

  int fd;
  struct sigaction action;

  if ((nPort = open(DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1) {
    printf("Cannot open port");
    return 0;
  }

  /* To store previous frame's information */
  if ((previousPos= malloc(sizeof(POSITION))) == NULL) {
      printf("Cannot allocate memory\n");
      return 1;
  }

  /* When we quit, make sure we stop tracking (turn off infrared illuminators */
  memset(&action, 0, sizeof(action));
  action.sa_handler = &sig_handler;
  sigemptyset(&action.sa_mask);
  sigaddset(&action.sa_mask, SIGSEGV);
  action.sa_flags = SA_SIGINFO;
  sigaction(SIGINT,&action,NULL);
  sigaction(SIGQUIT,&action,NULL);

  /* Shared memory stuff for the plotting function */
  //shm_unlink("/CURRENT_POSITION");

  if((fd = shm_open("/CURRENT_POSITION", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR)) == -1) {
    printf("Cannot create shared memory area\n");
    return(1);
  }

  if ((ftruncate(fd, sizeof(POSITION)))==-1) {
      printf("Cannot ftruncate shared memory area\n");
      return 1;
  }

  if((currentPos = mmap(0, sizeof(POSITION), PROT_READ|PROT_WRITE,
  			MAP_SHARED, fd, 0)) == (POSITION *)-1) {
    printf("Cannot map shared memory area\n");
    return(1);
  }

  /* UDP stuff to send to Rex */
  udp_close();
  udp_open(thisIP, otherIP, udpPORT);

  /* Polaris tracking stuff here */
  if (nBasicSetup())
    return 1;

  if (nStartTracking())
    return 1;

  nTrackMarkerLoop();

  return 0;
}

void sig_handler(int sig)
{

  nStopTracking();
  free(previousPos);
  //shm_unlink("/CURRENT_POSITION");
  udp_close();
  fprintf(stderr, "\nStopping tracking\n");
  kill(getpid(), SIGKILL);

}
