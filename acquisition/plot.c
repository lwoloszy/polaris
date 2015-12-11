#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>

#include "/usr/local/dislin/dislin.h"

/* Dimensions of box to plot (mm) */
/*#define XHALF 800
#define YHALF 1000
#define ZHALF 1300
#define STEP  400*/

#define XHALF 100
#define YHALF 500
#define ZHALF 500
#define STEP 100

/* Dimensions of Polaris camera (mm) */
#define XPOLARIS 86
#define YPOLARIS 613
#define ZPOLARIS 104

/* Dimensions of monitor */
#define XMONITOR 40
#define YMONITOR 600
#define ZMONITOR 340

/* Center of monitor */
#define XMONITORCEN -5.0
#define YMONITORCEN 54.0
#define ZMONITORCEN -1400.0

#define SYMBOLSIZE 200

typedef struct {
  int flagError;
  int frame;
  float x;
  float y;
  float z;

  float responseBox1x;
  float responseBox1y;
  float responseBox1z;
  float responseBox2x;
  float responseBox2y;
  float responseBox2z;
  float responseBoxHalfExtentX;
  float responseBoxHalfExtentY;
  float responseBoxHalfExtentZ;

  float fixBoxx;
  float fixBoxy;
  float fixBoxz;
  float fixBoxHalfExtentX;
  float fixBoxHalfExtentY;
  float fixBoxHalfExtentZ;
} POSITION;
typedef POSITION * POSITION_P;

int main(int argc, char *argv[]) {

  POSITION_P currentPos, previousPos,  localCurrentPos;
  POSITION tempPosStruct;
  float viewx, viewy, viewz;
  int nFirstTime=1;
  int fd;
  char ccFrame[17],cpFrame[17];
  char positionText[50], prevPositionText[50];

  struct timespec t;
  t.tv_sec=0;
  t.tv_nsec=50000000;

  /* Shared memory stuff for the plotting function */
  if((fd = shm_open("/CURRENT_POSITION", O_RDONLY, S_IRUSR)) == -1) {
    printf("Cannot create shared memory area\n");
    return(1);
  }

  if((currentPos = mmap(0, sizeof(POSITION), PROT_READ,
  			MAP_SHARED, fd, 0)) == (POSITION *)-1) {
    printf("Cannot map shared memory area\n");
    return(1);
  }

  previousPos = malloc(sizeof(POSITION));

  viewx = strtof(*++argv, NULL);
  viewy = strtof(*++argv, NULL);
  viewz = strtof(*++argv, NULL);

  while (1) {

    if (nFirstTime) {
      setpag("da4p");      // page size = 2100*2970
      //      window(0,0,525,750); // just played around with it a bit
      window(0,0,500,800); // just played around with it a bit
      metafl("xwin");
      disini();

      errmod("Warnings", "Off");
      color("white");
      axspos(0, 2500);
      axslen(2100, 2100);

      labl3d("horizontal");
      x11fnt("-Adobe-Helvetica-Bold-R-Normal-","Standard");
      name("X (mm)", "x");
      name("Y (mm)", "y");
      name("Z (mm)", "z");

      axis3d(1,(float) YHALF/XHALF, (float) ZHALF/XHALF);
      view3d(viewx, viewy, viewz,"abs");
      graf3d(-1*XHALF+XMONITORCEN, XHALF+XMONITORCEN, -1*XHALF+XMONITORCEN, STEP,
      	     -1*YHALF+YMONITORCEN, YHALF+YMONITORCEN, -1*YHALF+YMONITORCEN, STEP,
      	     -1*ZHALF+ZMONITORCEN, ZHALF+ZMONITORCEN, -1*ZHALF+ZMONITORCEN, STEP);
      clip3d("none");
      hsym3d(.01);

      previousPos->x = currentPos->x;
      previousPos->y = currentPos->y;
      previousPos->z = currentPos->z;

      nFirstTime = 0;
    }

    tempPosStruct = *currentPos;
    localCurrentPos = &tempPosStruct;

     // "Erase" plot from previous frame (text, symbol, and responseBox overwrite)
    color("black");
    hsymbl(SYMBOLSIZE);
    vtx3d(&previousPos->x, &previousPos->y, &previousPos->z, 1, "points");
    messag(cpFrame, 750, 2800);
    messag(prevPositionText, 750, 2900);
    //    plat3d(previousPos->responseBox1x, previousPos->responseBox1y, previousPos->responseBox1z,
    //	   previousPos->responseBoxHalfExtentX*2.25,"cube");
    //plat3d(previousPos->responseBox2x, previousPos->responseBox2y, previousPos->responseBox2z,
    //	   previousPos->responseBoxHalfExtentX*2.25,"cube");

    quad3d(previousPos->fixBoxx, previousPos->fixBoxy, previousPos->fixBoxz,
	   previousPos->fixBoxHalfExtentX*2.25,
	   previousPos->fixBoxHalfExtentY*2.25,
	   previousPos->fixBoxHalfExtentZ*2.25);
    quad3d(previousPos->responseBox1x, previousPos->responseBox1y, previousPos->responseBox1z,
	   previousPos->responseBoxHalfExtentX*2.25,
	   previousPos->responseBoxHalfExtentY*2.25,
	   previousPos->responseBoxHalfExtentZ*2.25);
    quad3d(previousPos->responseBox2x, previousPos->responseBox2y, previousPos->responseBox2z,
	   previousPos->responseBoxHalfExtentX*2.25,
	   previousPos->responseBoxHalfExtentY*2.25,
	   previousPos->responseBoxHalfExtentZ*2.25);

    quad3d(XMONITORCEN,YMONITORCEN,ZMONITORCEN, XMONITOR*1.25, YMONITOR*1.25, ZMONITOR*1.25);
    quad3d(0,0,(float) ZPOLARIS/2,XPOLARIS*1.25, YPOLARIS*1.25, ZPOLARIS*1.25);

    // Check whether the marker is within the bounds of hand fixation
    color("blue");
    if ((abs(localCurrentPos->x - localCurrentPos->fixBoxx) < localCurrentPos->fixBoxHalfExtentX) &&
    	(abs(localCurrentPos->y - localCurrentPos->fixBoxy) < localCurrentPos->fixBoxHalfExtentY) &&
    	(abs(localCurrentPos->z - localCurrentPos->fixBoxz) < localCurrentPos->fixBoxHalfExtentZ))
      tprval(1.0);
    else
      tprval(0.5);
    tprini();
    //    plat3d(localCurrentPos->responseBox1x, localCurrentPos->responseBox1y, localCurrentPos->responseBox1z,
    //	   localCurrentPos->responseBoxHalfExtentX*2,"cube");
    quad3d(localCurrentPos->fixBoxx, localCurrentPos->fixBoxy, localCurrentPos->fixBoxz,
    	   localCurrentPos->fixBoxHalfExtentX*2,
	   localCurrentPos->fixBoxHalfExtentY*2,
	   localCurrentPos->fixBoxHalfExtentZ*2);
    tprfin();


    // Check whether the marker is within the bounds of one or the other response windows
    color("green");
    if ((abs(localCurrentPos->x - localCurrentPos->responseBox1x) < localCurrentPos->responseBoxHalfExtentX) &&
    	(abs(localCurrentPos->y - localCurrentPos->responseBox1y) < localCurrentPos->responseBoxHalfExtentY) &&
    	(abs(localCurrentPos->z - localCurrentPos->responseBox1z) < localCurrentPos->responseBoxHalfExtentZ))
      tprval(1.0);
    else
      tprval(0.25);
    tprini();
    //    plat3d(localCurrentPos->responseBox1x, localCurrentPos->responseBox1y, localCurrentPos->responseBox1z,
    //	   localCurrentPos->responseBoxHalfExtentX*2,"cube");
    quad3d(localCurrentPos->responseBox1x, localCurrentPos->responseBox1y, localCurrentPos->responseBox1z,
    	   localCurrentPos->responseBoxHalfExtentX*2,
	   localCurrentPos->responseBoxHalfExtentY*2,
	   localCurrentPos->responseBoxHalfExtentZ*2);
    tprfin();

    color("red");
    if ((abs(localCurrentPos->x - localCurrentPos->responseBox2x) < localCurrentPos->responseBoxHalfExtentX) &&
    	(abs(localCurrentPos->y - localCurrentPos->responseBox2y) < localCurrentPos->responseBoxHalfExtentY) &&
    	(abs(localCurrentPos->z - localCurrentPos->responseBox2z) < localCurrentPos->responseBoxHalfExtentZ))
      tprval(1.0);
    else
      tprval(.25);
    tprini();
    //    plat3d(localCurrentPos->responseBox2x, localCurrentPos->responseBox2y, localCurrentPos->responseBox2z,
    //	   localCurrentPos->responseBoxHalfExtentX*2,"cube");
    quad3d(localCurrentPos->responseBox2x, localCurrentPos->responseBox2y, localCurrentPos->responseBox2z,
    	   localCurrentPos->responseBoxHalfExtentX*2,
	   localCurrentPos->responseBoxHalfExtentY*2,
	   localCurrentPos->responseBoxHalfExtentZ*2);
    tprfin();

    // Redraw monitor
    color("white");
    tprval(0.25);
    tprini();
    quad3d(XMONITORCEN,YMONITORCEN,ZMONITORCEN, XMONITOR, YMONITOR, ZMONITOR);
    tprfin();

    // Redraw Polaris camera
    color("white");
    tprval(0.5);
    tprini();
    quad3d(0,0,(float) ZPOLARIS/2,XPOLARIS, YPOLARIS, ZPOLARIS);
    tprfin();

    // Frame number
    color("white");
    memset(ccFrame, 0, sizeof(ccFrame));
    sprintf(ccFrame, "Frame: %10d", localCurrentPos->frame);
    messag(ccFrame, 750, 2800);

    // Position
    color("white");
    memset(positionText, 0, sizeof(positionText));
    sprintf(positionText, "Position: %4.2f %4.2f %4.2f",
	    localCurrentPos -> x,
	    localCurrentPos -> y,
	    localCurrentPos -> z
	    );
    messag(positionText, 750, 2900);

    // Is Polaris seeing the marker? 1 indicates we got nothing, 2 that there was a phantom marker
    // so we end up doing some pre-processing and 0 means ALL ABOARD
    if (localCurrentPos->flagError == 1)
      color("red");
    else if (localCurrentPos->flagError == 2)
      color("yellow");
    else
      color("white");
    box3d();
    // Plot the actual position of marker
    color("white");
    hsymbl(SYMBOLSIZE);
    vtx3d(&localCurrentPos->x, &localCurrentPos->y, &localCurrentPos->z, 1, "points");

    // Send to buffer
    sendbf();

    // Store previous marker position, frame number and  position of response windows
    // These values will be used to erase (black-out) old information, which lets
    // us erase parts of the plot without having to redraw the whole thing on every frame
    previousPos->x = localCurrentPos->x;
    previousPos->y = localCurrentPos->y;
    previousPos->z = localCurrentPos->z;
    previousPos->frame = localCurrentPos->frame;
    memcpy(cpFrame,ccFrame,sizeof(ccFrame));
    memcpy(prevPositionText,positionText,sizeof(positionText));

    previousPos->fixBoxx = localCurrentPos->fixBoxx;
    previousPos->fixBoxy = localCurrentPos->fixBoxy;
    previousPos->fixBoxz = localCurrentPos->fixBoxz;
    previousPos->fixBoxHalfExtentX = localCurrentPos-> fixBoxHalfExtentX;
    previousPos->fixBoxHalfExtentY = localCurrentPos-> fixBoxHalfExtentY;
    previousPos->fixBoxHalfExtentZ = localCurrentPos-> fixBoxHalfExtentZ;

    previousPos->responseBox1x = localCurrentPos->responseBox1x;
    previousPos->responseBox1y = localCurrentPos->responseBox1y;
    previousPos->responseBox1z = localCurrentPos->responseBox1z;
    previousPos->responseBox2x = localCurrentPos->responseBox2x;
    previousPos->responseBox2y = localCurrentPos->responseBox2y;
    previousPos->responseBox2z = localCurrentPos->responseBox2z;
    previousPos->responseBoxHalfExtentX = localCurrentPos->responseBoxHalfExtentX;
    previousPos->responseBoxHalfExtentY = localCurrentPos->responseBoxHalfExtentY;
    previousPos->responseBoxHalfExtentZ = localCurrentPos->responseBoxHalfExtentZ;

    nanosleep(&t, NULL);
  }

  return 0;

}
