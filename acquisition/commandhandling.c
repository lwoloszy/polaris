/*
**
**
** Based on API sample from NDigital
**
**
**
** Shadlen Lab, Luke Woloszyn
**
**
**
**
**
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <math.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/types.h>
#include <asm/types.h>
#include <time.h>

#include "polaris.h"
#include "commandhandling.h"
#include "commandconstruction.h"
#include "systemcrc.h"
#include "udp.h"

/*****************************************************************
Name: nHardwareReset

Inputs:
	None.

Return Value:
	int - 0 if passes, 1 if fail

Description:
        This routine sends a serial break to the system, resetting it.
        In addition, it sets the basic parameters of termios struct.
*****************************************************************/
int nHardwareReset()
{

  struct termios options;

  tcflush(nPort,TCIOFLUSH);

  tcsendbreak(nPort, 500); // Milliseconds

  /* After the system gets powered on, the defaults are:
   *
   *
   *   baudrate 9600,
   *   8 data bits,
   *   1 stop bit,
   *   NO parity
   *   NO hardware handshaking
   *
   *
   Lets implement these on the computer side */


  tcgetattr(nPort, &options);

  // Some basic settings first
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOKE | ECHONL | ISIG); //raw mode
  options.c_iflag &= ~(INPCK | IGNBRK | PARMRK | ISTRIP | IXON | IXOFF); //parity stuff off
  options.c_oflag &= ~OPOST; //no output processing

  // Now the settings we just spoke of
  options.c_cflag &= ~PARENB; // no parity
  options.c_cflag &= ~CSTOPB; // 1 stop bit
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;     // 8 data bits

  cfsetispeed(&options, B9600); // input baud rate = 9600
  cfsetospeed(&options, B9600); // output baud rate = 9600

  // options.c_cc[VMIN] = 1; // read() will wait for at least one byte before returning
  // options.c_cc[VTIME] = 0;

  tcsetattr(nPort, TCSANOW, &options);
  if (nGetResponse())
    return 1;
  if (nCheckReset(szReply))
    return 1;

  return 0;
} /* nHardwareReset */

/*****************************************************************
Name: nSetSystemCommParms

Inputs:
	int nBaudRate - the baud rate to set
	int nDataBits - the data bit setting
	int nParity   - the parity setting
	int nStopBits - the stop bit setting
	int nHardware - whether or not to use hardware handshaking

Return Value:
	int - 0 if fails, else nCheckResponse

Description:
	This routine sets the systems com port parameters, remember
	to immediatley set the computer's com port settings after this
	routine is called.
*****************************************************************/
int nSetSystemCommParms(int nBaudRate,
			int nDataBits,
			int nParity,
			int nStopBits,
			int nHardware)
{

  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "COMM %d%d%d%d%d",
	  nBaudRate,
	  nDataBits,
	  nParity,
	  nStopBits,
	  nHardware);

  if (nSendMessage(szCommand,ADDCRC)) {
    printf("here1 \n");
    return 1;
  }
  if (nGetResponse()) {
    printf("here2 \n");
    return 1;
  }
  if (nCheckResponse(szReply,CHECKCRC)) {
    printf("here3 \n");
    return 1;
  }

  return 0;
} /* nSetSystemComParms */

/*****************************************************************
Name: nSetCompCommParms

Inputs:
	int nBaudRate - the baud rate to set
	int nDataBits - the data bit setting
	int nParity   - the parity setting
	int nStopBits - the stop bit setting
	int nHardware - whether or not to use hardware handshaking

Return Value:
	int - 0 if fails, else 1

Description:
	This routine sets the computer's com port parameters, remember
	to immediatley set the computer's com port settings after the
	system's com port parameters.
*****************************************************************/
int nSetCompCommParms(int nBaudRate,
		      int nDataBits,
		      int nParity,
		      int nStopBits,
		      int nFlowControl)
{

  struct termios options;

  switch( nBaudRate )
    {
    case 0:
      nBaudRate = B9600;
      break;
    case 1:
      printf("Haven't set this up properly yet, will break!\n");
      break;
    case 2:
      nBaudRate = B19200;
      break;
    case 3:
      nBaudRate = B38400;
      break;
    case 4:
      nBaudRate = B57600;
      break;
    case 5:
      nBaudRate = B115200;
      break;
    case 6:
      nBaudRate = B921600;
      break;
    case 7:
      nBaudRate = B19200; //Aliased to 1.2 MBit
      break;
    default:
      nBaudRate = B9600;
      break;
    } /* switch */

  /* Change speed */
  tcgetattr(nPort, &options);
  cfsetispeed(&options, nBaudRate);
  cfsetospeed(&options, nBaudRate);

  tcsetattr(nPort, TCSANOW, &options);

  return 0;
} /* nSetCompCommParms */

/*****************************************************************
Name: nBeepSystem

Inputs:
	int nBeeps - the number of times the system should beep

Return Value:
	int - 0 if passes, 1 if fails

Description:
	This routine sends the beep command to the system
*****************************************************************/
int nBeepSystem(int nBeeps)
{
  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "BEEP %d", nBeeps);

  if (nSendMessage(szCommand,ADDCRC))
    return 1;
  if (nGetResponse())
    return 1;
  if (nCheckResponse(szReply,CHECKCRC))
    return 1;

  return 0;
} /* nBeepSystem */

/*****************************************************************
Name: nInitializeSystem

Inputs:
	None.

Return Value:
	int - 0 if passes, 1 if fails

Description:
    This routine initializes the system by sending INIT
*****************************************************************/
int nInitializeSystem()
{
  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "INIT ");

  if (nSendMessage(szCommand,ADDCRC))
    return 1;
  if (nGetResponse())
    return 1;
  if (nCheckResponse(szReply,CHECKCRC))
    return 1;

  return 0;
} /* nInitializeSystem */

/*****************************************************************
Name: nSetIllumRate

Inputs:
	none

Return Value:
	int - 0 if passes, 1 if fails

Description:
    This routine sets system's illumination rate by sending IRATE
*****************************************************************/
int nSetIllumRate(int nIllumRate)
{
  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "IRATE %d", nIllumRate);

  if (nSendMessage(szCommand,ADDCRC))
    return 1;
  if (nGetResponse())
    return 1;
  if (nCheckResponse(szReply,CHECKCRC))
    return 1;

  return 0;
} /* nSetIllumRate */

/*****************************************************************
Name: nRequestHandle

Inputs:
    none

Return Value:
	int - 0 if passes, 1 if fails

Description:
    This routine requests a handle by sending PHRQ
*****************************************************************/
int nRequestHandle( )
{
  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "PHRQ *********1****");

  if (nSendMessage(szCommand,ADDCRC))
    return 1;
  if (nGetResponse())
    return 1;
  if (nCheckResponse(szReply,CHECKCRC))
    return 1;

  return 0;
} /* nRequestHandle */

/*****************************************************************
Name:				nInitializeHandle

Inputs:
	int nHandle - the handle to be intialized

Return Value:
	int - 0 if passes, 1 if fails

Description:
    This routine initializes the specified handle be sending PINIT
*****************************************************************/
int nInitializeHandle( int nHandle )
{
  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "PINIT %02X", nHandle);

  if (nSendMessage(szCommand,ADDCRC))
    return 1;
  if (nGetResponse())
    return 1;
  if (nCheckResponse(szReply,CHECKCRC))
    return 1;

  return 0;
} /* nInitializeHandle */

/*****************************************************************
Name: nEnableHandle

Inputs:
	int nHandle - the handle to be enabled

Return Value:
	int - 0 if passes, 1 if fails

Description:
        This routine enables specified handle by sending PENA
*****************************************************************/
int nEnableHandle(int nHandle)
{
  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "PENA %02X%c", nHandle, 'S');

  if (nSendMessage(szCommand,ADDCRC))
    return 1;
  if (nGetResponse())
    return 1;
  if (nCheckResponse(szReply,CHECKCRC))
    return 1;

  return 0;
} /* nEnablePort */

int nSetTrackingSensitivity(int nSens) {
  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "SET PS-0.Param.Tracking.Sensitivity=%d", nSens);
  //sprintf(szCommand, "GETINFO PS-0.Param.Tracking.Sensitivity");

  if (nSendMessage(szCommand,ADDCRC))
    return 1;
  if (nGetResponse())
    return 1;
  if (nCheckResponse(szReply,CHECKCRC))
    return 1;

  return 0;
} /* nEnablePort */


/*****************************************************************
Name: nLoadToolFile

Inputs:
	int nHandle - the handle to which to assign the file

Return Value:
	int - 0 if passes, 1 if fails

Description:
	This routine virtual loads a ROM file to the specified handle.
	It uses the PVWR command to do this.
*****************************************************************/
int nLoadToolFile(int nHandle)
{
  FILE *pFileHandle = NULL;

  int
    nBytes = 0,
    nCnt = 0,
    i = 0;

  static unsigned char uchBuff[1024];

  if (!(pFileHandle = fopen("/home/luke/polaris/8700339.rom","rb")))
    return 0;

  if ((nBytes = fread(uchBuff,1,sizeof(uchBuff),pFileHandle)) < 1)
    return 0;

  for (nCnt = 0; nCnt < nBytes;) {
    memset(szCommand, 0, sizeof(szCommand));
    sprintf(szCommand, "PVWR %02X%04X", nHandle, nCnt);

    for (i = 0; i < 64; i++, nCnt++) {
      /* plus 11 for PVWR XX0000_ */
      sprintf(szCommand + 11 + 2*i, "%02X", uchBuff[nCnt]);
    }

    if(nSendMessage(szCommand,ADDCRC))
      return 1;
    if (nGetResponse())
      return 1;
    if (nCheckResponse(szReply,CHECKCRC))
      return 1;
  }

  if(fclose(pFileHandle) != 0)
    return 1;

  return 0;
} /* nLoadToolFile */


int nBasicSetup()
{
  if (!nHardwareReset()) {
    sleep(.3);
    if (!nSetSystemCommParms(7,0,0,0,0)) {
      if (!nSetCompCommParms(7,0,0,0,0)) {
	if (!nInitializeSystem()) {
	  if (!nSetIllumRate(2)) {
	    if (!nRequestHandle()) {
	      if (!nLoadToolFile(1)) {
		if (!nInitializeHandle(1)) {
		  if (!nEnableHandle(1)) {
		    if (!nSetTrackingSensitivity(5)) {
		      return 0;
		    } else {
		      printf("nSetTrackingSensitivity Failed\n");
		    }
		  } else {
		    printf("nEnableHandle Failed\n");
		  }
		} else {
		  printf("nInitializeHandle Failed\n");
		}
	      } else {
		printf("nLoadToolFile Failed\n");
	      }
	    } else {
	      printf("nRequestHandle Failed\n");
	    }
	  } else {
	    printf("nSetIllumRate Failed\n");
	  }
	} else {
	  printf("nInitializeSystem Failed\n");
	}
      } else {
	printf("nSetCompCommParms Failed\n");
      }
    } else {
      printf("nSetSystemCommParms Failed\n");
    }
  } else {
    printf("nHardwareReset Failed\n");
  }
  return 1;
}

/*****************************************************************
Name: nStartTracking

Inputs:
	none

Return Value:
	int - 0 if passes, 1 if fails

Description:
    This routine starts tracking mode by sending TSTART
*****************************************************************/
int nStartTracking()
{
  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "TSTART ");

  if(nSendMessage(szCommand,ADDCRC))
    return 1;
  if (nGetResponse())
    return 1;
  if (nCheckResponse(szReply,CHECKCRC))
    return 1;

  return 0;
} /* nStartTracking */

/*****************************************************************
Name:				nGetPositionTX

Inputs:
    none

Return Value:
	int - 0 if passes, 1 if fails

Description:
	This routine gets the transformation information using the TX
	command.
*****************************************************************/
int nGetPositionTX()
{
  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "TX 1001"); //The 8 means return markers outside of characterized volume

  if(nSendMessage(szCommand,ADDCRC))
    return 1;
  if (nGetResponse())
    return 1;
  if (nCheckResponse(szReply,CHECKCRC))
    return 1;

  return 0;
}

int nTrackMarkerLoop()
{

  int frameGap = 0;
  int nFirstTime = 1;
  int flag;
  char sndBuf[MAX_UDP_MESSAGE];
  char rcvBuf[MAX_UDP_MESSAGE];

  int rb1x, rb1y, rb1z, rb2x, rb2y, rb2z, rbHalfExtentX, rbHalfExtentY, rbHalfExtentZ;

  while (1) {
    if (!nGetPositionTX()) {

      nParsePositionReply();

      // check whether we missed a frame
      if ((frameGap = currentPos->frame - previousPos->frame) != 1 && !nFirstTime) {
	fprintf(stderr,"Missed %d frame(s) starting at frame %d \n",frameGap-1, previousPos->frame);
      }

      // for plotting purposes
      if (udp_check(0)) {
      	/* Means QNX sent us some info regarding the position of the response "windows" */
      	memset(rcvBuf, 0, sizeof(rcvBuf));
      	udp_read(rcvBuf, sizeof(rcvBuf));
      	sscanf(rcvBuf, "%d %d %d %d %d %d %d %d %d %d",
	       &flag, &rb1x, &rb1y, &rb1z, &rb2x, &rb2y, &rb2z, &rbHalfExtentX, &rbHalfExtentY, &rbHalfExtentZ);
	if (flag == 1) {
	  currentPos->fixBoxx = (float) rb1x/10.;
	  currentPos->fixBoxy = (float) rb1y/10.;
	  currentPos->fixBoxz = (float) rb1z/10.;
	  currentPos->fixBoxHalfExtentX = (float) rbHalfExtentX/10.;
	  currentPos->fixBoxHalfExtentY = (float) rbHalfExtentY/10.;
	  currentPos->fixBoxHalfExtentZ = (float) rbHalfExtentZ/10.;
	}  else if (flag == 2) {
	  currentPos->responseBox1x = (float) rb1x/10.;
	  currentPos->responseBox1y = (float) rb1y/10.;
	  currentPos->responseBox1z = (float) rb1z/10.;
	  currentPos->responseBox2x = (float) rb2x/10.;
	  currentPos->responseBox2y = (float) rb2y/10.;
	  currentPos->responseBox2z = (float) rb2z/10.;
	  currentPos->responseBoxHalfExtentX = (float) rbHalfExtentX/10.;
	  currentPos->responseBoxHalfExtentY = (float) rbHalfExtentY/10.;
	  currentPos->responseBoxHalfExtentZ = (float) rbHalfExtentZ/10.;
	}
      }
      // to send to QNX/Rex
      memset(sndBuf, 0, sizeof(sndBuf));
      sprintf(sndBuf, "%10d %4.2f %4.2f %4.2f %1d", currentPos->frame, currentPos->x, currentPos->y, currentPos->z, currentPos->flagError);
      udp_send(sndBuf);

      // store previous frame's info
      previousPos->flagError = currentPos->flagError;
      previousPos->frame = currentPos->frame;
      previousPos->x = currentPos->x;
      previousPos->y = currentPos->y;
      previousPos->z = currentPos->z;

      if (nFirstTime)
	nFirstTime = 0;
    }
  }
}

/*****************************************************************
Name: nStopTracking

Inputs:
	none

Return Value:
	int - 0 if passes, 1 if fails

Description:
        This routine stops tracking mode by sending TSTOP
*****************************************************************/
int nStopTracking()
{
  memset(szCommand, 0, sizeof(szCommand));
  sprintf(szCommand, "TSTOP ");

  if(nSendMessage(szCommand,ADDCRC))
    return 1;
  if (nGetResponse())
    return 1;
  if (nCheckResponse(szReply,CHECKCRC))
    return 1;

  return 0;
} /* nStopTracking */

/*****************************************************************
Name: nSendMessage

Inputs:
        char *msg - the command to be sent
	int bCRC - if true, we add the CRC to the command

Return Value:
	int -  0 if passes, 1 if fails

Description:
	This command takes in a command string and parses it depending
	on the value of bCRC.  If bCRC is true, we replace the
	space with a : and calculate and add the CRC to the command.
	We then send the command to the system.
*****************************************************************/
int nSendMessage(char *msg, int bCRC)
{
  int n;

  /* build the command, by adding a carriage return to it and crc if specified */
  if (nBuildCommand(msg,bCRC))
    return 1;

  if(strlen(msg) >= (MAX_COMMAND))
    return 1;

  n = write(nPort, msg, strlen(msg));

  return 0;
} /* nSendMessage */

/*****************************************************************
Name:				nGetResponse

Inputs:
    none

Return Value:
	int - 0 if passes

Description:
	This routine gets the response from the system that is to be
	polled through the serial port.  If the end of response
    (Carriage Return) is found, the reply is complete and 0 is
    returned.
*****************************************************************/
int nGetResponseX()
{
  int bDone = 0;
  int nBytes = 0;
  int nInsert = 0;

  memset(szReply, 0, sizeof(szReply)); // Clear previous response

  while (!bDone) {
    nBytes = read(nPort, &szReply[nInsert], sizeof(szReply));
    nBytes += nInsert;
    if (szReply[nBytes-1]=='\r') {
      szReply[nBytes]='\0';
      bDone = 1;
    } else {
      nInsert = nBytes;
    }
  }
  printf("%s\n", szReply);
  return 0;
}

// Non-blocking version
int nGetResponse()
{
  int bDone = 0;
  int nBytes = 0;
  int nRead = 0;

  memset(szReply, 0, sizeof(szReply)); // Clear previous response

  while (!bDone) {
    nRead = read(nPort, &szReply[nBytes], sizeof(szReply));
    if (nRead < 1) // Nothing waiting for us yet
      continue;
    else
      nBytes += nRead;
    if (szReply[nBytes-1]=='\r') {
      szReply[nBytes]='\0';
      bDone = 1;
    }
  }
  //printf("%s\n", szReply);
  return 0;
}

/*****************************************************************
Name: nCheckResponse

Inputs:
    char *reply - the reply to be checked
    int bCRC - whether to check the CRC

Return Value:
	int - 0 if passes, 1 if fails

Description:    This routine makes sure we got a valid reply.
                It also checks the CRC if desired.
*****************************************************************/
int nCheckResponse(char *reply, int bCRC)
{

  /* return error if reply equal to RESET, ERROR or WARNING */
  if (!strncmp(reply, "RESET",5) ||
      !strncmp(reply, "ERROR",5) ||
      !strncmp(reply, "WARNING",7)) {
    printf("%s\n", reply);
    return 1;
  }

  if (bCRC)
    if (SystemCheckCRC(reply))
      return 1; // bad CRC

  return 0;
} /* nCheckResponse */

/*****************************************************************
Name: nCheckReset

Inputs:
        char *reply - the reply to be checked
Return Value:
	int - 0 if passes, 1 if fails

Description:    Upon sending serial break, make sure we got a RESET
*****************************************************************/
int nCheckReset (char *reply)
{
  if (!strncmp(reply, "RESET",5))
    return 0;
  else {
    printf("%s\n", reply);
    return 1;
  }
}

/*****************************************************************
Name: nParsePositionReply

Inputs:
        None
Return Value:
	0

Description:
        This routine is called from within the tracking loop.
       Every time it's called, it fills anew the global structure currentPos
*****************************************************************/
int nParsePositionReply () {

  int i, j;

  char hFrame[9];     // Hex string
  char hMarkers[3];   // Hex string
  char szPosition[9]; // 1 for sign, 6 for numbers, 1 for decimal point, 1 for \0

  int nVolBits;
  int nMarkers;
  int nOutVolume;
  int nInVolume;

  memset(hFrame, 0, sizeof(hFrame));
  memset(hMarkers, 0, sizeof(hMarkers));
  //  memset(szPosition, 0, sizeof(szPosition));

  /*
   * Parse the frame number first
   */
  strncpy(hFrame,strchr(szReply,'\n')-8,8);
  hFrame[8] = '\0';
  currentPos->frame = (int) strtol(hFrame, NULL, 16);

  /*
   * How many markers do we have?
   */
  strncpy(hMarkers,strchr(szReply,'\n')+1,2);
  hMarkers[2] = '\0';
  nMarkers = (int) strtol(hMarkers, NULL, 16);

  if (nMarkers == 0) {
    //printf("ZERO markers on frame %d\n",currentPos->frame);
    currentPos->flagError = 1;
    currentPos->x = previousPos->x;
    currentPos->y = previousPos->y;
    currentPos->z = previousPos->z;
    return 0;
  } else if (nMarkers > 8) {
    //printf("Too many phantom markers; NMarkers = %d on frame %d\n",nMarkers,currentPos->frame);
    currentPos->flagError = 1;
    currentPos->x = previousPos->x;
    currentPos->y = previousPos->y;
    currentPos->z = previousPos->z;
    return 0;
  } else {
    /* We can do something with this */
    char szOutVolume[nVolBits = (int) ceil(nMarkers/4.)];
    strncpy(szOutVolume,strchr(szReply,'\n')+3,nVolBits);
    nOutVolume = strtof(szOutVolume,NULL);
    nInVolume = nMarkers - nOutVolume;
  }

  /* This is the ideal scenario */
  if (nMarkers == 1 && nInVolume == 1) {
    currentPos->flagError = 0;
    for (i = 0; i < 3; i++) {
      memset(szPosition, 0, sizeof(szPosition));
      strncpy(szPosition,strchr(szReply,'\n')+4+i*7,5);
      szPosition[5]='.';
      strncpy(&szPosition[6],strchr(szReply,'\n')+4+i*7+5,2);
      szPosition[10]='\0';
      switch (i)
	{
	case 0:
	  currentPos->x = strtof(szPosition,NULL);
	case 1:
      	  currentPos->y = strtof(szPosition,NULL);
	case 2:
	  currentPos->z = strtof(szPosition,NULL);
	}
    }
  }
  /*
   *
   * This right here is dangerous but appears to work well in practice.
   *
   * In particular, Polaris detects "phantom" markers.
   * It's impossible to tell unambiguously which one's
   * real and which ones are not.
   *
   * Here's what we do. We compute the distance of each marker to the
   * previously reported position and take the one's that closer
   * as the position of the real one. This is sufficient for
   * online control. But make sure to flag these frames so that we
   * can better reconstruct trajectories offline.
   *
   */
  else if (nMarkers > 1 && nInVolume >= 1) {

    float x[nInVolume];
    float y[nInVolume];
    float z[nInVolume];
    float dist[nInVolume];
    int minIndex;
    int startOffset, markerOffset = 21;

    if (nVolBits == 1) {
      startOffset = 4;
    } else if (nVolBits == 2) {
      startOffset = 5;
    }

    currentPos->flagError = 2;

    for (j = 0; j < nInVolume; j++) {
      for (i = 0; i < 3; i++) {
  	memset(szPosition, 0, sizeof(szPosition));
  	strncpy(szPosition,strchr(szReply,'\n')+startOffset+j*markerOffset+i*7,5);
  	szPosition[5]='.';
  	strncpy(&szPosition[6],strchr(szReply,'\n')+startOffset+j*markerOffset+i*7+5,2);
  	szPosition[10]='\0';
  	switch (i)
  	  {
  	  case 0:
  	    x[j] = strtof(szPosition,NULL);
  	  case 1:
  	    y[j] = strtof(szPosition,NULL);
  	  case 2:
  	    z[j] = strtof(szPosition,NULL);
  	  }
      }
      dist[j] = pow(x[j]-previousPos->x,2) + pow(y[j]-previousPos->y,2) + pow(z[j]-previousPos->z,2);
    }

    /* Find index of minimum value */
    minIndex = 0;
    for(i=1; i < nInVolume; i++)
      if(dist[i] < dist[minIndex]) minIndex = i;

    currentPos->x = x[minIndex];
    currentPos->y = y[minIndex];
    currentPos->z = z[minIndex];
  }
  else if (nInVolume == 0) {
    currentPos->flagError = 1;
    currentPos->x = previousPos->x;
    currentPos->y = previousPos->y;
    currentPos->z = previousPos->z;
  }
  return 0;
}
