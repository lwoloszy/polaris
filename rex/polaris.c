#include  <stdio.h>
#include  <unistd.h>
#include  <sys/types.h>
#include  <x86/inline.h>
#include  "rexhdrs.h"
#include  "int.h"
#include  "polaris.h"
#include  "../sset/ldev.h"

static int previousFrame = 0;

/*
 * Turn off hand position checking
 */
int hwd_off(long hwdnum) {

	HWD *hwdp = &i_b->hwd[hwdnum];

	InterruptLock(&i_b->spinLock);
		{
			hwdp -> hwd_cntrl = 0;

			/* Turn off bits, which indicates hand within window */
			handflag &= ~(hwdp->hwd_xflag | hwdp->hwd_yflag | hwdp-> hwd_zflag);

			/* Unlink from active list */
			if (hwdp->hwd_back != NP) {
				hwdp->hwd_back->hwd_for = hwdp->hwd_for;
				hwdp->hwd_for->hwd_back = hwdp->hwd_back;
				hwdp->hwd_back = NP;
			}


		}
	InterruptUnlock(&i_b->spinLock);

	return(0);
}

/*
 * Turn on hand position checking
 */
int hwd_on(long hwdnum) {

	HWD *hwdp = &i_b->hwd[hwdnum];

	InterruptLock(&i_b->spinLock);
		{
			hwdp -> hwd_cntrl = 1;

			/* Link window into active list at end */
			if (hwdp->hwd_back == NP) {
				hwdp->hwd_back = i_b->hwd_last.hwd_back;
				hwdp->hwd_back->hwd_for = hwdp;
				hwdp -> hwd_for = &i_b->hwd_last;
				i_b -> hwd_last.hwd_back = hwdp;
			}
		}
	InterruptUnlock(&i_b->spinLock);

	return(0);
}

/*
 * Set extent of window, which is a cube (or rectangular cube).
 */
int hwd_setSize(long hwdnum, long x, long y, long z) {

	HWD *hwdp = &i_b->hwd[hwdnum];

	InterruptLock(&i_b->spinLock);
		{
			hwdp -> hwd_xhalfextent = x;
			hwdp -> hwd_yhalfextent = y;
			hwdp -> hwd_zhalfextent = z;
		}
	InterruptUnlock(&i_b->spinLock);

	return(0);
}

/*
 * Set position of window in mm*10
 */
int hwd_setPosition(long hwdnum, long xpos, long ypos, long zpos) {

	HWD *hwdp = &i_b->hwd[hwdnum];

	InterruptLock(&i_b->spinLock);
		{
			hwdp->hwd_xpos = (int) xpos;
			hwdp->hwd_ypos = (int) ypos;
			hwdp->hwd_zpos = (int) zpos;
		}
	InterruptUnlock(&i_b->spinLock);

	return(0);
}


/*
 * Set position of window in degrees (transform to mm*10)
 * Z axis does not make sense here
 */
int hwd_setPositionDegrees(long hwdnum, long xdeg, long ydeg) {

	float deg_per_cm, cm_per_deg;
	int xpos, ypos;

//	deg_per_cm = 2.*atan(1./(2.*VIEW_DIST_CM))*180./PI;
//	cm_per_deg = 1./deg_per_cm;

//	xpos = (int) round(tan(xdeg/10./2.*PI/180.)*2*MONITOR_DISTANCE);
//	ypos = (int) round(tan(ydeg/10./2.*PI/180.)*2*MONITOR_DISTANCE);

	xpos = (int) round(xdeg/10.*CM_PER_DEG*100); /* in mm*10 */
	ypos = (int) round(ydeg/10.*CM_PER_DEG*100); /* in mm*10 */

	HWD *hwdp = &i_b->hwd[hwdnum];

	InterruptLock(&i_b->spinLock);
		{
			hwdp -> hwd_xpos = xpos;
			hwdp -> hwd_ypos = ypos;
			hwdp -> hwd_zpos = 0;
		}
	InterruptUnlock(&i_b->spinLock);

	return(0);
}

/*
 * Init hand window struct.
 */
int hwd_init(void) {

	HWD *hwdp;
	int i;

	i_b->hwd_first.hwd_back = NP;
	i_b->hwd_first.hwd_for = &i_b->hwd_last;
	i_b->hwd_last.hwd_back = &i_b->hwd_first;
	i_b->hwd_last.hwd_for = NP;


	InterruptLock(&i_b->spinLock);
		{
			for (i = 0; i < HWD_MAXNUM; i++) {

				HWD *hwdp = &i_b -> hwd[i];

				hwdp -> hwd_cntrl = 0;

				hwdp -> hwd_xpos = 0;
				hwdp -> hwd_ypos = 0;
				hwdp -> hwd_zpos = 0;

				hwdp -> hwd_xhalfextent = 0;
				hwdp -> hwd_yhalfextent = 0;
				hwdp -> hwd_zhalfextent = 0;

				hwdp -> hwd_xflag = (01 << (i*3));
				hwdp -> hwd_yflag = (02 << (i*3));
				hwdp -> hwd_zflag = (04 << (i*3));

				hwdp -> hwd_for = NP;
				hwdp -> hwd_back = NP;
			}
		}
	InterruptUnlock(&i_b->spinLock);

	return(0);
}

void inputPolaris(char *buf)
{
	POLARISDATA *poldatp;
	int i;
	int frameNum, flagError;
	float x, y, z;

	sscanf(buf, "%d %f %f %f %d",
		   &frameNum,
		   &x, &y, &z,
		   &flagError);

	poldatp = &i_b -> polarisdata;

	InterruptLock(&i_b->spinLock);
		{
			/*
			 * Multiply by 10 and cast to int becuase of interrupt handler
			 * These are the units we will be working with in QNX.
			 *
			 * We are decreasing 1/100 mm resolution down to 1/10 mm.
			 *
			 * y (polaris) is x (Rex)
			 * z (polaris) is y (Rex)
			 * x (polaris) is z (Rex)
			 *
			 */
			poldatp->x = (int) -1*(round(y*10)-POLARISY_CENTER);
			poldatp->y = (int) round(z*10)-POLARISZ_CENTER;
			poldatp->z = (int) round(x*10)-POLARISX_CENTER;
		}
	InterruptUnlock(&i_b->spinLock);

	if (!(previousFrame == frameNum)) {
		if (frameNum-previousFrame > 1) {
			printf("Missed %d Frames\n", frameNum - previousFrame - 1);
		}
		loadPolarisPlexonBuf(frameNum, flagError,
							 poldatp->x,
							 poldatp->y,
							 poldatp->z);
		previousFrame = frameNum;
	}
}

void loadPolarisPlexonBuf(int f, int err, int x, int y, int z) {

	typedef unsigned short us;
	us usx, usy, usz; /* position */
	us usf;           /* frame number */
	us userr;         /* Is it valid data or has it been massaged on Linux side? */

	/* We have 15 bits with which to discretize the position signals, that is, 32768 unique values */
	int resolution = 32768;
	int halfRes = resolution/2;

	usx = (us) (x+halfRes) % resolution;
	usy = (us) (y+halfRes) % resolution;
	usz = (us) (z+halfRes) % resolution;

	usf = (us) f % resolution;
	userr = (us) err;

	int Error=0;

	InterruptLock(&i_b->spinLock);
		{
			i_b->plpol_buf[i_b->plpol_buflx] = usf;
			if(++i_b->plpol_buflx >= PLPOL_MAXPOSITIONS) i_b->plpol_buflx = 0;
			if(i_b->plpol_buflx == i_b->plpol_bufdx) Error = 1;

			i_b->plpol_buf[i_b->plpol_buflx] = userr;
			if(++i_b->plpol_buflx >= PLPOL_MAXPOSITIONS) i_b->plpol_buflx = 0;
			if(i_b->plpol_buflx == i_b->plpol_bufdx) Error = 1;

			i_b->plpol_buf[i_b->plpol_buflx] = usx;
			if(++i_b->plpol_buflx >= PLPOL_MAXPOSITIONS) i_b->plpol_buflx = 0;
			if(i_b->plpol_buflx == i_b->plpol_bufdx) Error = 1;

			i_b->plpol_buf[i_b->plpol_buflx] = usy;
			if(++i_b->plpol_buflx >= PLPOL_MAXPOSITIONS) i_b->plpol_buflx = 0;
			if(i_b->plpol_buflx == i_b->plpol_bufdx) Error = 1;

			i_b->plpol_buf[i_b->plpol_buflx] = usz;
			if(++i_b->plpol_buflx >= PLPOL_MAXPOSITIONS) i_b->plpol_buflx = 0;
			if(i_b->plpol_buflx == i_b->plpol_bufdx) Error = 1;
		}

	InterruptUnlock(&i_b->spinLock);


	if (Error) {
		rxerr("Plexon Polaris buffer overflow");
	}
}
