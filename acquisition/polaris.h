#define DEVICE "/dev/ttyUSB0"

#define ADDCRC 0
#define CHECKCRC 0

#define MAX_COMMAND 1024
#define MAX_REPLY 4096
#define MAX_UDP_MESSAGE 256

#define thisIP "192.168.1.13"
#define otherIP "192.168.1.11"
#define udpPORT 6665

char szCommand[MAX_COMMAND];
char szReply[MAX_REPLY];
int  nPort;

typedef struct {
  int flagError;
  int frame;
  float x;
  float y;
  float z;

  /*
   *
   * For plotting response windows 
   * 
   */

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
POSITION_P currentPos, previousPos;

