int nHardwareReset(void);
int nSetSystemCommParms(int nBaudRate, int nDataBits, int nParity,int nStopBits, int nHardware);
int nSetCompCommParms(int nBaudRate, int nDataBits, int nParity,int nStopBits, int nHardware);
int nBeepSystem(int nBeeps);
int nInitializeSystem(void);
int nSetIllumRate(int nIllumRate);
int nRequestHandle(void);
int nLoadToolFile(int nHandle);
int nInitializeHandle(int nHandle);
int nEnableHandle(int nHandle);
int nSetTrackingSensitivity(int nSens);
int nBasicSetup(void);
int nStartTracking(void);
int nGetPositionTX(void);
int nTrackMarkerLoop(void);
int nStopTracking(void);

int nSendMessage(char *msg, int bCRC);
int nGetResponse(void);
int nCheckResponse(char *reply, int bCRC);
int nCheckReset(char *reply);

int nParsePositionReply(void);
