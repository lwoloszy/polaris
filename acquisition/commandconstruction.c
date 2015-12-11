#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "polaris.h"
#include "systemcrc.h"
#include "commandconstruction.h"

/*****************************************************************

Name: nAddCRCToCommand

Input Values:
	  char *pszCommandString - the message to have a CRC added

Output Values:
	  char *

Return Value:
	  int - 0 if passes, 1 if fails

Description:
	  This function adds a CRC to the end of a command
	  and replaces the space with a :.
*****************************************************************/
int nAddCRCToCommand(char *pszCommandString)
{
	int
		m,n, bFirstSpace = 0;
	unsigned int
		uCrc;

	if(strlen(pszCommandString) >= (MAX_COMMAND-6))
		return 0;

	n=strlen(pszCommandString);
	/*
	 * determine 16 bit CRC
	 */
	uCrc = 0;
	for(m=0;m<n;m++)
	{
		/*
		 * Replace space character with ':' if sending CRC.
		 * Since parameter names can have spaces we need to
		 * replace only the first space with ':'
		 */
		if(pszCommandString[m]==' ' && !bFirstSpace )
		{
			pszCommandString[m]=':';
			bFirstSpace = 1;
		}
		uCrc = CalcCrc16(uCrc,pszCommandString[m]);
	}
	sprintf(&pszCommandString[n],"%04X",uCrc);
	n+=4;

	return 0;
} /* nAddCRCToCommand */

/*****************************************************************
Name: nAddCRToCommand

Input Values:
	 char *pszCommandString - the message to have a carriage return added

Output Values:
	 char *

Return Value:
	 int - 0 if passes, 1 if fails

Description:
	 This function adds a carriage return  to the end of a command
*****************************************************************/
int nAddCRToCommand(char *pszCommandString)
{
  int n;

  if(strlen(pszCommandString) >= (MAX_COMMAND-1))
    return 1;

  n=strlen(pszCommandString);
  pszCommandString[n++] = '\r';
  pszCommandString[n++] = '\0';
  return 0;
} /* nAddCRToCommand */

/*****************************************************************

Name: nBuildCommand

Input Values:
	char *pszCommandString - the message to be built
	bool bAddCRC - whether or not to add the CRC to the command

Output Values:
	char *

Return Value:
	int - 0 if passes, 1 if fails

Description:
	This routine builds the message.  If bAddCRC is true, replace
	the space with a : and add the command's CRC to the end of it.
*****************************************************************/
int nBuildCommand(char *pszCommandString, int bAddCRC)
{
	if (bAddCRC) // Add CRC
	  if (nAddCRCToCommand( pszCommandString ))
		return 1;

	if (nAddCRToCommand( pszCommandString )) // Add CR
		return 1;

	return 0;
} /* nBuildCommand */
