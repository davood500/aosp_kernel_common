#ifndef _BTL_INTERFACE_H_
#define _BTL_INTERFACE_H_

typedef int tRESULT;
typedef int ErrCode;
#ifndef VOID
typedef void VOID;
#endif
#ifndef BYTE
typedef char 	BYTE;
#endif

/* Fix me lator ??? */
//#define TICKS_PER_SECOND		500

typedef char CHAR;
typedef unsigned char	UINT8;
typedef unsigned short UINT16;
typedef unsigned int UINT;

//typedef void*				tTaskID;
//typedef OS_EVENT 	tMUTEX;
//typedef OS_EVENT 	tSemID;
	
typedef int INT;
//typedef unsigned int UINT;

typedef unsigned int				UINT32;
typedef signed int					INT32;
typedef signed short 				INT16;
typedef signed long long 		INT64;
typedef unsigned long long 	UINT64;

typedef signed short 				SHORT;
typedef unsigned short 			USHORT;
typedef signed long 				LONG;
typedef unsigned long 			ULONG;

typedef unsigned short WORD;
typedef unsigned long DWORD;

typedef float FLOAT;
typedef double DOUBLE;

#ifndef NULL
#define NULL 0
#endif

#ifndef BOOL
#define BOOL BYTE
#endif

#ifndef  FALSE
#define  FALSE                        0
#endif

#ifndef  TRUE
#define  TRUE                         1
#endif
  	
#endif // _BTL_INTERFACE_H_
