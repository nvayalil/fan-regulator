/********************************************************************************
*               Type definition for HiTech C & MCC18 compliers                  *
*                                                                               *
*           Author          : C. V. Niras cvniras@gmail.com                     *
*           Last Modified   : 31/03/2008                                        *
*                                                                               *
*********************************************************************************/

#if defined(__18CXX)
// Defintions for MCC18 Compiler
typedef union _BYTE
{
    unsigned char _byte;
    struct
    {
        unsigned b0:1;
        unsigned b1:1;
        unsigned b2:1;
        unsigned b3:1;
        unsigned b4:1;
        unsigned b5:1;
        unsigned b6:1;
        unsigned b7:1;
    };
} BYTE;


typedef union _WORD
{
    unsigned int _word;
    struct
    {
        unsigned char byte0;
        unsigned char byte1;
    };
    struct
    {
        unsigned char v[2];
    };
} WORD;
#define LSB(a)      ((a).v[0])
#define MSB(a)      ((a).v[1])

typedef union _DWORD
{
    unsigned long int _dword;
    struct
    {
        unsigned char byte0;
        unsigned char byte1;
        unsigned char byte2;
        unsigned char byte3;
    };
    struct
    {
        unsigned int word0;
        unsigned int word1;
    };
    struct
    {
        unsigned char v[4];
    };
} DWORD;
#define LOWER_LSB(a)    ((a).v[0])
#define LOWER_MSB(a)    ((a).v[1])
#define UPPER_LSB(a)    ((a).v[2])
#define UPPER_MSB(a)    ((a).v[3])

#else


typedef union _BYTE
{
    unsigned char _byte;
    struct{
        unsigned _a0:1;
        unsigned _a1:1;
        unsigned _a2:1;
        unsigned _a3:1;
        unsigned _a4:1;
        unsigned _a5:1;
        unsigned _a6:1;
        unsigned _a7:1;
    }BITS;
} BYTE;

#define b0  BITS._a0
#define b1  BITS._a1
#define b2  BITS._a2
#define b3  BITS._a3
#define b4  BITS._a4
#define b5  BITS._a5
#define b6  BITS._a6
#define b7  BITS._a7

typedef union _WORD
{
    unsigned int _word;
    struct
    {
        unsigned char _byte0;
        unsigned char _byte1;
    }BYTES;
    unsigned char v[2];
} WORD;
#define LSB(a)      ((a).v[0])
#define MSB(a)      ((a).v[1])


typedef union _DWORD
{
    unsigned long int _dword;
    struct
    {
        unsigned char _byte0;
        unsigned char _byte1;
        unsigned char _byte2;
        unsigned char _byte3;
    }BYTES;
    struct
    {
        unsigned int _word0;
        unsigned int _word1;
    }WORDS;
    unsigned char v[4];
} DWORD;

#define byte0   BYTES._byte0
#define byte1   BYTES._byte1
#define byte2   BYTES._byte2
#define byte3   BYTES._byte3

#define word0   WORDS._word0
#define word1   WORDS._word1

#define LOWER_LSB(a)    ((a).v[0])
#define LOWER_MSB(a)    ((a).v[1])
#define UPPER_LSB(a)    ((a).v[2])
#define UPPER_MSB(a)    ((a).v[3])

#endif


