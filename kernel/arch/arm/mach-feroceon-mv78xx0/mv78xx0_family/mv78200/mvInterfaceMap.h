#ifndef mvresmgr_h
#define mvresmgr_h

typedef enum
{
	PEX00=0,
	PEX10,
	GIGA0,
	GIGA1,
	GIGA2,
	GIGA3,
	XOR,
	IDMA,
	USB0, 
	USB1, 
	USB2, 
	CESA,
	NOR_FLASH,
	NAND_FLASH,
	SPI_FLASH,
	SATA,
	MAX_UNITS
} MV_SOC_UNIT;


typedef struct __MV_RES_MAP 
{
	int cpuId;
	char* unitName;
} MV_RES_MAP;

MV_BOOL mvIsUnitMappedToThisCpu(MV_SOC_UNIT unit);
int mvGetUnitMapping(MV_SOC_UNIT unit);
MV_VOID mvSetUnitMapping(MV_SOC_UNIT unit, int cpuId);
MV_BOOL mvFillUnitMapTable(char* p, int cpuId);

#endif
