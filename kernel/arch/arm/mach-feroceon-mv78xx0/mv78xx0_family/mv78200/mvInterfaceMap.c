#include "ctrlEnv/mvCtrlEnvLib.h"
#include "ctrlEnv/sys/mvCpuIf.h"
#include "cpu/mvCpu.h"
#include "boardEnv/mvBoardEnvLib.h"
#include "mv78200/mvInterfaceMap.h"


static MV_RES_MAP mv_res_table[] = {
	{0, "pcie0"},
	{0, "pcie1"},
	{0, "eth0"},
	{1, "eth1"},
	{0, "eth2"},
	{0, "eth3"},
	{0, "xor"},
	{0, "idma"},
	{0, "usb0"},
	{0, "usb1"},
	{0, "usb2"},
	{0, "cesa"},
	{0, "nor"},
	{0, "nand"},
	{0, "spi"},
	{0, "sata"},
	{-1, 0},
	};

/*
 * We override for the IDMA channel... DRI HACK
 */
int mvGetUnitMapping(MV_SOC_UNIT unit)
{
        if (unit == IDMA)
                return whoAmI();
	return mv_res_table[unit].cpuId;
}

MV_BOOL mvIsUnitMappedToThisCpu(MV_SOC_UNIT unit)
{
        if (unit == IDMA)
                return 1;
	return (mvGetUnitMapping(unit) == whoAmI());
}


MV_VOID mvSetUnitMapping(MV_SOC_UNIT unit, int cpuId)
{
	mv_res_table[unit].cpuId = cpuId;
}

MV_BOOL mvUnitMappingUnitIsIn(char *src, char *str)
{
	char *p = src;
	char *strPtr = str;

	if(*strPtr == '\0')
	{
		return MV_TRUE;
	}

	while( (*p != '\n') && (*p != '\0') && (*p != ' ') && (*strPtr != '\0') )
	{
		if (*strPtr == *p)
		{
			strPtr++;
			if(*strPtr == '\0')
			{
				/* Found the string */
				return MV_TRUE;
			}
		}
		else
		{
			strPtr = str;
		}
		
		p++;
	}
	return MV_FALSE;
}

MV_BOOL mvFillUnitMapTable(char* p, int cpuId)
{
	int i;
	for (i = 0; mv_res_table[i].cpuId != -1; i++) {
		if (mvUnitMappingUnitIsIn(p, mv_res_table[i].unitName)) {
			//printk("mvFillUnitMapTable: Index: %d, res: %s, old cpu: %d, new cpu: %d\n", i, p, mv_res_table[i].cpuId, cpuId);
			mv_res_table[i].cpuId = cpuId;
			//printk("mvFillUnitMapTable: res: %s, new cpu set: %d\n", p, mv_res_table[i].cpuId);
		}
	}
	return MV_TRUE;
}



