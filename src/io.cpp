#include "io.h"
static int returnValue = 0;
static int config_check(int wkc)
{
	int success = 0;
	if(wkc==1)
	{
		returnValue ++;
		printf("config success, returnValue = %d\n", returnValue);
		success = 1;
	}
	else
		{
		std::cerr << "config error, returnValue = " << returnValue << "\n";
		success = 0;
	}
	return success;
}


int io_config()
{
	uint32 sdo_data32=0;
	uint16 sdo_data16=0;
	uint8 sdo_data8=0;
	void * p32 = (void *)(&sdo_data32);
	void * p16 = (void *)(&sdo_data16);
	void * p8 = (void *)(&sdo_data8);
	returnValue = 0;
	for(int cnt = 1; cnt <= ec_slavecount ; cnt++)
	{

		/* 0x1a07 mapping start */
		sdo_data8 = 0x0U;
		config_check(ec_SDOwrite(cnt, 0x1a07U, 0U, FALSE, 1, p8, EC_TIMEOUTRXM));

		sdo_data32 = 0x60640020U;
		config_check(ec_SDOwrite(cnt, 0x1a07U, 1U, FALSE, 4, p32, EC_TIMEOUTRXM));
		sdo_data32 = 0x606c0020U;
		config_check(ec_SDOwrite(cnt, 0x1a07U, 2U, FALSE, 4, p32, EC_TIMEOUTRXM));
		sdo_data32 = 0x60770010U;
		config_check(ec_SDOwrite(cnt, 0x1a07U, 3U, FALSE, 4, p32, EC_TIMEOUTRXM));
		sdo_data32 = 0x60410010U;
		config_check(ec_SDOwrite(cnt, 0x1a07U, 4U, FALSE, 4, p32, EC_TIMEOUTRXM));
		sdo_data32 = 0x60610008U;
		config_check(ec_SDOwrite(cnt, 0x1a07U, 5U, FALSE, 4, p32, EC_TIMEOUTRXM));

		sdo_data8 = 0x5U;
		config_check(ec_SDOwrite(cnt, 0x1a07U, 0U, FALSE, 1, p8, EC_TIMEOUTRXM));
		/* 0x1a07 mapping end */

		/* 0x1607 mapping start */
		sdo_data8 = 0x0U;
		config_check(ec_SDOwrite(cnt, 0x1607U, 0U, FALSE, 1, p8, EC_TIMEOUTRXM));

		sdo_data32 = 0x607A0020U;
		config_check(ec_SDOwrite(cnt, 0x1607U, 1U, FALSE, 4, p32, EC_TIMEOUTRXM));
		sdo_data32 = 0x60ff0020U;
		config_check(ec_SDOwrite(cnt, 0x1607U, 2U, FALSE, 4, p32, EC_TIMEOUTRXM));
		sdo_data32 = 0x60B10020U;
		config_check(ec_SDOwrite(cnt, 0x1607U, 3U, FALSE, 4, p32, EC_TIMEOUTRXM));
		sdo_data32 = 0x60710010U;
		config_check(ec_SDOwrite(cnt, 0x1607U, 4U, FALSE, 4, p32, EC_TIMEOUTRXM));
		sdo_data32 = 0x60B20010U;
		config_check(ec_SDOwrite(cnt, 0x1607U, 5U, FALSE, 4, p32, EC_TIMEOUTRXM));
		sdo_data32 = 0x60400010U;
		config_check(ec_SDOwrite(cnt, 0x1607U, 6U, FALSE, 4, p32, EC_TIMEOUTRXM));
		sdo_data32 = 0x60600008U;
		config_check(ec_SDOwrite(cnt, 0x1607U, 7U, FALSE, 4, p32, EC_TIMEOUTRXM));

		sdo_data8 = 0x7U;
		config_check(ec_SDOwrite(cnt, 0x1607U, 0U, FALSE, 1, p8, EC_TIMEOUTRXM));
		/* 0x1607 mapping end */
		/* 0x1c13 mapping start */
		sdo_data8 = 0x0U;
		config_check(ec_SDOwrite(cnt, 0x1c13U, 0U, FALSE, 1, p8, EC_TIMEOUTRXM));

		sdo_data16 = 0x1a07U;
		config_check(ec_SDOwrite(cnt, 0x1c13U, 1U, FALSE, 2, p16, EC_TIMEOUTRXM));

		sdo_data8 = 0x1U;
		config_check(ec_SDOwrite(cnt, 0x1c13U, 0U, FALSE, 1, p8, EC_TIMEOUTRXM));
		/* 0x1c13 mapping end */
		/* 0x1c12 mapping start */
		sdo_data8 = 0x0U;
		config_check(ec_SDOwrite(cnt, 0x1c12U, 0U, FALSE, 1, p8, EC_TIMEOUTRXM));

		sdo_data16 = 0x1607U;
		config_check(ec_SDOwrite(cnt, 0x1c12U, 1U, FALSE, 2, p16, EC_TIMEOUTRXM));

		sdo_data8 = 0x1U;
		config_check(ec_SDOwrite(cnt, 0x1c12U, 0U, FALSE, 1, p8, EC_TIMEOUTRXM));
		/* 0x1c12 mapping end */
	}
	int total_sdo_write_nums = 22;
	if(total_sdo_write_nums * ec_slavecount == returnValue)
		return 0;
	else return 1;
}
