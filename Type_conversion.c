/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Type_conversion.c
 * 描述   :数据类型转换
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Type_conversion.h"


/* 浮点数到十六进制转换 */
/* floatNum：浮点型变量  */
/* byteArry：分解成4个字节的首地址  */
void FloatToByte(float floatNum,unsigned char* byteArry)
{
	int i;
	char* pchar=(char*)&floatNum;
	for(i=0;i<sizeof(float);i++)
	{
		*byteArry=*pchar;
		pchar++;
		byteArry++;
	}
}

void arrycat(u8 *dst,u8 index,u8 *src,u8 len)
{
	u8 i=0;
	for(i=0;i<len;i++)
	{
		*(dst+index+i)=*(src+i);
	}
}


