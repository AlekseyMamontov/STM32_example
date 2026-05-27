

/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif


	void CAN2_to_CAN1(void);
	void CAN1_to_CAN2(void);
	void FDCANs_Config(void);
	void Error_Handler(void);


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
