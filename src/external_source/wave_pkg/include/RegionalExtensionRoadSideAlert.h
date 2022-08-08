/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#ifndef	_RegionalExtensionRoadSideAlert_H_
#define	_RegionalExtensionRoadSideAlert_H_


#include <asn_application.h>

/* Including external dependencies */
#include "RegionId.h"
#include <ANY.h>
#include <asn_ioc.h>
#include "RSA-addGrpD.h"
#include <OPEN_TYPE.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RegionalExtensionRoadSideAlert__regExtValue_PR {
	RegionalExtensionRoadSideAlert__regExtValue_PR_NOTHING,	/* No components present */
	RegionalExtensionRoadSideAlert__regExtValue_PR_RSA_addGrpD
} RegionalExtensionRoadSideAlert__regExtValue_PR;

/* RegionalExtensionRoadSideAlert */
typedef struct RegionalExtensionRoadSideAlert {
	RegionId_t	 regionId;
	struct RegionalExtensionRoadSideAlert__regExtValue {
		RegionalExtensionRoadSideAlert__regExtValue_PR present;
		union {
			RSA_addGrpD_t	 RSA_addGrpD;
		};
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} regExtValue;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RegionalExtensionRoadSideAlert_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RegionalExtensionRoadSideAlert;
extern asn_SEQUENCE_specifics_t asn_SPC_RegionalExtensionRoadSideAlert_specs_1;
extern asn_TYPE_member_t asn_MBR_RegionalExtensionRoadSideAlert_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _RegionalExtensionRoadSideAlert_H_ */
#include <asn_internal.h>
