/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ISO14827-2"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#ifndef	_Cost_H_
#define	_Cost_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Cost */
typedef struct Cost {
	OCTET_STRING_t	 amount_Currency_code;
	long	 amount_Factor_quantity;
	long	 amount_Quantity_quantity;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Cost_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Cost;
extern asn_SEQUENCE_specifics_t asn_SPC_Cost_specs_1;
extern asn_TYPE_member_t asn_MBR_Cost_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _Cost_H_ */
#include <asn_internal.h>
