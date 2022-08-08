/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ISO14827-2"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#ifndef	_Subscription_H_
#define	_Subscription_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeInteger.h>
#include "SubscriptionType.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Subscription */
typedef struct Subscription {
	unsigned long	 datexSubscribe_Serial_nbr;
	SubscriptionType_t	 datexSubscribe_Type;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Subscription_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_datexSubscribe_Serial_nbr_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_Subscription;
extern asn_SEQUENCE_specifics_t asn_SPC_Subscription_specs_1;
extern asn_TYPE_member_t asn_MBR_Subscription_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _Subscription_H_ */
#include <asn_internal.h>
