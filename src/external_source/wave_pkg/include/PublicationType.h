/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ISO14827-2"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#ifndef	_PublicationType_H_
#define	_PublicationType_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include "MessageFrame.h"
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum PublicationType_PR {
	PublicationType_PR_NOTHING,	/* No components present */
	PublicationType_PR_datexPublication_Management_cd,
	PublicationType_PR_datexPublish_Data
} PublicationType_PR;
typedef enum PublicationType__datexPublication_Management_cd {
	PublicationType__datexPublication_Management_cd_temporarilySuspended	= 0,
	PublicationType__datexPublication_Management_cd_resume	= 1,
	PublicationType__datexPublication_Management_cd_terminate_other	= 2,
	PublicationType__datexPublication_Management_cd_terminate_dataNoLongerAvailable	= 3,
	PublicationType__datexPublication_Management_cd_terminate_publicationsBeingRejected	= 4,
	PublicationType__datexPublication_Management_cd_terminate_PendingShutdown	= 5,
	PublicationType__datexPublication_Management_cd_terminate_processingMgmt	= 6,
	PublicationType__datexPublication_Management_cd_terminate_bandwidthMgmt	= 7,
	PublicationType__datexPublication_Management_cd_terminate_accessDenied	= 8,
	PublicationType__datexPublication_Management_cd_unknownRequest	= 9
	/*
	 * Enumeration is extensible
	 */
} e_PublicationType__datexPublication_Management_cd;

/* PublicationType */
typedef struct PublicationType {
	PublicationType_PR present;
	union {
		long	 datexPublication_Management_cd;
		MessageFrame_t	 datexPublish_Data;
	};
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PublicationType_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_datexPublication_Management_cd_2;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_PublicationType;
extern asn_CHOICE_specifics_t asn_SPC_PublicationType_specs_1;
extern asn_TYPE_member_t asn_MBR_PublicationType_1[2];
extern asn_per_constraints_t asn_PER_type_PublicationType_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _PublicationType_H_ */
#include <asn_internal.h>
