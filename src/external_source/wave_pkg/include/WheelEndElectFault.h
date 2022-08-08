/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#ifndef	_WheelEndElectFault_H_
#define	_WheelEndElectFault_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum WheelEndElectFault {
	WheelEndElectFault_isOk	= 0,
	WheelEndElectFault_isNotDefined	= 1,
	WheelEndElectFault_isError	= 2,
	WheelEndElectFault_isNotSupported	= 3
} e_WheelEndElectFault;

/* WheelEndElectFault */
typedef long	 WheelEndElectFault_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_WheelEndElectFault_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_WheelEndElectFault;
extern const asn_INTEGER_specifics_t asn_SPC_WheelEndElectFault_specs_1;
asn_struct_free_f WheelEndElectFault_free;
asn_struct_print_f WheelEndElectFault_print;
asn_constr_check_f WheelEndElectFault_constraint;
ber_type_decoder_f WheelEndElectFault_decode_ber;
der_type_encoder_f WheelEndElectFault_encode_der;
xer_type_decoder_f WheelEndElectFault_decode_xer;
xer_type_encoder_f WheelEndElectFault_encode_xer;
oer_type_decoder_f WheelEndElectFault_decode_oer;
oer_type_encoder_f WheelEndElectFault_encode_oer;
per_type_decoder_f WheelEndElectFault_decode_uper;
per_type_encoder_f WheelEndElectFault_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _WheelEndElectFault_H_ */
#include <asn_internal.h>
