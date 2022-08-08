/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#ifndef	_PayloadData_H_
#define	_PayloadData_H_


#include <asn_application.h>

/* Including external dependencies */
#include <OCTET_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PayloadData */
typedef OCTET_STRING_t	 PayloadData_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PayloadData;
asn_struct_free_f PayloadData_free;
asn_struct_print_f PayloadData_print;
asn_constr_check_f PayloadData_constraint;
ber_type_decoder_f PayloadData_decode_ber;
der_type_encoder_f PayloadData_encode_der;
xer_type_decoder_f PayloadData_decode_xer;
xer_type_encoder_f PayloadData_encode_xer;
oer_type_decoder_f PayloadData_decode_oer;
oer_type_encoder_f PayloadData_encode_oer;
per_type_decoder_f PayloadData_decode_uper;
per_type_encoder_f PayloadData_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _PayloadData_H_ */
#include <asn_internal.h>
