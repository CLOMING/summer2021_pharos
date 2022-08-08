/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#include "NMEA-Revision.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static asn_oer_constraints_t asn_OER_type_NMEA_Revision_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_NMEA_Revision_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  0,  6 }	/* (0..6,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_NMEA_Revision_value2enum_1[] = {
	{ 0,	7,	"unknown" },
	{ 1,	8,	"reserved" },
	{ 2,	4,	"rev1" },
	{ 3,	4,	"rev2" },
	{ 4,	4,	"rev3" },
	{ 5,	4,	"rev4" },
	{ 6,	4,	"rev5" }
	/* This list is extensible */
};
static const unsigned int asn_MAP_NMEA_Revision_enum2value_1[] = {
	1,	/* reserved(1) */
	2,	/* rev1(2) */
	3,	/* rev2(3) */
	4,	/* rev3(4) */
	5,	/* rev4(5) */
	6,	/* rev5(6) */
	0	/* unknown(0) */
	/* This list is extensible */
};
const asn_INTEGER_specifics_t asn_SPC_NMEA_Revision_specs_1 = {
	asn_MAP_NMEA_Revision_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_NMEA_Revision_enum2value_1,	/* N => "tag"; sorted by N */
	7,	/* Number of elements in the maps */
	8,	/* Extensions before this member */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_NMEA_Revision_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_NMEA_Revision = {
	"NMEA-Revision",
	"NMEA-Revision",
	&asn_OP_NativeEnumerated,
	asn_DEF_NMEA_Revision_tags_1,
	sizeof(asn_DEF_NMEA_Revision_tags_1)
		/sizeof(asn_DEF_NMEA_Revision_tags_1[0]), /* 1 */
	asn_DEF_NMEA_Revision_tags_1,	/* Same as above */
	sizeof(asn_DEF_NMEA_Revision_tags_1)
		/sizeof(asn_DEF_NMEA_Revision_tags_1[0]), /* 1 */
	{ &asn_OER_type_NMEA_Revision_constr_1, &asn_PER_type_NMEA_Revision_constr_1, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_NMEA_Revision_specs_1	/* Additional specs */
};

