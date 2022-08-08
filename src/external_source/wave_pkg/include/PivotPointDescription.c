/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "../asn1c/J2735_201603DA+ITSK4-0.4_fix.update"
 * 	`asn1c -no-gen-example -fcompound-names -funnamed-unions -pdu=MessageFrame`
 */

#include "PivotPointDescription.h"

asn_TYPE_member_t asn_MBR_PivotPointDescription_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PivotPointDescription, pivotOffset),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Offset_B11,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"pivotOffset"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PivotPointDescription, pivotAngle),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_DSRC_Angle,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"pivotAngle"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PivotPointDescription, pivots),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_PivotingAllowed,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"pivots"
		},
};
static const ber_tlv_tag_t asn_DEF_PivotPointDescription_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_PivotPointDescription_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* pivotOffset */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* pivotAngle */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* pivots */
};
asn_SEQUENCE_specifics_t asn_SPC_PivotPointDescription_specs_1 = {
	sizeof(struct PivotPointDescription),
	offsetof(struct PivotPointDescription, _asn_ctx),
	asn_MAP_PivotPointDescription_tag2el_1,
	3,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_PivotPointDescription = {
	"PivotPointDescription",
	"PivotPointDescription",
	&asn_OP_SEQUENCE,
	asn_DEF_PivotPointDescription_tags_1,
	sizeof(asn_DEF_PivotPointDescription_tags_1)
		/sizeof(asn_DEF_PivotPointDescription_tags_1[0]), /* 1 */
	asn_DEF_PivotPointDescription_tags_1,	/* Same as above */
	sizeof(asn_DEF_PivotPointDescription_tags_1)
		/sizeof(asn_DEF_PivotPointDescription_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_PivotPointDescription_1,
	3,	/* Elements count */
	&asn_SPC_PivotPointDescription_specs_1	/* Additional specs */
};

